#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace agnocast
{

// Internal implementation - users should use agnocast::Service<ServiceT> instead.
template <typename ServiceT, typename BridgeRequestPolicy>
class BasicService
: public std::enable_shared_from_this<BasicService<ServiceT, BridgeRequestPolicy>>
{
private:
  // TODO(bdm-k): Consider supporting callbacks that take lvalue references.
  template <typename Func>
  struct is_basic_cb : std::bool_constant<std::is_invocable_v<
                         std::decay_t<Func>, ipc_shared_ptr<typename ServiceT::Request> &&,
                         ipc_shared_ptr<typename ServiceT::Response> &&>>
  {
  };
  template <typename Func>
  struct is_deferred_cb
  : std::bool_constant<std::is_invocable_v<
      std::decay_t<Func>, std::shared_ptr<BasicService<ServiceT, BridgeRequestPolicy>>,
      ipc_shared_ptr<typename ServiceT::Request> &&>>
  {
  };

  // To avoid name conflicts, members of RequestT and ResponseT are given an underscore prefix.
  struct RequestT : public ServiceT::Request
  {
    std::string _node_name;
    int64_t _sequence_number;
  };
  struct ResponseT : public ServiceT::Response
  {
    int64_t _sequence_number;
  };

  using ServiceResponsePublisher = BasicPublisher<ResponseT, NoBridgeRequestPolicy>;
  using ServiceRequestSubscriber = BasicSubscription<RequestT, NoBridgeRequestPolicy>;

  std::variant<rclcpp::Node *, agnocast::Node *> node_;
  const std::string service_name_;
  const rclcpp::QoS qos_;
  std::mutex publishers_mtx_;
  std::unordered_map<std::string, typename ServiceResponsePublisher::SharedPtr> publishers_;
  typename ServiceRequestSubscriber::SharedPtr subscriber_;

  typename ServiceResponsePublisher::SharedPtr get_or_create_publisher_for(
    const std::string & node_name)
  {
    typename ServiceResponsePublisher::SharedPtr pub;
    {
      std::lock_guard<std::mutex> lock(publishers_mtx_);
      auto it = publishers_.find(node_name);
      if (it == publishers_.end()) {
        std::visit(
          [this, &pub, &node_name](auto * node) {
            std::string topic_name = create_service_response_topic_name(service_name_, node_name);
            agnocast::PublisherOptions pub_options;
            pub = std::make_shared<ServiceResponsePublisher>(node, topic_name, qos_, pub_options);
            publishers_[node_name] = pub;
          },
          node_);
      } else {
        pub = it->second;
      }
    }
    return pub;
  }

  template <typename Func>
  auto wrap_basic_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<RequestT> && request) {
      auto publisher = this->get_or_create_publisher_for(request->_node_name);

      ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
      response->_sequence_number = request->_sequence_number;

      ipc_shared_ptr<typename ServiceT::Response> response_double(response);

      callback(
        ipc_shared_ptr<typename ServiceT::Request>(std::move(request)), std::move(response_double));

      publisher->publish(std::move(response));

      // Safety regarding response_double
      //   When `response` is published, all references that share its control block are
      //   invalidated. Since `response_double` shares its control block with `response`,
      //   dereferencing `response_double` after publication is disallowed, preventing accidental
      //   (and erroneous) writes to the response via `response_double`.
    };
  }

  template <typename Func>
  auto wrap_deferred_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<RequestT> && request) {
      callback(this->shared_from_this(), std::move(request));
    };
  }

public:
  using SharedPtr = std::shared_ptr<BasicService<ServiceT, BridgeRequestPolicy>>;

  template <typename Func, typename NodeT>
  BasicService(
    NodeT * node, const std::string & service_name, Func && callback, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : node_(node),
    service_name_(node->get_node_services_interface()->resolve_service_name(service_name)),
    // TransientLocal durability is not allowed for services.
    qos_(rclcpp::QoS(qos).durability_volatile())
  {
    static_assert(
      is_basic_cb<Func>::value || is_deferred_cb<Func>::value,
      "Callback must be callable with one of the following argument pairs:\n"
      "1. basic: (ipc_shared_ptr<ServiceT::Request>, ipc_shared_ptr<ServiceT::Response>)\n"
      "2. deferred: (std::shared_ptr<Service>, ipc_shared_ptr<ServiceT::Request>)\n"
      "ipc_shared_ptr arguments can be received by const&, &&, or by value");

    SubscriptionOptions options{group};
    std::string topic_name = create_service_request_topic_name(service_name_);
    if constexpr (is_basic_cb<Func>::value) {
      subscriber_ = std::make_shared<ServiceRequestSubscriber>(
        node, topic_name, qos_,
        wrap_basic_service_callback_for_subscriber(std::forward<Func>(callback)), options);
    } else if constexpr (is_deferred_cb<Func>::value) {
      subscriber_ = std::make_shared<ServiceRequestSubscriber>(
        node, topic_name, qos_,
        wrap_deferred_service_callback_for_subscriber(std::forward<Func>(callback)), options);
    }

    BridgeRequestPolicy::template request_bridge<NodeT, ServiceT>(node, service_name_);
  }

  /**
   * @brief Sends a response to the client that initiated the service call. This function is
   * expected to be used in deferred response callbacks.
   *
   * `response` must be the object returned by `borrow_loaned_response()`. The entire
   * `borrow_loaned_response()` -> populate -> `send_response()` sequence must run on the same
   * thread (typically in a single callback).
   *
   * @param request The request that initiated the service call.
   * @param response The response to send. Must be acquired by calling borrow_loaned_response().
   */
  // AGNOCAST_PUBLIC
  void send_response(
    ipc_shared_ptr<typename ServiceT::Request> && request,
    ipc_shared_ptr<typename ServiceT::Response> && response)
  {
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    auto internal_response = static_ipc_shared_ptr_cast<ResponseT>(std::move(response));
    auto publisher = get_or_create_publisher_for(internal_request->_node_name);
    publisher->publish(std::move(internal_response));
  }

  /**
   * @brief Allocate a service response message in shared memory. This function is expected to be
   * used in deferred response callbacks.
   *
   * This function does not consume `request`. In deferred callbacks, keep `request` and pass it to
   * `send_response()` after populating the returned response.
   *
   * @param request The request that initiated the service call.
   * @return Owned pointer to the response message in shared memory.
   */
  // AGNOCAST_PUBLIC
  ipc_shared_ptr<typename ServiceT::Response> borrow_loaned_response(
    const ipc_shared_ptr<typename ServiceT::Request> & request)
  {
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(request);
    auto publisher = get_or_create_publisher_for(internal_request->_node_name);
    ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
    response->_sequence_number = internal_request->_sequence_number;
    return ipc_shared_ptr<typename ServiceT::Response>(std::move(response));
  }
};

struct RosToAgnocastServiceRequestPolicy;

/**
 * @brief The user-facing Agnocast service server.
 * Alias for `BasicService<ServiceT>`. Use this type (not BasicService directly) when declaring
 * service server variables.
 * The service/client API is experimental and may change in future versions.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
// AGNOCAST_PUBLIC
template <typename ServiceT>
using Service = BasicService<ServiceT, RosToAgnocastServiceRequestPolicy>;

}  // namespace agnocast
