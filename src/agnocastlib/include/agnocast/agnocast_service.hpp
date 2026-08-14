#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/internal/service_wire_type.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace agnocast
{

// Forward declaration
namespace detail
{
template <typename>
class ResponsePublisherManager;
}  // namespace detail

enum class ServiceRole : uint8_t {
  /// User-created service; issues an R2A bridge request.
  Default,
  /// Used by the bridge plugin's own service; no bridge request is issued.
  /// Not intended for direct use by application code.
  AgnocastOnly,
};

class ServiceBase
{
  template <typename>
  friend class detail::ResponsePublisherManager;

protected:
  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  std::string service_name_;
  const rclcpp::QoS qos_;

  template <typename NodeT>
  explicit ServiceBase(NodeT * node, const std::string & service_name, const rclcpp::QoS & qos)
  : node_(node), qos_(rclcpp::QoS(qos).durability_volatile())
  {
    service_name_ = node->get_node_services_interface()->resolve_service_name(service_name);
  }

public:
  /// @brief Return the resolved service name.
  /// @return Null-terminated service name string.
  AGNOCAST_PUBLIC
  const char * get_service_name() const { return service_name_.c_str(); }

  virtual ~ServiceBase() = default;
};

namespace detail
{

// TODO(bdm-k): Implement reaping of publishers that are no longer needed (Issue #1355).
template <typename PublisherT>
class ResponsePublisherManager
{
  std::mutex mtx_;
  std::unordered_map<std::string, typename PublisherT::SharedPtr> pubs_;

public:
  typename PublisherT::SharedPtr get_or_create_publisher_for(
    const ServiceBase * service_base, const std::string & node_name)
  {
    typename PublisherT::SharedPtr pub;
    {
      std::lock_guard<std::mutex> lock(mtx_);

      auto it = pubs_.find(node_name);
      if (it == pubs_.end()) {
        std::visit(
          [&](auto * node) {
            std::string topic_name =
              create_service_response_topic_name(service_base->service_name_, node_name);
            agnocast::PublisherOptions pub_options;
            if constexpr (std::is_same_v<PublisherT, agnocast::TypeErasedPublisher>) {
              pub = std::make_shared<PublisherT>(
                node, topic_name, "", service_base->qos_, pub_options, PublisherRole::AgnocastOnly);
            } else {
              pub = std::make_shared<PublisherT>(
                node, topic_name, service_base->qos_, pub_options, PublisherRole::AgnocastOnly);
            }
            pubs_[node_name] = pub;
          },
          service_base->node_);
      } else {
        pub = it->second;
      }
    }
    return pub;
  }
};

}  // namespace detail

/**
 * @brief Service server for zero-copy Agnocast service communication.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
template <typename ServiceT>
class Service : public ServiceBase, public std::enable_shared_from_this<Service<ServiceT>>
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
  struct is_deferred_cb : std::bool_constant<std::is_invocable_v<
                            std::decay_t<Func>, std::shared_ptr<Service<ServiceT>>,
                            ipc_shared_ptr<typename ServiceT::Request> &&>>
  {
  };

  using RequestT = ServiceRequestWrapper<ServiceT>;
  using ResponseT = ServiceResponseWrapper<ServiceT>;

  detail::ResponsePublisherManager<Publisher<ResponseT>> pub_manager_;
  typename Subscription<RequestT>::SharedPtr subscriber_;

  template <typename Func>
  auto wrap_basic_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<RequestT> && request) {
      auto publisher = pub_manager_.get_or_create_publisher_for(this, request->node_name);

      ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
      response->seqno = request->seqno;

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

  template <typename Func, typename NodeT>
  void constructor_impl(
    NodeT * node, Func && callback, rclcpp::CallbackGroup::SharedPtr group, ServiceRole role)
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
      subscriber_ = std::make_shared<Subscription<RequestT>>(
        node, topic_name, qos_,
        wrap_basic_service_callback_for_subscriber(std::forward<Func>(callback)), options,
        SubscriptionRole::AgnocastOnly);
    } else if constexpr (is_deferred_cb<Func>::value) {
      subscriber_ = std::make_shared<Subscription<RequestT>>(
        node, topic_name, qos_,
        wrap_deferred_service_callback_for_subscriber(std::forward<Func>(callback)), options,
        SubscriptionRole::AgnocastOnly);
    }

    if (role == ServiceRole::Default) {
      std::optional<std::pair<std::string, std::string>> shadow_node_identity{std::nullopt};
      if constexpr (std::is_same_v<std::remove_cv_t<NodeT>, agnocast::Node>) {
        shadow_node_identity =
          std::make_pair(std::string(node->get_namespace()), std::string(node->get_name()));
      }
      register_service_bridge(
        rosidl_generator_traits::name<ServiceT>(), service_name_, BridgeDirection::ROS2_TO_AGNOCAST,
        shadow_node_identity);
    }
  }

public:
  using SharedPtr = std::shared_ptr<Service<ServiceT>>;

  template <typename Func>
  Service(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group,
    ServiceRole role = ServiceRole::Default)
  : ServiceBase(node, service_name, qos)
  {
    constructor_impl(node, std::forward<Func>(callback), group, role);
  }

  template <typename Func>
  Service(
    agnocast::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group,
    ServiceRole role = ServiceRole::Default)
  : ServiceBase(node, service_name, qos)
  {
    constructor_impl(node, std::forward<Func>(callback), group, role);
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
  AGNOCAST_PUBLIC
  void send_response(
    ipc_shared_ptr<typename ServiceT::Request> && request,
    ipc_shared_ptr<typename ServiceT::Response> && response)
  {
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    auto internal_response = static_ipc_shared_ptr_cast<ResponseT>(std::move(response));
    auto publisher = pub_manager_.get_or_create_publisher_for(this, internal_request->node_name);
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
  AGNOCAST_PUBLIC
  ipc_shared_ptr<typename ServiceT::Response> borrow_loaned_response(
    const ipc_shared_ptr<typename ServiceT::Request> & request)
  {
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(request);
    auto publisher = pub_manager_.get_or_create_publisher_for(this, internal_request->node_name);
    ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
    response->seqno = internal_request->seqno;
    return ipc_shared_ptr<typename ServiceT::Response>(std::move(response));
  }

  const char * get_service_name() const { return service_name_.c_str(); }
};

/**
 * @brief Generic service server for zero-copy Agnocast service communication.
 *
 * The service type is supplied as a runtime string, rather than a compile-time template argument.
 * If the given service type is invalid, the constructor will throw an exception.
 *
 * The usage is mostly the same as agnocast::Service. One difference is cancel_response(), which
 * only exists in GenericService. This is relevant for deferred callbacks. The user must either call
 * send_response() or cancel_response() for every response borrowed via borrow_loaned_response().
 * Otherwise, the process will terminate.
 */
class GenericService : public ServiceBase, public std::enable_shared_from_this<GenericService>
{
  template <typename Func>
  struct is_basic_cb
  : std::bool_constant<
      std::is_invocable_v<std::decay_t<Func>, ipc_shared_ptr<void> &&, ipc_shared_ptr<void> &&>>
  {
  };
  template <typename Func>
  struct is_deferred_cb
  : std::bool_constant<std::is_invocable_v<
      std::decay_t<Func>, std::shared_ptr<GenericService>, ipc_shared_ptr<void> &&>>
  {
  };

  detail::ResponsePublisherManager<TypeErasedPublisher> pub_manager_;
  typename Subscription<void>::SharedPtr subscriber_;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members_{nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_{nullptr};

  template <typename Func>
  auto wrap_basic_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<void> && request) {
      auto req_wrapper = GenericRequestWrapper(request_members_, std::move(request));
      auto publisher = pub_manager_.get_or_create_publisher_for(this, req_wrapper.node_name());

      auto res_wrapper = GenericResponseWrapper::allocate(
        response_members_,
        [&publisher](size_t size) { return publisher->borrow_loaned_message(size); });
      res_wrapper.seqno() = req_wrapper.seqno();

      ipc_shared_ptr<void> response = std::move(res_wrapper).take_response();
      ipc_shared_ptr<void> response_double(response);

      // If the callback throws, we destroy the `response` (ipc_shared_ptr<void>) via
      // cancel_message() to prevent ipc_shared_ptr::reset() from calling std::terminate(), and then
      // rethrow. We only need to destroy `response`, not `response_double`:
      // (1) If `response_double` was moved from, it is empty and does not need to be destroyed.
      // (2) If `response_double` was not moved from, it will be invalidated when `response` is
      //     destroyed.
      try {
        callback(std::move(req_wrapper).take_request(), std::move(response_double));
      } catch (...) {
        publisher->cancel_message(std::move(response), [this](void * p) {
          GenericResponseWrapper::free(p, this->response_members_);
        });
        throw;
      }

      publisher->publish(std::move(response), [this](void * p) {
        GenericResponseWrapper::free(p, this->response_members_);
      });

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
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<void> && request) {
      callback(this->shared_from_this(), std::move(request));
    };
  }

  void load_typesupport_impl(const std::string & service_type);

  template <typename Func, typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_type, Func && callback,
    const rclcpp::CallbackGroup::SharedPtr & group, ServiceRole role)
  {
    static_assert(
      is_basic_cb<Func>::value || is_deferred_cb<Func>::value,
      "Callback must be callable with one of the following argument pairs:\n"
      "1. basic: (ipc_shared_ptr<void>, ipc_shared_ptr<void>)\n"
      "2. deferred: (std::shared_ptr<GenericService>, ipc_shared_ptr<void>)\n"
      "ipc_shared_ptr arguments can be received by const&, &&, or by value");

    load_typesupport_impl(service_type);

    SubscriptionOptions sub_options{group};
    std::string req_topic_name = create_service_request_topic_name(service_name_);
    if constexpr (is_basic_cb<Func>::value) {
      subscriber_ = std::make_shared<Subscription<void>>(
        node, req_topic_name, "", qos_,
        wrap_basic_service_callback_for_subscriber(std::forward<Func>(callback)), sub_options,
        SubscriptionRole::AgnocastOnly);
    } else if constexpr (is_deferred_cb<Func>::value) {
      subscriber_ = std::make_shared<Subscription<void>>(
        node, req_topic_name, "", qos_,
        wrap_deferred_service_callback_for_subscriber(std::forward<Func>(callback)), sub_options,
        SubscriptionRole::AgnocastOnly);
    }

    if (role == ServiceRole::Default) {
      std::optional<std::pair<std::string, std::string>> shadow_node_identity{std::nullopt};
      if constexpr (std::is_same_v<std::remove_cv_t<NodeT>, agnocast::Node>) {
        shadow_node_identity =
          std::make_pair(std::string(node->get_namespace()), std::string(node->get_name()));
      }
      register_service_bridge(
        service_type, service_name_, BridgeDirection::ROS2_TO_AGNOCAST, shadow_node_identity);
    }
  }

public:
  using SharedPtr = std::shared_ptr<GenericService>;

  template <typename Func>
  GenericService(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group,
    ServiceRole role = ServiceRole::Default)
  : ServiceBase(node, service_name, qos)
  {
    constructor_impl(node, service_type, std::forward<Func>(callback), group, role);
  }

  template <typename Func>
  GenericService(
    agnocast::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group,
    ServiceRole role = ServiceRole::Default)
  : ServiceBase(node, service_name, qos)
  {
    constructor_impl(node, service_type, std::forward<Func>(callback), group, role);
  }

  void send_response(ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response);

  void cancel_response(ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response);

  ipc_shared_ptr<void> borrow_loaned_response(const ipc_shared_ptr<void> & request);

  const char * get_service_name() const { return service_name_.c_str(); }
};

}  // namespace agnocast
