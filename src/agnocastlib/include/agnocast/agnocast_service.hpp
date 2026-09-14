#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_service_event_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/internal/service_typesupport.hpp"
#include "agnocast/internal/service_wire_type.hpp"
#include "rclcpp/rclcpp.hpp"

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
#include <service_msgs/msg/service_event_info.hpp>
#endif

#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace agnocast
{

enum class ServiceRole : uint8_t {
  /// User-created service; issues an R2A bridge request.
  Default,
  /// Used by the bridge implementation itself; marks the endpoints it creates as bridges in kmod
  /// and issues no bridge request. A bridge service shares the request topic with a real one, so
  /// that mark is what tells them apart.
  /// Not intended for direct use by application code.
  BridgeInternal,
};

constexpr SubscriptionRole to_subscription_role(const ServiceRole role)
{
  return role == ServiceRole::BridgeInternal ? SubscriptionRole::BridgeInternal
                                             : SubscriptionRole::AgnocastOnly;
}

constexpr PublisherRole to_publisher_role(const ServiceRole role)
{
  return role == ServiceRole::BridgeInternal ? PublisherRole::BridgeInternal
                                             : PublisherRole::AgnocastOnly;
}

// Internal implementation - users should use agnocast::Service<ServiceT> instead.
template <typename ServiceT>
class BasicService : public std::enable_shared_from_this<BasicService<ServiceT>>
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
                            std::decay_t<Func>, std::shared_ptr<BasicService<ServiceT>>,
                            ipc_shared_ptr<typename ServiceT::Request> &&>>
  {
  };

  using RequestT = ServiceRequestWrapper<ServiceT>;
  using ResponseT = ServiceResponseWrapper<ServiceT>;

  using ServiceResponsePublisher = Publisher<ResponseT>;
  using ServiceRequestSubscriber = Subscription<RequestT>;

  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  std::string service_name_;
  const rclcpp::QoS qos_;
  const ServiceRole role_;
  std::mutex publishers_mtx_;
  std::unordered_map<std::string, typename ServiceResponsePublisher::SharedPtr> publishers_;
#if AGNOCAST_HAS_SERVICE_INTROSPECTION
  // Declared before subscriber_ so that it outlives the callback that uses it.
  std::shared_ptr<ServiceEventPublisher> event_publisher_;
#endif
  typename ServiceRequestSubscriber::SharedPtr subscriber_;

  rclcpp::Logger get_logger() const
  {
    return std::visit([](auto * n) { return n->get_logger(); }, node_);
  }

  typename ServiceResponsePublisher::SharedPtr get_or_create_publisher_for(
    const std::string & response_topic_name)
  {
    typename ServiceResponsePublisher::SharedPtr pub;
    {
      std::lock_guard<std::mutex> lock(publishers_mtx_);
      auto it = publishers_.find(response_topic_name);
      if (it == publishers_.end()) {
        std::visit(
          [this, &pub, &response_topic_name](auto * node) {
            agnocast::PublisherOptions pub_options;
            pub = std::make_shared<ServiceResponsePublisher>(
              node, response_topic_name, qos_, pub_options, to_publisher_role(role_));
            publishers_[response_topic_name] = pub;
          },
          node_);
      } else {
        pub = it->second;
      }
    }
    return pub;
  }

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
  void publish_request_received_event(const ipc_shared_ptr<RequestT> & request)
  {
    const auto * payload = static_cast<const typename ServiceT::Request *>(request.get());

    event_publisher_->publish_service_event_message(
      service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED, payload, request->RequestMeta::seqno,
      request->RequestMeta::client_gid);
  }

  void publish_response_sent_event(
    const ipc_shared_ptr<RequestT> & request,
    const std::optional<typename ServiceT::Response> & response)
  {
    event_publisher_->publish_service_event_message(
      service_msgs::msg::ServiceEventInfo::RESPONSE_SENT, response ? &*response : nullptr,
      request->RequestMeta::seqno, request->RequestMeta::client_gid);
  }

  // Must be called before publish(). Only CONTENTS puts the payload in the event, so the other
  // states pay nothing; raising to CONTENTS in between costs that one event its payload.
  std::optional<typename ServiceT::Response> copy_response_if_contents(
    const ipc_shared_ptr<ResponseT> & response)
  {
    if (event_publisher_->introspection_state() != RCL_SERVICE_INTROSPECTION_CONTENTS) {
      return std::nullopt;
    }
    return static_cast<const typename ServiceT::Response &>(*response);
  }
#endif

  template <typename Func>
  auto wrap_basic_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<RequestT> && request) {
#if AGNOCAST_HAS_SERVICE_INTROSPECTION
      publish_request_received_event(request);
#endif

      // The name comes from the request, so a bad one is the caller's fault.
      typename ServiceResponsePublisher::SharedPtr publisher;
      try {
        publisher = this->get_or_create_publisher_for(request->RequestMeta::response_topic_name);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(), "Dropping a request for service %s: response topic name %s: %s",
          service_name_.c_str(), request->RequestMeta::response_topic_name.c_str(), e.what());
        return;
      }

      // Allocate the response and set its metadata.
      ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
      response->ResponseMeta::seqno = request->RequestMeta::seqno;

      // Invoke the user callback.
      ipc_shared_ptr<typename ServiceT::Request> request_double = request;
      ipc_shared_ptr<typename ServiceT::Response> response_double(response);
      callback(std::move(request_double), std::move(response_double));

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
      const auto sent_response = copy_response_if_contents(response);
#endif

      publisher->publish(std::move(response));

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
      publish_response_sent_event(request, sent_response);
#endif

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
#if AGNOCAST_HAS_SERVICE_INTROSPECTION
      publish_request_received_event(request);
#endif

      callback(this->shared_from_this(), std::move(request));
    };
  }

  template <typename Func, typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_name, Func && callback,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    static_assert(
      is_basic_cb<Func>::value || is_deferred_cb<Func>::value,
      "Callback must be callable with one of the following argument pairs:\n"
      "1. basic: (ipc_shared_ptr<ServiceT::Request>, ipc_shared_ptr<ServiceT::Response>)\n"
      "2. deferred: (std::shared_ptr<Service>, ipc_shared_ptr<ServiceT::Request>)\n"
      "ipc_shared_ptr arguments can be received by const&, &&, or by value");

    service_name_ = node->get_node_services_interface()->resolve_service_name(service_name);

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
    // Must precede the subscription: its callback is runnable as soon as it is registered.
    event_publisher_ = std::make_shared<ServiceEventPublisher>(
      node_, service_name_, rosidl_generator_traits::name<ServiceT>());
#endif

    SubscriptionOptions options{group};
    std::string topic_name = create_service_request_topic_name(service_name_);
    const SubscriptionRole subscriber_role = to_subscription_role(role_);
    if constexpr (is_basic_cb<Func>::value) {
      subscriber_ = std::make_shared<ServiceRequestSubscriber>(
        node, topic_name, qos_,
        wrap_basic_service_callback_for_subscriber(std::forward<Func>(callback)), options,
        subscriber_role);
    } else if constexpr (is_deferred_cb<Func>::value) {
      subscriber_ = std::make_shared<ServiceRequestSubscriber>(
        node, topic_name, qos_,
        wrap_deferred_service_callback_for_subscriber(std::forward<Func>(callback)), options,
        subscriber_role);
    }

    if (role_ == ServiceRole::Default) {
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
  using SharedPtr = std::shared_ptr<BasicService<ServiceT>>;

  template <typename Func>
  BasicService(
    rclcpp::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group,
    ServiceRole role = ServiceRole::Default)
  : node_(node), qos_(rclcpp::QoS(qos).durability_volatile()), role_(role)
  {
    constructor_impl(node, service_name, std::forward<Func>(callback), group);
  }

  template <typename Func>
  BasicService(
    agnocast::Node * node, const std::string & service_name, Func && callback,
    const rclcpp::QoS & qos, rclcpp::CallbackGroup::SharedPtr group,
    ServiceRole role = ServiceRole::Default)
  : node_(node), qos_(rclcpp::QoS(qos).durability_volatile()), role_(role)
  {
    constructor_impl(node, service_name, std::forward<Func>(callback), group);
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
    auto publisher =
      get_or_create_publisher_for(internal_request->RequestMeta::response_topic_name);

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
    const auto sent_response = copy_response_if_contents(internal_response);
#endif

    publisher->publish(std::move(internal_response));

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
    publish_response_sent_event(internal_request, sent_response);
#endif
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
    auto publisher =
      get_or_create_publisher_for(internal_request->RequestMeta::response_topic_name);
    ipc_shared_ptr<ResponseT> response = publisher->borrow_loaned_message();
    response->ResponseMeta::seqno = internal_request->RequestMeta::seqno;
    return ipc_shared_ptr<typename ServiceT::Response>(std::move(response));
  }

#if AGNOCAST_HAS_SERVICE_INTROSPECTION
  /**
   * @brief Configure service introspection.
   * @param clock The clock to use to generate introspection timestamps.
   * @param qos_service_event_pub The QoS settings to use when creating the introspection publisher.
   * @param introspection_state The state to set introspection to.
   * @throws std::invalid_argument if @p clock is null, including when disabling, as in rcl, or if
   * @p qos_service_event_pub cannot be used by Agnocast. The QoS is only checked when a publisher
   * is about to be created, so disabling never rejects it.
   * @throws std::runtime_error if the typesupport libraries for the event message cannot be
   * loaded. Only the first transition out of OFF loads them.
   */
  AGNOCAST_PUBLIC
  void configure_introspection(
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::QoS & qos_service_event_pub,
    rcl_service_introspection_state_t introspection_state)
  {
    event_publisher_->configure(clock, qos_service_event_pub, introspection_state);
  }
#endif

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
class GenericService : public std::enable_shared_from_this<GenericService>
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

  const std::variant<rclcpp::Node *, agnocast::Node *> node_;
  std::string service_name_;
  const rclcpp::QoS qos_;
  const ServiceRole role_;
  std::mutex publishers_mtx_;
  std::unordered_map<std::string, typename TypeErasedPublisher::SharedPtr> publishers_;
  typename Subscription<void>::SharedPtr subscriber_;

  ServiceTsBundle service_ts_bundle_;

  // Defined in the .cpp: agnocast::Node is only forward-declared here.
  rclcpp::Logger get_logger() const;

  typename TypeErasedPublisher::SharedPtr get_or_create_publisher_for(
    const std::string & response_topic_name);

  template <typename Func>
  auto wrap_basic_service_callback_for_subscriber(Func && callback)
  {
    return [this, callback = std::forward<Func>(callback)](ipc_shared_ptr<void> && request) {
      auto req_wrapper =
        GenericRequestWrapper(service_ts_bundle_.request_members, std::move(request));
      // The name comes from the request, so a bad one is the caller's fault.
      typename TypeErasedPublisher::SharedPtr publisher;
      try {
        publisher = this->get_or_create_publisher_for(req_wrapper.response_topic_name());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(), "Dropping a request for service %s: response topic name %s: %s",
          service_name_.c_str(), req_wrapper.response_topic_name().c_str(), e.what());
        return;
      }

      auto res_wrapper = GenericResponseWrapper::allocate(
        service_ts_bundle_.response_members,
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
          GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
        });
        throw;
      }

      publisher->publish(std::move(response), [this](void * p) {
        GenericResponseWrapper::free(p, this->service_ts_bundle_.response_members);
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

  template <typename Func, typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::CallbackGroup::SharedPtr & group)
  {
    static_assert(
      is_basic_cb<Func>::value || is_deferred_cb<Func>::value,
      "Callback must be callable with one of the following argument pairs:\n"
      "1. basic: (ipc_shared_ptr<void>, ipc_shared_ptr<void>)\n"
      "2. deferred: (std::shared_ptr<GenericService>, ipc_shared_ptr<void>)\n"
      "ipc_shared_ptr arguments can be received by const&, &&, or by value");

    service_ts_bundle_ = load_service_typesupport(service_type);

    service_name_ = node->get_node_services_interface()->resolve_service_name(service_name);

    SubscriptionOptions sub_options{group};
    std::string req_topic_name = create_service_request_topic_name(service_name_);
    const SubscriptionRole subscriber_role = to_subscription_role(role_);
    if constexpr (is_basic_cb<Func>::value) {
      subscriber_ = std::make_shared<Subscription<void>>(
        node, req_topic_name, "", qos_,
        wrap_basic_service_callback_for_subscriber(std::forward<Func>(callback)), sub_options,
        subscriber_role);
    } else if constexpr (is_deferred_cb<Func>::value) {
      subscriber_ = std::make_shared<Subscription<void>>(
        node, req_topic_name, "", qos_,
        wrap_deferred_service_callback_for_subscriber(std::forward<Func>(callback)), sub_options,
        subscriber_role);
    }

    if (role_ == ServiceRole::Default) {
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
  : node_(node), qos_(rclcpp::QoS(qos).durability_volatile()), role_(role)
  {
    constructor_impl(node, service_name, service_type, std::forward<Func>(callback), group);
  }

  template <typename Func>
  GenericService(
    agnocast::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group,
    ServiceRole role = ServiceRole::Default)
  : node_(node), qos_(rclcpp::QoS(qos).durability_volatile()), role_(role)
  {
    constructor_impl(node, service_name, service_type, std::forward<Func>(callback), group);
  }

  void send_response(ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response);

  void cancel_response(ipc_shared_ptr<void> && request, ipc_shared_ptr<void> && response);

  ipc_shared_ptr<void> borrow_loaned_response(const ipc_shared_ptr<void> & request);

  const char * get_service_name() const { return service_name_.c_str(); }
};

/**
 * @brief The user-facing Agnocast service server.
 * Alias for `BasicService<ServiceT>`. Use this type (not BasicService directly) when declaring
 * service server variables.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
AGNOCAST_PUBLIC
template <typename ServiceT>
using Service = BasicService<ServiceT>;

}  // namespace agnocast
