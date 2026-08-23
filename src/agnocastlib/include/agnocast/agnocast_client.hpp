#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "agnocast/internal/service_typesupport.hpp"
#include "agnocast/internal/service_wire_type.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

namespace agnocast
{

bool service_is_ready_core(const std::string & service_name);
bool wait_for_service_nanoseconds(
  const std::function<bool()> & check_context_ok, const std::string & service_name,
  std::chrono::nanoseconds timeout);

extern int agnocast_fd;

enum class ClientRole : uint8_t {
  /// User-created client; issues an A2R bridge request.
  Default,
  /// Used by the bridge implementation itself; marks the endpoints it creates as bridges in kmod
  /// and issues no bridge request.
  /// Not intended for direct use by application code.
  BridgeInternal,
};

constexpr SubscriptionRole to_subscription_role(const ClientRole role)
{
  return role == ClientRole::BridgeInternal ? SubscriptionRole::BridgeInternal
                                            : SubscriptionRole::AgnocastOnly;
}

constexpr PublisherRole to_publisher_role(const ClientRole role)
{
  return role == ClientRole::BridgeInternal ? PublisherRole::BridgeInternal
                                            : PublisherRole::AgnocastOnly;
}

namespace detail
{

/// ResponseCallInfo template - parameterized on the response type.
template <typename Response>
struct ResponseCallInfo
{
  using SharedFuture = std::shared_future<ipc_shared_ptr<Response>>;

  std::promise<ipc_shared_ptr<Response>> promise;
  std::optional<SharedFuture> shared_future;
  std::optional<std::function<void(SharedFuture)>> callback;

  ResponseCallInfo() = default;

  explicit ResponseCallInfo(std::function<void(SharedFuture)> && cb) : callback(std::move(cb))
  {
    shared_future = promise.get_future().share();
  }
};

}  // namespace detail

class ClientBase
{
protected:
  std::atomic<int64_t> next_sequence_number_{0};
  std::string node_name_;
  std::string service_name_;
  std::function<bool()> check_context_ok_;

  template <typename NodeT>
  void init_base(NodeT * node, const std::string & service_name)
  {
    node_name_ = node->get_fully_qualified_name();
    service_name_ = node->get_node_services_interface()->resolve_service_name(service_name);

    check_context_ok_ = [context = node->get_node_base_interface()->get_context()]() {
      if constexpr (std::is_same_v<agnocast::Node, NodeT>) {
        return agnocast::ok();
      } else {
        return rclcpp::ok(context);
      }
    };
  }

public:
  /** @brief Return the resolved service name.
   *  @return Null-terminated service name string. */
  AGNOCAST_PUBLIC
  const char * get_service_name() const { return service_name_.c_str(); }

  /** @brief Check if the service server is available.
   *  @return True if the service server is available. */
  AGNOCAST_PUBLIC
  bool service_is_ready() const { return service_is_ready_core(service_name_); }

  /** @brief Block until the service is available or the timeout expires.
   *  @param timeout Maximum duration to wait (-1 = wait forever).
   *  @return True if service became available, false on timeout. */
  AGNOCAST_PUBLIC
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) const
  {
    return wait_for_service_nanoseconds(
      check_context_ok_, service_name_,
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual ~ClientBase() = default;
};

/**
 * @brief Service client for zero-copy Agnocast service communication.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
AGNOCAST_PUBLIC
template <typename ServiceT>
class Client : public ClientBase
{
public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;

  /// Future that resolves to the service response. Returned by async_send_request() (no-callback
  /// overload).
  AGNOCAST_PUBLIC
  using Future = std::future<ipc_shared_ptr<typename ServiceT::Response>>;
  /// Shared future that resolves to the service response. Passed to the callback in
  /// async_send_request().
  AGNOCAST_PUBLIC
  using SharedFuture = std::shared_future<ipc_shared_ptr<typename ServiceT::Response>>;

  /// Return type of async_send_request() (no-callback overload). Contains a Future and the request
  /// ID. Access the future via the `future` member and the request ID via `request_id`.
  AGNOCAST_PUBLIC
  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    /// Convert to a SharedFutureAndRequestId by sharing the underlying future.
    AGNOCAST_PUBLIC
    SharedFuture share() noexcept { return this->future.share(); }
  };
  /// Return type of async_send_request() (callback overload). Contains a SharedFuture and the
  /// request ID. Access the shared future via the `future` member and the request ID via
  /// `request_id`.
  AGNOCAST_PUBLIC
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

private:
  using RequestT = ServiceRequestWrapper<ServiceT>;
  using ResponseT = ServiceResponseWrapper<ServiceT>;
  using ResponseCallInfo = detail::ResponseCallInfo<typename ServiceT::Response>;

  using ServiceRequestPublisher = Publisher<RequestT>;
  using ServiceResponseSubscriber = Subscription<ResponseT>;

  std::mutex seqno2_response_call_info_mtx_;
  std::unordered_map<int64_t, ResponseCallInfo> seqno2_response_call_info_;
  typename ServiceRequestPublisher::SharedPtr publisher_;
  typename ServiceResponseSubscriber::SharedPtr subscriber_;

  template <typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group, ClientRole role)
  {
    init_base(node, service_name);

    // TransientLocal durability is not allowed for services.
    const rclcpp::QoS qos = rclcpp::QoS(qos_arg).durability_volatile();

    agnocast::PublisherOptions pub_options;
    publisher_ = std::make_shared<ServiceRequestPublisher>(
      node, create_service_request_topic_name(service_name_), qos, pub_options,
      to_publisher_role(role));

    auto subscriber_callback = [this, node](ipc_shared_ptr<ResponseT> && response) {
      std::unique_lock<std::mutex> lock(seqno2_response_call_info_mtx_);
      /* --- critical section begin --- */
      // Get the corresponding ResponseCallInfo and remove it from the map
      auto it = seqno2_response_call_info_.find(response->ResponseMeta::seqno);
      if (it == seqno2_response_call_info_.end()) {
        lock.unlock();
        RCLCPP_ERROR(node->get_logger(), "Agnocast internal implementation error: bad entry id");
        return;
      }
      ResponseCallInfo info = std::move(it->second);
      seqno2_response_call_info_.erase(it);
      /* --- critical section end --- */
      lock.unlock();

      info.promise.set_value(ipc_shared_ptr<typename ServiceT::Response>(std::move(response)));
      if (info.callback.has_value()) {
        (info.callback.value())(info.shared_future.value());
      }
    };

    SubscriptionOptions options{group};
    std::string topic_name = create_service_response_topic_name(service_name_, node_name_);
    const SubscriptionRole subscriber_role = to_subscription_role(role);
    subscriber_ = std::make_shared<ServiceResponseSubscriber>(
      node, topic_name, qos, std::move(subscriber_callback), options, subscriber_role);

    if (role == ClientRole::Default) {
      register_service_bridge(
        rosidl_generator_traits::name<ServiceT>(), service_name_, BridgeDirection::AGNOCAST_TO_ROS2,
        std::nullopt);
    }
  }

  // Return the GID of this client.
  //
  // It is the same as the GID of the underlying publisher. Because every client has a distinct
  // publisher that is not observable to users, we can safely reuse the GID for the client.
  const rmw_gid_t & get_gid() const { return publisher_->get_gid(); }

public:
  Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group, ClientRole role = ClientRole::Default)
  {
    constructor_impl(node, service_name, qos_arg, group, role);
  }

  Client(
    agnocast::Node * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group, ClientRole role = ClientRole::Default)
  {
    constructor_impl(node, service_name, qos_arg, group, role);
  }

  /** @brief Allocate a request message in shared memory.
   *  @return Owned pointer to the request message in shared memory. */
  AGNOCAST_PUBLIC
  ipc_shared_ptr<typename ServiceT::Request> borrow_loaned_request()
  {
    auto request = publisher_->borrow_loaned_message();

    request->RequestMeta::seqno = next_sequence_number_.fetch_add(1);
    std::memcpy(
      static_cast<void *>(request->RequestMeta::client_gid),
      static_cast<const void *>(get_gid().data), RMW_GID_STORAGE_SIZE);
    request->RequestMeta::node_name = node_name_;

    return ipc_shared_ptr<typename ServiceT::Request>(std::move(request));
  }

  /** @brief Send a request asynchronously and invoke a callback when the response arrives.
   *  @param request Request from borrow_loaned_request(). Must be moved in.
   *  @param callback Invoked with a SharedFuture when the response arrives. Call future.get() to
   * obtain the response.
   *  @return A SharedFutureAndRequestId containing the shared future (`.future`) and a sequence
   * number (`.request_id`). */
  AGNOCAST_PUBLIC
  SharedFutureAndRequestId async_send_request(
    ipc_shared_ptr<typename ServiceT::Request> && request,
    std::function<void(SharedFuture)> callback)
  {
    SharedFuture shared_future;
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    int64_t seqno = internal_request->RequestMeta::seqno;

    {
      std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
      auto it = seqno2_response_call_info_.try_emplace(seqno, std::move(callback)).first;
      shared_future = it->second.shared_future.value();
    }

    publisher_->publish(std::move(internal_request));
    return SharedFutureAndRequestId(std::move(shared_future), seqno);
  }

  /** @brief Send a request asynchronously and return a future for the response.
   *  @param request Request from borrow_loaned_request(). Must be moved in.
   *  @return A FutureAndRequestId containing the future (`.future`) and a sequence number
   * (`.request_id`). Call `.future.get()` to block until the response arrives. */
  AGNOCAST_PUBLIC
  FutureAndRequestId async_send_request(ipc_shared_ptr<typename ServiceT::Request> && request)
  {
    Future future;
    auto internal_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    int64_t seqno = internal_request->RequestMeta::seqno;

    {
      std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
      auto it = seqno2_response_call_info_.try_emplace(seqno).first;
      future = it->second.promise.get_future();
    }

    publisher_->publish(std::move(internal_request));
    return FutureAndRequestId(std::move(future), seqno);
  }
};

/**
 * @brief Generic service client for zero-copy Agnocast service communication.
 *
 * The service type is supplied as a runtime string rather than a compile-time template argument.
 * If the given service type is invalid, the constructor will throw an exception.
 *
 * The basic usage is the same as agnocast::Client, except that if you decide not to send a loaned
 * request, you must call cancel_request() to free the loaned request memory. Otherwise, the process
 * will terminate.
 */
class GenericClient : public ClientBase
{
public:
  using SharedPtr = std::shared_ptr<GenericClient>;

  using Future = std::future<ipc_shared_ptr<void>>;
  using SharedFuture = std::shared_future<ipc_shared_ptr<void>>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

private:
  using ResponseCallInfo = detail::ResponseCallInfo<void>;

  std::mutex seqno2_response_call_info_mtx_;
  std::unordered_map<int64_t, ResponseCallInfo> seqno2_response_call_info_;
  typename TypeErasedPublisher::SharedPtr publisher_;
  typename Subscription<void>::SharedPtr subscriber_;

  ServiceTsBundle service_ts_bundle_;

  template <typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos_arg, const rclcpp::CallbackGroup::SharedPtr & group, ClientRole role)
  {
    init_base(node, service_name);

    service_ts_bundle_ = load_service_typesupport(service_type);

    // TransientLocal durability is not allowed for services.
    const rclcpp::QoS qos = rclcpp::QoS(qos_arg).durability_volatile();

    agnocast::PublisherOptions pub_options;
    std::string req_topic_name = create_service_request_topic_name(service_name_);
    publisher_ = std::make_shared<TypeErasedPublisher>(
      node, req_topic_name, "", qos, pub_options, to_publisher_role(role));

    auto subscriber_callback = [this, node](ipc_shared_ptr<void> && response) {
      auto generic_response_wrapper =
        GenericResponseWrapper(service_ts_bundle_.response_members, std::move(response));
      int64_t response_seqno = generic_response_wrapper.seqno();

      std::unique_lock<std::mutex> lock(seqno2_response_call_info_mtx_);
      /* --- critical section begin --- */
      // Get the corresponding ResponseCallInfo and remove it from the map
      auto it = seqno2_response_call_info_.find(response_seqno);
      if (it == seqno2_response_call_info_.end()) {
        lock.unlock();
        RCLCPP_ERROR(node->get_logger(), "Agnocast internal implementation error: bad entry id");
        return;
      }
      ResponseCallInfo info = std::move(it->second);
      seqno2_response_call_info_.erase(it);
      /* --- critical section end --- */
      lock.unlock();

      info.promise.set_value(std::move(generic_response_wrapper).take_response());
      if (info.callback.has_value()) {
        (info.callback.value())(info.shared_future.value());
      }
    };

    SubscriptionOptions sub_options{group};
    std::string res_topic_name = create_service_response_topic_name(service_name_, node_name_);
    const SubscriptionRole subscriber_role = to_subscription_role(role);
    subscriber_ = std::make_shared<Subscription<void>>(
      node, res_topic_name, "", qos, std::move(subscriber_callback), sub_options, subscriber_role);

    if (role == ClientRole::Default) {
      register_service_bridge(
        service_type, service_name_, BridgeDirection::AGNOCAST_TO_ROS2, std::nullopt);
    }
  }

  // Return the GID of this client.
  //
  // It is the same as the GID of the underlying publisher. Because every client has a distinct
  // publisher that is not observable to users, we can safely reuse the GID for the client.
  const rmw_gid_t & get_gid() const { return publisher_->get_gid(); }

public:
  GenericClient(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    const rclcpp::CallbackGroup::SharedPtr & group = nullptr,
    ClientRole role = ClientRole::Default);

  GenericClient(
    agnocast::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    const rclcpp::CallbackGroup::SharedPtr & group = nullptr,
    ClientRole role = ClientRole::Default);

  /// @brief Allocate a type-erased request message in shared memory. The message is initialized
  /// via the introspection init function with MessageInitialization::SKIP.
  /// @return Owned pointer to the request message in shared memory, which must be either sent via
  /// async_send_request() or freed via cancel_request() later.
  ipc_shared_ptr<void> borrow_loaned_request();

  /// @brief Send a request asynchronously and invoke a callback when the response arrives.
  /// @param request Request from borrow_loaned_request(). Must be moved in.
  /// @param callback Invoked with a SharedFuture when the response arrives. Call .get() to obtain
  /// the response.
  /// @return A SharedFutureAndRequestId containing the shared future and the sequence number of
  /// the request.
  SharedFutureAndRequestId async_send_request(
    ipc_shared_ptr<void> && request, std::function<void(SharedFuture)> && callback);

  /// @brief Send a request asynchronously and return a future for the response.
  /// @param request Request from borrow_loaned_request(). Must be moved in.
  /// @return A FutureAndRequestId containing the future and the sequence number of the request.
  FutureAndRequestId async_send_request(ipc_shared_ptr<void> && request);

  /// @brief Cancel a request that was not sent via async_send_request().
  /// @param request Request from borrow_loaned_request(). Must be moved in.
  void cancel_request(ipc_shared_ptr<void> && request);
};

}  // namespace agnocast
