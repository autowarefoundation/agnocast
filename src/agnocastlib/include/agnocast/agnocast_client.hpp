#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
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

/**
 * @brief Service client for zero-copy Agnocast service communication.
 * @tparam ServiceT The ROS service type (e.g., std_srvs::srv::SetBool).
 */
AGNOCAST_PUBLIC
template <typename ServiceT>
class Client
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

  struct ResponseCallInfo
  {
    std::promise<ipc_shared_ptr<typename ServiceT::Response>> promise;
    std::optional<SharedFuture> shared_future;
    std::optional<std::function<void(SharedFuture)>> callback;

    ResponseCallInfo() = default;

    explicit ResponseCallInfo(std::function<void(SharedFuture)> && cb) : callback(std::move(cb))
    {
      shared_future = promise.get_future().share();
    }
  };

  using ServiceRequestPublisher = Publisher<RequestT>;
  using ServiceResponseSubscriber = Subscription<ResponseT>;

  std::atomic<int64_t> next_sequence_number_;
  std::mutex seqno2_response_call_info_mtx_;
  std::unordered_map<int64_t, ResponseCallInfo> seqno2_response_call_info_;
  std::string node_name_;
  std::string service_name_;
  std::function<bool()> check_context_ok_;
  typename ServiceRequestPublisher::SharedPtr publisher_;
  typename ServiceResponseSubscriber::SharedPtr subscriber_;

  template <typename NodeT>
  void constructor_impl(
    NodeT * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group)
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

    // TransientLocal durability is not allowed for services.
    const rclcpp::QoS qos = rclcpp::QoS(qos_arg).durability_volatile();

    agnocast::PublisherOptions pub_options;
    publisher_ = std::make_shared<ServiceRequestPublisher>(
      node, create_service_request_topic_name(service_name_), qos, pub_options,
      PublisherRole::AgnocastOnly);

    auto subscriber_callback = [this, node](ipc_shared_ptr<ResponseT> && response) {
      std::unique_lock<std::mutex> lock(seqno2_response_call_info_mtx_);
      /* --- critical section begin --- */
      // Get the corresponding ResponseCallInfo and remove it from the map
      auto it = seqno2_response_call_info_.find(response->_sequence_number);
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
    subscriber_ = std::make_shared<ServiceResponseSubscriber>(
      node, topic_name, qos, std::move(subscriber_callback), options,
      SubscriptionRole::AgnocastOnly);
  }

public:
  Client(
    rclcpp::Node * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    constructor_impl(node, service_name, qos_arg, group);
  }

  Client(
    agnocast::Node * node, const std::string & service_name, const rclcpp::QoS & qos_arg,
    rclcpp::CallbackGroup::SharedPtr group)
  {
    constructor_impl(node, service_name, qos_arg, group);
  }

  /** @brief Allocate a request message in shared memory.
   *  @return Owned pointer to the request message in shared memory. */
  AGNOCAST_PUBLIC
  ipc_shared_ptr<typename ServiceT::Request> borrow_loaned_request()
  {
    auto request = publisher_->borrow_loaned_message();
    request->_node_name = node_name_;
    request->_sequence_number = next_sequence_number_.fetch_add(1);
    return ipc_shared_ptr<typename ServiceT::Request>(std::move(request));
  }

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
    int64_t seqno = internal_request->_sequence_number;

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
    int64_t seqno = internal_request->_sequence_number;

    {
      std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
      auto it = seqno2_response_call_info_.try_emplace(seqno).first;
      future = it->second.promise.get_future();
    }

    publisher_->publish(std::move(internal_request));
    return FutureAndRequestId(std::move(future), seqno);
  }
};

}  // namespace agnocast
