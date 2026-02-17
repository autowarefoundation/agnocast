#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
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
#include <unordered_map>
#include <utility>
#include <variant>

namespace agnocast
{

bool service_is_ready_core(const std::string & service_name);
bool wait_for_service_nanoseconds(
  const rclcpp::Context::SharedPtr & context, const std::string & service_name,
  std::chrono::nanoseconds timeout);

extern int agnocast_fd;

template <typename ServiceT>
class Client
{
public:
  using SharedPtr = std::shared_ptr<Client<ServiceT>>;

  using Future = std::future<ipc_shared_ptr<typename ServiceT::Response>>;
  using SharedFuture = std::shared_future<ipc_shared_ptr<typename ServiceT::Response>>;

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

  using ServiceRequestPublisher = BasicPublisher<RequestT, NoBridgeRequestPolicy>;

  std::atomic<int64_t> next_sequence_number_;
  std::mutex seqno2_response_call_info_mtx_;
  std::unordered_map<int64_t, ResponseCallInfo> seqno2_response_call_info_;
  std::variant<rclcpp::Node *, agnocast::Node *> node_;
  rclcpp::Logger logger_;
  const std::string node_name_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  const std::string service_name_;
  typename ServiceRequestPublisher::SharedPtr publisher_;
  typename BasicSubscription<ResponseT, NoBridgeRequestPolicy>::SharedPtr subscriber_;

public:
  template <typename NodeT>
  Client(
    NodeT * node, const std::string & service_name, const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group)
  : node_(node),
    logger_(node->get_logger()),
    node_name_(node->get_fully_qualified_name()),
    node_base_(node->get_node_base_interface()),
    service_name_(node->get_node_services_interface()->resolve_service_name(service_name))
  {
    static_assert(
      std::is_same_v<NodeT, rclcpp::Node> || std::is_same_v<NodeT, agnocast::Node>,
      "NodeT must be either rclcpp::Node or agnocast::Node");
    RCLCPP_WARN(
      logger_,
      "Agnocast service/client is not officially supported yet and the API may change in the "
      "future: %s",
      service_name_.c_str());
    agnocast::PublisherOptions pub_options;
    publisher_ = std::make_shared<ServiceRequestPublisher>(
      node, create_service_request_topic_name(service_name_), qos, pub_options);

    auto subscriber_callback = [this](ipc_shared_ptr<ResponseT> && response) {
      std::unique_lock<std::mutex> lock(seqno2_response_call_info_mtx_);
      /* --- critical section begin --- */
      // Get the corresponding ResponseCallInfo and remove it from the map
      auto it = seqno2_response_call_info_.find(response->_sequence_number);
      if (it == seqno2_response_call_info_.end()) {
        lock.unlock();
        RCLCPP_ERROR(logger_, "Agnocast internal implementation error: bad entry id");
        return;
      }
      ResponseCallInfo info = std::move(it->second);
      seqno2_response_call_info_.erase(it);
      /* --- critical section end --- */
      lock.unlock();

      info.promise.set_value(
        static_ipc_shared_ptr_cast<typename ServiceT::Response>(std::move(response)));
      if (info.callback.has_value()) {
        (info.callback.value())(info.shared_future.value());
      }
    };

    SubscriptionOptions options{group};
    std::string topic_name = create_service_response_topic_name(service_name_, node_name_);
    std::visit(
      [this, &topic_name, &qos, cb = std::move(subscriber_callback), &options](auto * node) {
        subscriber_ = std::make_shared<BasicSubscription<ResponseT, NoBridgeRequestPolicy>>(
          node, topic_name, qos, std::move(cb), options);
      },
      node_);
  }

  ipc_shared_ptr<typename ServiceT::Request> borrow_loaned_request()
  {
    auto request = publisher_->borrow_loaned_message();
    request->_node_name = node_name_;
    request->_sequence_number = next_sequence_number_.fetch_add(1);
    return static_ipc_shared_ptr_cast<typename ServiceT::Request>(std::move(request));
  }

  const char * get_service_name() const { return service_name_.c_str(); }

  bool service_is_ready() const { return service_is_ready_core(service_name_); }

  template <typename RepT, typename RatioT>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::nanoseconds(-1)) const
  {
    return wait_for_service_nanoseconds(
      node_base_->get_context(), service_name_,
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  SharedFutureAndRequestId async_send_request(
    ipc_shared_ptr<typename ServiceT::Request> && request,
    std::function<void(SharedFuture)> callback)
  {
    SharedFuture shared_future;
    auto derived_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    int64_t seqno = derived_request->_sequence_number;

    {
      std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
      auto it = seqno2_response_call_info_.try_emplace(seqno, std::move(callback)).first;
      shared_future = it->second.shared_future.value();
    }

    publisher_->publish(std::move(derived_request));
    return SharedFutureAndRequestId(std::move(shared_future), seqno);
  }

  FutureAndRequestId async_send_request(ipc_shared_ptr<typename ServiceT::Request> && request)
  {
    Future future;
    auto derived_request = static_ipc_shared_ptr_cast<RequestT>(std::move(request));
    int64_t seqno = derived_request->_sequence_number;

    {
      std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
      auto it = seqno2_response_call_info_.try_emplace(seqno).first;
      future = it->second.promise.get_future();
    }

    publisher_->publish(std::move(derived_request));
    return FutureAndRequestId(std::move(future), seqno);
  }
};

}  // namespace agnocast
