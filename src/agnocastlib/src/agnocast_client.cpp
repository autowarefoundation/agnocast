#include "agnocast/agnocast_client.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <chrono>
#include <cstring>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace agnocast
{

rclcpp::Logger ClientBase::get_logger() const
{
  return std::visit([](auto * n) { return n->get_logger(); }, node_);
}

uint32_t get_reachable_agnocast_sub_count(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return args.ret_same_process_subscriber_num + args.ret_other_process_subscriber_num +
         args.ret_other_domain_subscriber_num;
}

bool service_is_ready_core(const std::string & service_name)
{
  const uint32_t sub_count =
    get_reachable_agnocast_sub_count(create_service_request_topic_name(service_name));

  if (sub_count == 0) {
    return false;
  }

  if (sub_count > 1) {
    RCLCPP_WARN(
      logger, "Multiple services with the same name found (name=%s).", service_name.c_str());
  }
  return true;
}

bool wait_for_service_nanoseconds(
  const std::function<bool()> & check_context_ok, const std::string & service_name,
  nanoseconds timeout)
{
  auto start = steady_clock::now();
  if (service_is_ready_core(service_name)) {
    return true;
  }
  if (timeout == nanoseconds(0)) {
    // non-blocking, return immediately
    return false;
  }
  // If timeout is negative, wait indefinitely.
  nanoseconds time_to_wait =
    timeout > nanoseconds(0) ? timeout - (steady_clock::now() - start) : nanoseconds::max();
  do {
    if (!check_context_ok()) {
      return false;
    }
    nanoseconds interval = std::min(time_to_wait, duration_cast<nanoseconds>(100ms));
    std::this_thread::sleep_for(interval);
    if (service_is_ready_core(service_name)) {
      return true;
    }
    if (timeout > nanoseconds(0)) {
      time_to_wait = timeout - (steady_clock::now() - start);
    }
    // If timeout is negative, time_to_wait will never reach zero.
  } while (time_to_wait > nanoseconds(0));
  return false;
}

GenericClient::GenericClient(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group, ClientRole role)
{
  constructor_impl(node, service_name, service_type, qos, group, role);
}

GenericClient::GenericClient(
  agnocast::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group, ClientRole role)
{
  constructor_impl(node, service_name, service_type, qos, group, role);
}

ipc_shared_ptr<void> GenericClient::borrow_loaned_request()
{
  auto generic_request_wrapper = GenericRequestWrapper::allocate(
    service_ts_bundle_.request_members,
    [this](size_t size) { return publisher_->borrow_loaned_message(size); });

  generic_request_wrapper.seqno() = next_sequence_number_.fetch_add(1);
  std::memcpy(
    static_cast<void *>(generic_request_wrapper.client_gid()),
    static_cast<const void *>(get_gid().data), RMW_GID_STORAGE_SIZE);
  generic_request_wrapper.node_name() = node_name_;

  return std::move(generic_request_wrapper).take_request();
}

GenericClient::SharedFutureAndRequestId GenericClient::async_send_request(
  ipc_shared_ptr<void> && request, std::function<void(SharedFuture)> && callback)
{
  SharedFuture shared_future;
  auto generic_request_wrapper =
    GenericRequestWrapper(service_ts_bundle_.request_members, std::move(request));
  int64_t seqno = generic_request_wrapper.seqno();

  {
    std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
    auto it = seqno2_response_call_info_.try_emplace(seqno, std::move(callback)).first;
    shared_future = it->second.shared_future.value();
  }

  publisher_->publish(std::move(generic_request_wrapper).take_request(), [this](void * p) {
    GenericRequestWrapper::free(p, this->service_ts_bundle_.request_members);
  });
  return {std::move(shared_future), seqno};
}

GenericClient::FutureAndRequestId GenericClient::async_send_request(ipc_shared_ptr<void> && request)
{
  Future future;
  auto generic_request_wrapper =
    GenericRequestWrapper(service_ts_bundle_.request_members, std::move(request));
  int64_t seqno = generic_request_wrapper.seqno();

  {
    std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
    auto it = seqno2_response_call_info_.try_emplace(seqno).first;
    future = it->second.promise.get_future();
  }

  publisher_->publish(std::move(generic_request_wrapper).take_request(), [this](void * p) {
    GenericRequestWrapper::free(p, this->service_ts_bundle_.request_members);
  });
  return {std::move(future), seqno};
}

void GenericClient::cancel_request(ipc_shared_ptr<void> && request)
{
  publisher_->cancel_message(std::move(request), [this](void * p) {
    GenericRequestWrapper::free(p, this->service_ts_bundle_.request_members);
  });
}

}  // namespace agnocast
