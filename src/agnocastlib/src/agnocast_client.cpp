#include "agnocast/agnocast_client.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <array>
#include <chrono>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace agnocast
{

uint32_t get_agnocast_sub_count(const std::string & topic_name)
{
  auto topic_info_buffer = std::make_unique<std::array<topic_info_ret, MAX_TOPIC_INFO_RET_NUM>>();

  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {topic_name.c_str(), topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(topic_info_buffer->data());
  topic_info_args.topic_info_ret_buffer_size = MAX_TOPIC_INFO_RET_NUM;
  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return topic_info_args.ret_topic_info_ret_num;
}

bool service_is_ready_core(const std::string & service_name)
{
  uint32_t sub_count = get_agnocast_sub_count(create_service_request_topic_name(service_name));

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

void GenericClient::load_typesupport_impl(const std::string & service_type)
{
  static const std::string ts_introspection_identifier = "rosidl_typesupport_introspection_cpp";
  const std::string request_type = service_type + "_Request";
  const std::string response_type = service_type + "_Response";

  ts_lib_introspection_ =
    rclcpp::get_typesupport_library(service_type, ts_introspection_identifier);

#if RCLCPP_VERSION_MAJOR >= 28
  const rosidl_message_type_support_t * request_ts = rclcpp::get_message_typesupport_handle(
    request_type, ts_introspection_identifier, *ts_lib_introspection_);
  const rosidl_message_type_support_t * response_ts = rclcpp::get_message_typesupport_handle(
    response_type, ts_introspection_identifier, *ts_lib_introspection_);
#else
  const rosidl_message_type_support_t * request_ts = rclcpp::get_typesupport_handle(
    request_type, ts_introspection_identifier, *ts_lib_introspection_);
  const rosidl_message_type_support_t * response_ts = rclcpp::get_typesupport_handle(
    response_type, ts_introspection_identifier, *ts_lib_introspection_);
#endif

  request_members_ =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(request_ts->data);
  response_members_ =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(response_ts->data);
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
    request_members_, [this](size_t size) { return publisher_->borrow_loaned_message(size); });

  generic_request_wrapper.node_name() = node_name_;
  generic_request_wrapper.seqno() = next_sequence_number_.fetch_add(1);

  return std::move(generic_request_wrapper).take_request();
}

GenericClient::SharedFutureAndRequestId GenericClient::async_send_request(
  ipc_shared_ptr<void> && request, std::function<void(SharedFuture)> callback)
{
  SharedFuture shared_future;
  auto generic_request_wrapper = GenericRequestWrapper(request_members_, std::move(request));
  int64_t seqno = generic_request_wrapper.seqno();

  {
    std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
    auto it = seqno2_response_call_info_.try_emplace(seqno, std::move(callback)).first;
    shared_future = it->second.shared_future.value();
  }

  publisher_->publish(std::move(generic_request_wrapper).take_request(), [this](void * p) {
    GenericRequestWrapper::free(p, this->request_members_);
  });
  return {std::move(shared_future), seqno};
}

GenericClient::FutureAndRequestId GenericClient::async_send_request(ipc_shared_ptr<void> && request)
{
  Future future;
  auto generic_request_wrapper = GenericRequestWrapper(request_members_, std::move(request));
  int64_t seqno = generic_request_wrapper.seqno();

  {
    std::lock_guard<std::mutex> lock(seqno2_response_call_info_mtx_);
    auto it = seqno2_response_call_info_.try_emplace(seqno).first;
    future = it->second.promise.get_future();
  }

  publisher_->publish(std::move(generic_request_wrapper).take_request(), [this](void * p) {
    GenericRequestWrapper::free(p, this->request_members_);
  });
  return {std::move(future), seqno};
}

void GenericClient::cancel_request(ipc_shared_ptr<void> && request)
{
  publisher_->cancel_message(std::move(request), [this](void * p) {
    GenericRequestWrapper::free(p, this->request_members_);
  });
}

}  // namespace agnocast
