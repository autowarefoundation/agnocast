#include "agnocast/agnocast_client.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/node/agnocast_context.hpp"

#include <array>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace agnocast
{

uint32_t get_agnocast_sub_count(const std::string & topic_name)
{
  std::array<topic_info_ret, MAX_TOPIC_INFO_RET_NUM> topic_info_buffer{};

  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {topic_name.c_str(), topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr = reinterpret_cast<uint64_t>(topic_info_buffer.data());
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
  const rclcpp::Context::SharedPtr & context, const std::string & service_name, nanoseconds timeout)
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
    // TODO(Koichi98): agnocast::ok and agnocast::shutdown are planned to be implemented.
    if (context ? !rclcpp::ok(context) : false) {
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

}  // namespace agnocast
