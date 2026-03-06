#include "agnocast/node/agnocast_rosout.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <rcl_interfaces/msg/log.hpp>
#include <rcutils/logging.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace agnocast
{

static Publisher<rcl_interfaces::msg::Log>::SharedPtr g_rosout_pub;
static std::mutex g_rosout_mtx;
static std::atomic<bool> g_in_handler{false};
static std::atomic<bool> g_rosout_initialized{false};
static rcutils_logging_output_handler_t g_original_handler = nullptr;

static uint8_t severity_to_log_level(int severity)
{
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return rcl_interfaces::msg::Log::DEBUG;
    case RCUTILS_LOG_SEVERITY_INFO:
      return rcl_interfaces::msg::Log::INFO;
    case RCUTILS_LOG_SEVERITY_WARN:
      return rcl_interfaces::msg::Log::WARN;
    case RCUTILS_LOG_SEVERITY_ERROR:
      return rcl_interfaces::msg::Log::ERROR;
    case RCUTILS_LOG_SEVERITY_FATAL:
      return rcl_interfaces::msg::Log::FATAL;
    default:
      return rcl_interfaces::msg::Log::INFO;
  }
}

static void rosout_output_handler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  // Chain the original handler (console or noop, depending on --disable-stdout-logs).
  // Pass a copy because the handler may consume the va_list via vsnprintf.
  if (g_original_handler) {
    va_list args_for_original;
    va_copy(args_for_original, *args);
    g_original_handler(location, severity, name, timestamp, format, &args_for_original);
    va_end(args_for_original);
  }

  // Recursion guard: publishing may trigger log calls internally
  bool expected = false;
  if (!g_in_handler.compare_exchange_strong(expected, true)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(g_rosout_mtx);
    if (g_rosout_pub) {
      auto loaned_msg = g_rosout_pub->borrow_loaned_message();

      loaned_msg->stamp.sec = static_cast<int32_t>(timestamp / 1000000000LL);
      loaned_msg->stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000LL);
      loaned_msg->level = severity_to_log_level(severity);
      loaned_msg->name = (name != nullptr) ? name : "";

      // Format the message string from va_list
      va_list args_copy;
      va_copy(args_copy, *args);
      int len = vsnprintf(nullptr, 0, format, args_copy);
      va_end(args_copy);
      if (len >= 0) {
        std::string msg_str(static_cast<size_t>(len), '\0');
        va_copy(args_copy, *args);
        vsnprintf(&msg_str[0], static_cast<size_t>(len) + 1, format, args_copy);
        va_end(args_copy);
        loaned_msg->msg = std::move(msg_str);
      }

      if (location != nullptr) {
        loaned_msg->file = (location->file_name != nullptr) ? location->file_name : "";
        loaned_msg->function = (location->function_name != nullptr) ? location->function_name : "";
        loaned_msg->line = location->line_number;
      }

      g_rosout_pub->publish(std::move(loaned_msg));
    }
  }

  g_in_handler.store(false);
}

void setup_rosout_handler(Node * node)
{
  // Only set up once per process, even with multiple nodes
  bool expected = false;
  if (!g_rosout_initialized.compare_exchange_strong(expected, true)) {
    return;
  }

  auto pub = node->create_publisher<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS(100).reliable().transient_local());

  {
    std::lock_guard<std::mutex> lock(g_rosout_mtx);
    g_rosout_pub = pub;
  }

  // Capture the current handler (console or noop) before replacing
  g_original_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(rosout_output_handler);
}

}  // namespace agnocast
