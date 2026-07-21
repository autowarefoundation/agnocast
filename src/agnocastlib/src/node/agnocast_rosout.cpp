#include "agnocast/node/agnocast_rosout.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <rcl_interfaces/msg/log.hpp>

#include <rcutils/logging.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agnocast
{

static std::unordered_map<std::string, Publisher<rcl_interfaces::msg::Log>::SharedPtr>
  g_rosout_pubs;
static std::mutex g_rosout_mtx;
static std::atomic<bool> g_in_handler{false};
static std::atomic<bool> g_handler_installed{false};
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
  rcutils_time_point_value_t timestamp, const char * format, va_list * args) noexcept
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
  // RAII guard: ensures g_in_handler is cleared even if an exception is thrown internally
  struct InHandlerGuard
  {
    ~InHandlerGuard() { g_in_handler.store(false); }
  } guard;

  try {
    const std::string logger_name = (name != nullptr) ? name : "";
    std::lock_guard<std::mutex> lock(g_rosout_mtx);
    auto it = g_rosout_pubs.find(logger_name);
    if (it == g_rosout_pubs.end()) {
      return;
    }
    auto & pub = it->second;

    auto loaned_msg = pub->borrow_loaned_message();

    loaned_msg->stamp.sec = static_cast<int32_t>(timestamp / 1000000000LL);
    loaned_msg->stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000LL);
    loaned_msg->level = severity_to_log_level(severity);
    loaned_msg->name = logger_name;

    // Format the message string from va_list
    va_list args_copy;
    va_copy(args_copy, *args);
    int len = vsnprintf(nullptr, 0, format, args_copy);
    va_end(args_copy);
    if (len >= 0) {
      const size_t msg_len = static_cast<size_t>(len);
      std::string msg_str;
      msg_str.resize(msg_len + 1);
      va_copy(args_copy, *args);
      vsnprintf(&msg_str[0], msg_len + 1, format, args_copy);
      va_end(args_copy);
      msg_str.resize(msg_len);
      loaned_msg->msg = std::move(msg_str);
    }

    if (location != nullptr) {
      loaned_msg->file = (location->file_name != nullptr) ? location->file_name : "";
      loaned_msg->function = (location->function_name != nullptr) ? location->function_name : "";
      loaned_msg->line = location->line_number;
    }

    pub->publish(std::move(loaned_msg));
  } catch (...) {
    // Swallow exceptions: throwing across this C callback boundary is UB
  }
}

void setup_rosout_handler(Node * node, const rclcpp::QoS & qos)
{
  auto pub = node->create_publisher<rcl_interfaces::msg::Log>("/rosout", qos);

  {
    std::lock_guard<std::mutex> lock(g_rosout_mtx);
    g_rosout_pubs[node->get_logger().get_name()] = std::move(pub);
  }

  // Install handler only once per init/shutdown cycle
  bool expected = false;
  if (g_handler_installed.compare_exchange_strong(expected, true)) {
    g_original_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(rosout_output_handler);
  }
}

void teardown_rosout_handler(Node * node)
{
  std::lock_guard<std::mutex> lock(g_rosout_mtx);
  g_rosout_pubs.erase(node->get_logger().get_name());
}

void shutdown_rosout_handler()
{
  {
    std::lock_guard<std::mutex> lock(g_rosout_mtx);
    g_rosout_pubs.clear();
  }

  if (g_handler_installed.exchange(false)) {
    if (g_original_handler) {
      rcutils_logging_set_output_handler(g_original_handler);
      g_original_handler = nullptr;
    }
  }
}

size_t get_rosout_publisher_count()
{
  std::lock_guard<std::mutex> lock(g_rosout_mtx);
  return g_rosout_pubs.size();
}

}  // namespace agnocast
