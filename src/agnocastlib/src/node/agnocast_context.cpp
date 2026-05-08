#include "agnocast/node/agnocast_context.hpp"

#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast_signal_handler.hpp"

#include <rcl/error_handling.h>
#include <rcl/logging.h>
#include <rcutils/logging_macros.h>

namespace agnocast
{

Context g_context;
std::mutex g_context_mtx;

void Context::init(int argc, char const * const * argv)
{
  if (initialized_) {
    return;
  }

  // Copy argv into a safe container to avoid pointer arithmetic
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }

  parsed_arguments_ = parse_arguments(args);

  // Initialize rcl logging so that RCLCPP_INFO/WARN/etc. are written to
  // ~/.ros/log/ files via rcl_logging_spdlog, matching rclcpp::init() behavior.
  // This also applies --log-level from parsed_arguments_ via
  // rcl_arguments_get_log_levels() internally.
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
    parsed_arguments_.get(), &allocator, rcl_logging_multiple_output_handler);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "agnocast", "Failed to configure logging: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  initialized_ = true;
  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
}

void Context::shutdown()
{
  if (!initialized_) {
    return;
  }
  initialized_ = false;
}

void init(int argc, char const * const * argv)
{
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    g_context.init(argc, argv);
  }
  SignalHandler::install();
}

void shutdown()
{
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    g_context.shutdown();
  }

  SignalHandler::notify_all_executors();
  SignalHandler::uninstall();

  rcl_ret_t ret = rcl_logging_fini();
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "agnocast", "Failed to finalize logging: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

bool ok()
{
  std::lock_guard<std::mutex> lock(g_context_mtx);
  return g_context.is_initialized();
}

}  // namespace agnocast
