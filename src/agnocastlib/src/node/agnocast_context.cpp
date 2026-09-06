#include "agnocast/node/agnocast_context.hpp"

#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast_context_internal.hpp"
#include "agnocast_signal_handler.hpp"

#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcl/logging.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>

#include <stdexcept>

namespace agnocast
{

Context g_context;
std::mutex g_context_mtx;

void Context::init(int argc, char const * const * argv)
{
  if (initialized_) {
    // parsed_arguments_ is valid if and only if a previous init() ran: rcl allocates the
    // impl even for an empty command line, and init_without_arguments() never parses one.
    if (parsed_arguments_.is_valid()) {
      throw std::runtime_error("agnocast::init() called on an already-initialized context");
    }
    throw std::runtime_error(
      "agnocast::init() was called after an agnocast::Node or an Agnocast-only executor had "
      "already brought the context up. Call it before creating either, or not at all.");
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
  // rcl_logging_configure_with_output_handler reads --disable-stdout-logs and
  // --disable-external-lib-logs from parsed_arguments_ and registers only the
  // enabled sub-handlers inside rcl_logging_multiple_output_handler, so stdout
  // can be suppressed while file logging via spdlog is preserved.
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
    parsed_arguments_.get(), &allocator, rcl_logging_multiple_output_handler);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "agnocast", "Failed to configure logging: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  initialized_ = true;
  shutdown_called_ = false;
  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
}

void Context::adopt_rclcpp_context(const rclcpp::Context::SharedPtr & rclcpp_context)
{
  if (rclcpp_context_) {
    return;
  }
  if (rclcpp_context && rclcpp_context->is_valid()) {
    rclcpp_context_ = rclcpp_context;
  }
}

bool Context::is_ok() const
{
  if (!initialized_) {
    return false;
  }
  if (rclcpp_context_) {
    return rclcpp_context_->is_valid();
  }
  return true;
}

bool Context::init_without_arguments()
{
  if (initialized_ || shutdown_called_) {
    return initialized_;
  }

  // Deliberately no parse_arguments() and no rcl_logging_configure_with_output_handler():
  // see the declaration in agnocast_context.hpp.
  initialized_ = true;
  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
  return true;
}

void Context::shutdown()
{
  // A later init() starts a fresh cycle and must not inherit a context that is on its way out.
  rclcpp_context_.reset();

  if (!initialized_) {
    return;
  }
  initialized_ = false;
  shutdown_called_ = true;
}

void init(int argc, char const * const * argv)
{
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    g_context.init(argc, argv);
  }
  SignalHandler::install();
}

void ensure_initialized(const rclcpp::Context::SharedPtr & rclcpp_context)
{
  bool initialized = false;
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    g_context.adopt_rclcpp_context(rclcpp_context);
    initialized = g_context.init_without_arguments();
  }

  // Skip while the context is shut down. Objects created during teardown must neither revive
  // agnocast::ok() nor, when the teardown came from agnocast::shutdown(), reinstall the handler
  // it has just removed.
  if (initialized) {
    SignalHandler::install();
  }
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
  return g_context.is_ok();
}

}  // namespace agnocast
