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
    // rcl allocates the arguments impl even for an empty command line, so this means init() ran.
    if (parsed_arguments_.is_valid()) {
      throw std::runtime_error("agnocast::init() called on an already-initialized context");
    }
    throw std::runtime_error(
      "agnocast::init() was called after an agnocast::Node or an Agnocast-only executor had "
      "already brought the context up. Call it before creating either; in a process rclcpp "
      "started, do not call it at all.");
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
  } else {
    configured_logging_ = true;
  }

  initialized_ = true;
  shutdown_called_ = false;
  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
}

bool Context::take_configured_logging()
{
  const bool configured = configured_logging_;
  configured_logging_ = false;
  return configured;
}

bool Context::adopt_rclcpp_context(const rclcpp::Context::SharedPtr & rclcpp_context)
{
  if (adopted_) {
    return false;
  }
  if (rclcpp_context && rclcpp_context->is_valid()) {
    rclcpp_context_ = rclcpp_context;
    adopted_ = true;
  }
  return adopted_;
}

bool Context::is_ok(rclcpp::Context::SharedPtr & governing_context) const
{
  if (!initialized_) {
    return false;
  }
  if (!adopted_) {
    return true;
  }
  governing_context = rclcpp_context_.lock();
  return governing_context && governing_context->is_valid();
}

bool Context::init_without_arguments()
{
  if (initialized_ || shutdown_called_) {
    return initialized_;
  }

  initialized_ = true;
  TRACEPOINT(agnocast_init, static_cast<const void *>(this));
  return true;
}

void Context::shutdown()
{
  // A later init() starts a fresh cycle and must not inherit a context that is on its way out.
  rclcpp_context_.reset();
  adopted_ = false;

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
  bool hook_teardown_onto_rclcpp = false;

  {
    // install() runs under g_context_mtx so a concurrent shutdown() cannot uninstall between the
    // decision and the call. SignalHandler::mutex_ is never taken before g_context_mtx, so
    // nesting them this way cannot deadlock.
    std::lock_guard<std::mutex> lock(g_context_mtx);

    // Skip while the context is shut down.
    if (g_context.init_without_arguments()) {
      const bool adopted = g_context.adopt_rclcpp_context(rclcpp_context);
      SignalHandler::install();
      // Only hook the teardown on when the context came up lazily. A process that called
      // agnocast::init() owns the teardown itself, and its handler went in underneath rclcpp's,
      // where coming off from a shutdown callback would restore a handler rclcpp is still using.
      hook_teardown_onto_rclcpp = adopted && g_context.get_parsed_arguments() == nullptr;
    }
  }

  // Nothing else calls agnocast::shutdown() in a process that never called agnocast::init().
  // rclcpp runs these callbacks before uninstalling its own signal handlers, so Agnocast's comes
  // off first -- the reverse of the order they went on in.
  //
  // Registered outside g_context_mtx, which the callback takes: rclcpp runs the callbacks under
  // its own lock, so acquiring that lock while holding g_context_mtx would invert the order.
  if (hook_teardown_onto_rclcpp) {
    rclcpp_context->add_on_shutdown_callback([]() { agnocast::shutdown(); });
  }
}

void shutdown()
{
  bool finalize_logging = false;
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    finalize_logging = g_context.take_configured_logging();
    g_context.shutdown();
  }

  SignalHandler::notify_all_executors();
  SignalHandler::uninstall();

  if (finalize_logging) {
    rcl_ret_t ret = rcl_logging_fini();
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "agnocast", "Failed to finalize logging: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
}

bool ok()
{
  // Declared before the lock so that it outlives it: dropping the last reference to the governing
  // context runs its shutdown callbacks, and ours takes g_context_mtx.
  rclcpp::Context::SharedPtr governing_context;
  std::lock_guard<std::mutex> lock(g_context_mtx);
  return g_context.is_ok(governing_context);
}

}  // namespace agnocast
