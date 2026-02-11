#include "agnocast/node/agnocast_context.hpp"
#include <rcl/log_level.h>
#include <rcutils/logging.h>

#include "agnocast/agnocast_tracepoint_wrapper.h"

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

  // Apply --log-level settings from parsed arguments
  rcl_log_levels_t log_levels = rcl_get_zero_initialized_log_levels();
  rcl_ret_t ret = rcl_arguments_get_log_levels(parsed_arguments_.get(), &log_levels);
  if (RCL_RET_OK == ret) {
    if (log_levels.default_logger_level != RCUTILS_LOG_SEVERITY_UNSET) {
      rcutils_logging_set_default_logger_level(static_cast<int>(log_levels.default_logger_level));
    }
    for (size_t i = 0; i < log_levels.num_logger_settings; ++i) {
      rcutils_logging_set_logger_level(
        log_levels.logger_settings[i].name, static_cast<int>(log_levels.logger_settings[i].level));
    }
    rcl_log_levels_fini(&log_levels);
  } else {
    rcl_reset_error();
  }

  initialized_ = true;
}

void init(int argc, char const * const * argv)
{
  std::lock_guard<std::mutex> lock(g_context_mtx);
  g_context.init(argc, argv);

  TRACEPOINT(agnocast_init, static_cast<const void *>(&g_context));
}

}  // namespace agnocast
