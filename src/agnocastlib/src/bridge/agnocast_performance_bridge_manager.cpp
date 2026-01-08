
#include "agnocast/bridge/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <sys/prctl.h>
#include <unistd.h>

namespace agnocast
{

extern int agnocast_fd;

PerformanceBridgeManager::PerformanceBridgeManager()
: logger_(rclcpp::get_logger("agnocast_performance_bridge_manager")),
  event_loop_(logger_),
  loader_(logger_),
  config_(logger_)
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  rclcpp::InitOptions init_options{};
  init_options.shutdown_on_signal = false;
  rclcpp::init(0, nullptr, init_options);

  RCLCPP_INFO(logger_, "Performance Bridge Manager initialized (PID: %d).", getpid());
}

PerformanceBridgeManager::~PerformanceBridgeManager()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void PerformanceBridgeManager::run()
{
  constexpr int EVENT_LOOP_TIMEOUT_MS = 1000;

  std::string proc_name = "agno_pbr_" + std::to_string(getpid());
  prctl(PR_SET_NAME, proc_name.c_str(), 0, 0, 0);

  RCLCPP_INFO(logger_, "Performance Bridge Manager started.");

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      RCLCPP_ERROR(logger_, "Event loop spin failed.");
      break;
    }

    check_and_request_shutdown();
  }
}

void PerformanceBridgeManager::check_and_request_shutdown()
{
  struct ioctl_get_process_num_args args = {};
  if (ioctl(agnocast_fd, AGNOCAST_GET_PROCESS_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger_, "Failed to get active process count from kernel module.");
    return;
  }

  // Request shutdown if there is no other process excluding poll_for_unlink.
  if (args.ret_process_num <= 1) {
    shutdown_requested_ = true;
  }
}

}  // namespace agnocast
