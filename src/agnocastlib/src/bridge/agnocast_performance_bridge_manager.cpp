
#include "agnocast/bridge/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <mqueue.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <vector>

namespace agnocast
{

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
  if (executor_) {
    executor_->cancel();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

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

  start_ros_execution();

  event_loop_.set_mq_handler([this](int fd) { this->on_mq_request(fd); });
  event_loop_.set_signal_handler([this]() { this->on_signal(); });
  event_loop_.set_reload_handler([this]() { this->on_reload(); });

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      RCLCPP_ERROR(logger_, "Event loop spin failed.");
      break;
    }

    check_and_request_shutdown();
  }
}

void PerformanceBridgeManager::start_ros_execution()
{
  std::string node_name = "agnocast_bridge_node_" + std::to_string(getpid());
  container_node_ = std::make_shared<rclcpp::Node>(node_name);

  // TODO(yutarokobayashi): Select executor type based on config
  executor_ = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>();
  executor_->add_node(container_node_);

  executor_thread_ = std::thread([this]() {
    try {
      this->executor_->spin();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Executor Thread CRASHED: %s", e.what());
    }
  });
}

void PerformanceBridgeManager::on_mq_request(int fd)
{
  std::vector<char> buffer(PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE);

  ssize_t bytes_read = mq_receive(fd, buffer.data(), buffer.size(), nullptr);

  if (bytes_read < 0) {
    if (errno != EAGAIN) {
      RCLCPP_WARN(logger_, "mq_receive failed: %s", strerror(errno));
    }
    return;
  }

  // TODO(yutarokobayashi): Process the received message
  RCLCPP_INFO(logger_, "Received MQ message (%zd bytes)", bytes_read);
}

void PerformanceBridgeManager::on_signal()
{
  RCLCPP_INFO(logger_, "Shutdown signal received.");
  shutdown_requested_ = true;
  if (executor_) {
    executor_->cancel();
  }
}

void PerformanceBridgeManager::on_reload()
{
  // TODO(yutarokobayashi): Implement configuration reload
  RCLCPP_INFO(logger_, "Reload signal (SIGHUP) received.");
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
