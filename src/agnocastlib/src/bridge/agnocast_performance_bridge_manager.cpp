
#include "agnocast/bridge/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

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

  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "Performance Bridge Manager started.");

  start_ros_execution();

  event_loop_.set_mq_handler([this](int fd) { this->on_mq_request(fd); });
  event_loop_.set_signal_handler([this]() { this->on_signal(); });

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      RCLCPP_ERROR(logger_, "Event loop spin failed.");
      break;
    }

    check_and_remove_bridges();
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

  auto * msg = reinterpret_cast<MqMsgPerformanceBridge *>(buffer.data());

  std::string topic_name = static_cast<const char *>(msg->target.topic_name);
  topic_local_id_t target_id = msg->target.target_id;
  std::string message_type = static_cast<const char *>(msg->message_type);

  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "Processing MQ Request: %s (Target ID: %d)", topic_name.c_str(), target_id);

  if (msg->direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    if (active_r2a_bridges_.count(topic_name) > 0) {
      // TODO(yutarokobayashi): For debugging. Remove later.
      RCLCPP_INFO(logger_, "R2A Bridge for '%s' already exists. Skipping.", topic_name.c_str());
      return;
    }

    rclcpp::QoS qos = get_subscriber_qos(topic_name, target_id);
    auto bridge = loader_.create_r2a_bridge(container_node_, topic_name, message_type, qos);

    if (bridge) {
      active_r2a_bridges_[topic_name] = bridge;
      // TODO(yutarokobayashi): For debugging. Remove later.
      RCLCPP_INFO(logger_, "Activated R2A Bridge. Total active: %zu", active_r2a_bridges_.size());
    } else {
      RCLCPP_ERROR(logger_, "Failed to create R2A Bridge for %s", topic_name.c_str());
    }
  } else if (msg->direction == BridgeDirection::AGNOCAST_TO_ROS2) {
    if (active_a2r_bridges_.count(topic_name) > 0) {
      // TODO(yutarokobayashi): For debugging. Remove later.
      RCLCPP_INFO(logger_, "A2R Bridge for '%s' already exists. Skipping.", topic_name.c_str());
      return;
    }

    rclcpp::QoS qos = get_publisher_qos(topic_name, target_id);
    auto bridge = loader_.create_a2r_bridge(container_node_, topic_name, message_type, qos);

    if (bridge) {
      active_a2r_bridges_[topic_name] = bridge;
      // TODO(yutarokobayashi): For debugging. Remove later.
      RCLCPP_INFO(logger_, "Activated A2R Bridge. Total active: %zu", active_a2r_bridges_.size());
    } else {
      RCLCPP_ERROR(logger_, "Failed to create A2R Bridge for %s", topic_name.c_str());
    }
  } else {
    RCLCPP_ERROR(logger_, "Invalid bridge direction received: %d", (int)msg->direction);
  }
}

void PerformanceBridgeManager::on_signal()
{
  RCLCPP_INFO(logger_, "Shutdown signal received.");
  shutdown_requested_ = true;
  if (executor_) {
    executor_->cancel();
  }
}

void PerformanceBridgeManager::check_and_remove_bridges()
{
  constexpr int THRESHOLD_WITH_REVERSE = 1;
  constexpr int THRESHOLD_WITHOUT_REVERSE = 0;

  auto r2a_it = active_r2a_bridges_.begin();
  while (r2a_it != active_r2a_bridges_.end()) {
    const std::string & topic_name = r2a_it->first;
    int count = get_agnocast_subscriber_count(topic_name);
    if (count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get subscriber count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      shutdown_requested_ = true;
      return;
    }

    bool reverse_exists = (active_a2r_bridges_.count(topic_name) > 0);
    int threshold = reverse_exists ? THRESHOLD_WITH_REVERSE : THRESHOLD_WITHOUT_REVERSE;
    if (count <= threshold) {
      r2a_it = active_r2a_bridges_.erase(r2a_it);
    } else {
      ++r2a_it;
    }
  }

  auto a2r_it = active_a2r_bridges_.begin();
  while (a2r_it != active_a2r_bridges_.end()) {
    const std::string & topic_name = a2r_it->first;
    int count = get_agnocast_publisher_count(topic_name);
    if (count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get publisher count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      shutdown_requested_ = true;
      return;
    }

    bool reverse_exists = (active_r2a_bridges_.count(topic_name) > 0);
    int threshold = reverse_exists ? THRESHOLD_WITH_REVERSE : THRESHOLD_WITHOUT_REVERSE;
    if (count <= threshold) {
      a2r_it = active_a2r_bridges_.erase(a2r_it);
    } else {
      ++a2r_it;
    }
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
