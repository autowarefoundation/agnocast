
#include "agnocast/bridge/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <mqueue.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
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

    check_and_create_bridges();
    check_and_remove_bridges();
    check_and_remove_request_cache();
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

  request_cache_[topic_name][target_id] = *msg;

  create_bridge_if_needed(topic_name, request_cache_[topic_name], message_type, msg->direction);
}

void PerformanceBridgeManager::on_signal()
{
  RCLCPP_INFO(logger_, "Shutdown signal received.");
  shutdown_requested_ = true;
  if (executor_) {
    executor_->cancel();
  }
}

void PerformanceBridgeManager::check_and_create_bridges()
{
  for (auto cache_it = request_cache_.begin(); cache_it != request_cache_.end();) {
    const auto & topic_name = cache_it->first;
    auto & requests = cache_it->second;

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
      continue;
    }

    const std::string message_type =
      static_cast<const char *>(requests.begin()->second.message_type);

    create_bridge_if_needed(topic_name, requests, message_type, BridgeDirection::ROS2_TO_AGNOCAST);
    create_bridge_if_needed(topic_name, requests, message_type, BridgeDirection::AGNOCAST_TO_ROS2);

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
    } else {
      ++cache_it;
    }
  }
}

void PerformanceBridgeManager::check_and_remove_bridges()
{
  auto r2a_it = active_r2a_bridges_.begin();
  while (r2a_it != active_r2a_bridges_.end()) {
    const std::string & topic_name = r2a_it->first;
    auto result = get_agnocast_subscriber_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_publisher(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get subscriber count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      shutdown_requested_ = true;
      return;
    }

    const int threshold = result.bridge_exist ? 1 : 0;
    if (result.count <= threshold || !is_demanded_by_ros2) {
      r2a_it = active_r2a_bridges_.erase(r2a_it);
    } else {
      ++r2a_it;
    }
  }

  auto a2r_it = active_a2r_bridges_.begin();
  while (a2r_it != active_a2r_bridges_.end()) {
    const std::string & topic_name = a2r_it->first;
    auto result = get_agnocast_publisher_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_subscriber(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get publisher count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      shutdown_requested_ = true;
      return;
    }

    const int threshold = result.bridge_exist ? 1 : 0;
    if (result.count <= threshold || !is_demanded_by_ros2) {
      a2r_it = active_a2r_bridges_.erase(a2r_it);
    } else {
      ++a2r_it;
    }
  }
}

void PerformanceBridgeManager::check_and_remove_request_cache()
{
  for (auto cache_it = request_cache_.begin(); cache_it != request_cache_.end();) {
    const auto & topic_name = cache_it->first;
    auto & requests = cache_it->second;

    remove_invalid_requests(topic_name, requests);

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
    } else {
      ++cache_it;
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

bool PerformanceBridgeManager::should_create_bridge(
  const std::string & topic_name, BridgeDirection direction) const
{
  if (direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    if (active_r2a_bridges_.count(topic_name) > 0) {
      return false;
    }

    const auto stats = get_agnocast_subscriber_count(topic_name);
    const int threshold = stats.bridge_exist ? 1 : 0;
    if (stats.count == -1 || stats.count <= threshold) {
      return false;
    }

    return has_external_ros2_publisher(container_node_.get(), topic_name);
  }
  if (active_a2r_bridges_.count(topic_name) > 0) {
    return false;
  }

  const auto stats = get_agnocast_publisher_count(topic_name);
  const int threshold = stats.bridge_exist ? 1 : 0;
  if (stats.count == -1 || stats.count <= threshold) {
    return false;
  }

  return agnocast::has_external_ros2_subscriber(container_node_.get(), topic_name);
}

void PerformanceBridgeManager::create_bridge_if_needed(
  const std::string & topic_name, RequestMap & requests, const std::string & message_type,
  BridgeDirection direction)
{
  if (!should_create_bridge(topic_name, direction)) {
    return;
  }

  topic_local_id_t qos_source_id = -1;
  for (const auto & [id, req] : requests) {
    if (req.direction == direction) {
      qos_source_id = id;
      break;
    }
  }
  if (qos_source_id == -1) {
    return;
  }

  try {
    const bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);

    PerformanceBridgeResult result;
    if (is_r2a) {
      auto qos = get_subscriber_qos(topic_name, qos_source_id);
      result = loader_.create_r2a_bridge(container_node_, topic_name, message_type, qos);
    } else {
      auto qos = get_publisher_qos(topic_name, qos_source_id);
      result = loader_.create_a2r_bridge(container_node_, topic_name, message_type, qos);
    }

    if (result.entity_handle) {
      if (is_r2a) {
        active_r2a_bridges_[topic_name] = result.entity_handle;
      } else {
        active_a2r_bridges_[topic_name] = result.entity_handle;
      }

      if (result.callback_group) {
        executor_->add_callback_group(
          result.callback_group, container_node_->get_node_base_interface(), true);
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to create bridge for '%s': %s. Removing invalid request ID %d.",
      topic_name.c_str(), e.what(), qos_source_id);
    requests.erase(qos_source_id);
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Unknown error creating bridge for '%s'. Removing invalid request ID %d.",
      topic_name.c_str(), qos_source_id);
    requests.erase(qos_source_id);
  }
}

void PerformanceBridgeManager::remove_invalid_requests(
  const std::string & topic_name, RequestMap & request_map)
{
  for (auto req_it = request_map.begin(); req_it != request_map.end();) {
    const auto target_id = req_it->first;
    const auto & msg = req_it->second;

    // Verify liveness by attempting to retrieve QoS.
    // If the target no longer exists, an exception is thrown.
    try {
      if (msg.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
        get_subscriber_qos(topic_name, target_id);
      } else {
        get_publisher_qos(topic_name, target_id);
      }
      ++req_it;
    } catch (...) {
      // TODO(yutarokobayashi): For debugging. Remove later.
      RCLCPP_INFO(
        logger_, "Removed dead ID %d from cache for topic %s", target_id, topic_name.c_str());
      req_it = request_map.erase(req_it);
    }
  }
}

}  // namespace agnocast
