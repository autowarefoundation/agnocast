#include "agnocast/bridge/standard/agnocast_standard_bridge_manager.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <sys/prctl.h>
#include <unistd.h>

#include <csignal>
#include <stdexcept>
#include <string>

namespace agnocast
{

StandardBridgeManager::StandardBridgeManager(pid_t target_pid)
: target_pid_(target_pid),
  logger_(rclcpp::get_logger("agnocast_standard_bridge_manager")),
  event_loop_(logger_)
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  rclcpp::InitOptions init_options{};
  init_options.shutdown_on_signal = false;
  rclcpp::init(0, nullptr, init_options);
}

StandardBridgeManager::~StandardBridgeManager()
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    try {
      executor_thread_.join();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("StandardBridgeManager"), "Failed to join thread: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(
        rclcpp::get_logger("StandardBridgeManager"), "Failed to join thread: unknown error");
    }
  }

  active_bridges_.clear();
  container_node_.reset();
  executor_.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void StandardBridgeManager::run()
{
  constexpr int EVENT_LOOP_TIMEOUT_MS = 1000;

  std::string proc_name = "agno_br_" + std::to_string(getpid());
  prctl(PR_SET_NAME, proc_name.c_str(), 0, 0, 0);

  start_ros_execution();

  event_loop_.set_mq_handler([this](int fd) { this->on_mq_request(fd); });
  event_loop_.set_signal_handler([this]() { this->on_signal(); });

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      break;
    }

    check_parent_alive();
    check_managed_bridges();
    check_active_bridges();
    check_should_exit();
  }
}

void StandardBridgeManager::start_ros_execution()
{
  std::string node_name = "agnocast_bridge_node_" + std::to_string(getpid());
  container_node_ = std::make_shared<rclcpp::Node>(node_name);
  loader_ = std::make_unique<StandardBridgeLoader>(container_node_, logger_);

  // We must not use single-threaded executors because of how service bridges work. Service bridges
  // require two callback groups to execute concurrently. If a single-threaded executor is used, it
  // can deadlock. See the service bridge implementation for details.
  executor_ = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  executor_->add_node(container_node_);

  executor_thread_ = std::thread([this]() {
    try {
      this->executor_->spin();
    } catch (const std::exception & e) {
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      RCLCPP_ERROR(logger_, "Executor Thread CRASHED: %s", e.what());
    }
  });
}

void StandardBridgeManager::on_mq_request(mqd_t fd)
{
  MqMsgBridge req{};
  while (mq_receive(fd, reinterpret_cast<char *>(&req), sizeof(req), nullptr) > 0) {
    if (shutdown_requested_) {
      break;
    }
    register_request(req);
  }
}

void StandardBridgeManager::on_signal()
{
  if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
    RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
  }
  shutdown_requested_ = true;
  if (executor_) {
    executor_->cancel();
  }
}

void StandardBridgeManager::register_request(const MqMsgBridge & req)
{
  // Locally, unique keys include the direction. However, we register the raw topic name (without
  // direction) to the kernel to enforce single-process ownership for the entire topic.
  const auto [topic_name, topic_name_with_direction] = extract_topic_info(req);
  if (active_bridges_.count(topic_name_with_direction) != 0U) {
    return;
  }

  auto it = managed_bridges_.find(topic_name);
  if (it == managed_bridges_.end()) {
    if (*static_cast<const char *>(req.factory.shared_lib_path) == '\0') {
      RCLCPP_WARN(
        logger_,
        "Skipping %s bridge request for new topic '%s' due to missing factory information. "
        "This occurs when delegating a request to a bridge manager that has already removed "
        "the topic from its managed bridges.",
        req.direction == BridgeDirection::ROS2_TO_AGNOCAST ? "R2A" : "A2R", topic_name.c_str());
      return;
    }

    auto & entry = managed_bridges_[topic_name];

    if (
      std::strcmp(static_cast<const char *>(req.factory.symbol_name), MAIN_EXECUTABLE_SYMBOL) ==
      0) {
      entry.factory_spec.shared_lib_path = std::nullopt;
    } else {
      entry.factory_spec.shared_lib_path =
        std::string(static_cast<const char *>(req.factory.shared_lib_path));
    }

    if (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
      entry.factory_spec.fn_offset_r2a = req.factory.fn_offset;
      entry.factory_spec.fn_offset_a2r = req.factory.fn_offset_reverse;
      entry.target_id_r2a = req.target.target_id;
      entry.is_requested_r2a = true;
      entry.reset_a2r();
    } else {
      entry.factory_spec.fn_offset_r2a = req.factory.fn_offset_reverse;
      entry.factory_spec.fn_offset_a2r = req.factory.fn_offset;
      entry.target_id_a2r = req.target.target_id;
      entry.is_requested_a2r = true;
      entry.reset_r2a();
    }
  } else {
    auto & entry = it->second;

    if (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
      entry.target_id_r2a = req.target.target_id;
      entry.is_requested_r2a = true;
    } else {
      entry.target_id_a2r = req.target.target_id;
      entry.is_requested_a2r = true;
    }
  }
}

StandardBridgeManager::BridgeKernelResult StandardBridgeManager::try_add_bridge_to_kernel(
  const std::string & topic_name, bool is_r2a)
{
  struct ioctl_add_bridge_args add_bridge_args
  {
  };
  add_bridge_args.topic_name = {topic_name.c_str(), topic_name.size()};
  add_bridge_args.is_r2a = is_r2a;

  int ret = ioctl(agnocast_fd, AGNOCAST_ADD_BRIDGE_CMD, &add_bridge_args);

  if (ret == 0 || errno == EEXIST) {
    return BridgeKernelResult{
      (ret == 0) ? AddBridgeResult::SUCCESS : AddBridgeResult::EXIST, add_bridge_args.ret_pid,
      add_bridge_args.ret_has_r2a, add_bridge_args.ret_has_a2r};
  }

  return BridgeKernelResult{AddBridgeResult::ERROR, 0, false, false};
}

void StandardBridgeManager::rollback_bridge_from_kernel(const std::string & topic_name, bool is_r2a)
{
  struct ioctl_remove_bridge_args remove_bridge_args
  {
  };
  remove_bridge_args.topic_name = {topic_name.c_str(), topic_name.size()};
  remove_bridge_args.is_r2a = is_r2a;

  if (ioctl(agnocast_fd, AGNOCAST_REMOVE_BRIDGE_CMD, &remove_bridge_args) < 0) {
    RCLCPP_ERROR(
      logger_, "Rollback AGNOCAST_REMOVE_BRIDGE_CMD failed for topic '%s': %s", topic_name.c_str(),
      strerror(errno));
  }
}

bool StandardBridgeManager::activate_bridge(const DirectedBridgeRef bridge_ref)
{
  const auto & [topic_name, entry, direction] = bridge_ref;

  bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);
  std::string_view suffix = is_r2a ? SUFFIX_R2A : SUFFIX_A2R;
  std::string topic_name_with_direction = topic_name + std::string(suffix);

  if (active_bridges_.count(topic_name_with_direction) != 0U) {
    return true;
  }

  try {
    rclcpp::QoS target_qos = is_r2a ? get_subscriber_qos(topic_name, entry.target_id_r2a)
                                    : get_publisher_qos(topic_name, entry.target_id_a2r);

    auto bridge = loader_->create(topic_name, direction, entry.factory_spec, target_qos);

    if (!bridge) {
      RCLCPP_ERROR(logger_, "Failed to create bridge for '%s'", topic_name_with_direction.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return false;
    }

    if (is_r2a) {
      if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
      }
    } else {
      if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
      }
    }
    active_bridges_[topic_name_with_direction] = bridge;

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to activate bridge for topic '%s': %s", topic_name_with_direction.c_str(),
      e.what());
    return false;
  }
}

void StandardBridgeManager::send_delegation(const DirectedBridgeRef bridge_ref, pid_t owner_pid)
{
  const auto & [topic_name, entry, direction] = bridge_ref;

  std::string mq_name = create_mq_name_for_bridge(owner_pid);

  mqd_t mq = mq_open(mq_name.c_str(), O_WRONLY | O_NONBLOCK);
  if (mq == -1) {
    RCLCPP_WARN(
      logger_, "Failed to open delegation MQ '%s': %s, try again later.", mq_name.c_str(),
      strerror(errno));
    return;
  }

  /* --- Construct request --- */
  MqMsgBridge req{};
  req.direction = direction;
  req.target.target_id =
    (direction == BridgeDirection::ROS2_TO_AGNOCAST) ? entry.target_id_r2a : entry.target_id_a2r;
  snprintf(
    static_cast<char *>(req.target.topic_name), TOPIC_NAME_BUFFER_SIZE, "%s", topic_name.c_str());
  // req.factory can be left zeroed because it is not going to be used.
  /* ------------------------- */

  if (mq_send(mq, reinterpret_cast<const char *>(&req), sizeof(req), 0) < 0) {
    RCLCPP_WARN(
      logger_, "Failed to send delegation request to MQ '%s': %s, try again later.",
      mq_name.c_str(), strerror(errno));
    mq_close(mq);
    return;
  }

  mq_close(mq);
}

void StandardBridgeManager::process_managed_bridge(const DirectedBridgeRef bridge_ref)
{
  const auto & [topic_name, entry, direction] = bridge_ref;

  bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);

  if (is_r2a && !entry.is_requested_r2a) {
    return;
  }
  if (!is_r2a && !entry.is_requested_a2r) {
    return;
  }

  // Check demand before adding bridge to kernel to avoid unnecessary add+remove cycles
  if (
    (is_r2a ? get_agnocast_subscriber_count(topic_name).count
            : get_agnocast_publisher_count(topic_name).count) <= 0) {
    return;
  }
  if (
    is_r2a ? !has_external_ros2_publisher(container_node_.get(), topic_name)
           : !has_external_ros2_subscriber(container_node_.get(), topic_name)) {
    return;
  }

  auto [status, owner_pid, kernel_has_r2a, kernel_has_a2r] =
    try_add_bridge_to_kernel(topic_name, is_r2a);
  bool is_active_in_owner = is_r2a ? kernel_has_r2a : kernel_has_a2r;

  switch (status) {
    case AddBridgeResult::SUCCESS:
      if (!activate_bridge(bridge_ref)) {
        // Rollback: remove bridge from kernel if activation failed
        rollback_bridge_from_kernel(topic_name, is_r2a);
      }
      break;

    case AddBridgeResult::EXIST:
      if (!is_active_in_owner) {
        send_delegation(bridge_ref, owner_pid);
      }
      break;

    case AddBridgeResult::ERROR:
      RCLCPP_ERROR(logger_, "Failed to add bridge for '%s'", topic_name.c_str());
      break;
  }
}

bool StandardBridgeManager::should_remove_bridge(const std::string & topic_name, bool is_r2a)
{
  int count = 0;
  bool is_demanded_by_ros2 = false;
  if (is_r2a) {
    count = get_agnocast_subscriber_count(topic_name).count;
    is_demanded_by_ros2 = has_external_ros2_publisher(container_node_.get(), topic_name);
    if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
      RCLCPP_ERROR(
        logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
    }
  } else {
    count = get_agnocast_publisher_count(topic_name).count;
    is_demanded_by_ros2 = has_external_ros2_subscriber(container_node_.get(), topic_name);
    if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
      RCLCPP_ERROR(
        logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
    }
  }

  if (count <= 0) {
    if (count < 0) {
      RCLCPP_ERROR(
        logger_, "Failed to get connection count for %s. Removing %s bridge.", topic_name.c_str(),
        is_r2a ? "R2A" : "A2R");
    }
    return true;
  }

  return !is_demanded_by_ros2;
}

void StandardBridgeManager::check_parent_alive()
{
  if (!is_parent_alive_) {
    return;
  }
  if (kill(target_pid_, 0) != 0) {
    is_parent_alive_ = false;
    managed_bridges_.clear();
  }
}

void StandardBridgeManager::check_active_bridges()
{
  for (auto it = active_bridges_.begin(); it != active_bridges_.end();) {
    const std::string & key = it->first;
    const std::shared_ptr<BridgeBase> & bridge = it->second;
    if (key.size() <= SUFFIX_LEN) {
      ++it;
      continue;
    }

    std::string_view key_view = key;
    std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
    std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

    bool is_r2a = (suffix == SUFFIX_R2A);
    std::string topic_name_str(topic_name_view);

    if (!should_remove_bridge(topic_name_str, is_r2a)) {
      ++it;
      continue;
    }

    // Unregister the bridge from kernel module.
    ioctl_remove_bridge_args args{};
    args.topic_name = {topic_name_view.data(), topic_name_view.size()};
    args.is_r2a = is_r2a;
    if (ioctl(agnocast_fd, AGNOCAST_REMOVE_BRIDGE_CMD, &args) != 0) {
      RCLCPP_ERROR(
        logger_, "AGNOCAST_REMOVE_BRIDGE_CMD failed for key '%s': %s", key.c_str(),
        strerror(errno));
    }

    // Stop the child executor for this bridge's callback group before destroying the bridge.
    // This ensures any in-flight callback completes before the subscription is destroyed,
    // preventing use-after-free when the subscriber's reference bits are cleared by the kernel.
    auto cb_group = bridge->get_callback_group();
    if (cb_group) {
      executor_->stop_callback_group(cb_group);
    }

    // Erase the bridge in-place.
    it = active_bridges_.erase(it);
  }
}

void StandardBridgeManager::check_managed_bridges()
{
  for (auto it = managed_bridges_.begin(); it != managed_bridges_.end();) {
    if (shutdown_requested_) {
      break;
    }

    const auto & topic_name = it->first;
    auto & entry = it->second;

    // Clean up requests when Agnocast entity no longer exists (count == 0)
    // Note: count < 0 indicates an error, so we keep the request in that case
    if (entry.is_requested_r2a && get_agnocast_subscriber_count(topic_name).count == 0) {
      entry.reset_r2a();
    }
    if (entry.is_requested_a2r && get_agnocast_publisher_count(topic_name).count == 0) {
      entry.reset_a2r();
    }

    if (!entry.is_requested_r2a && !entry.is_requested_a2r) {
      it = managed_bridges_.erase(it);
      continue;
    }

    process_managed_bridge(DirectedBridgeRef{topic_name, entry, BridgeDirection::ROS2_TO_AGNOCAST});
    process_managed_bridge(DirectedBridgeRef{topic_name, entry, BridgeDirection::AGNOCAST_TO_ROS2});
    ++it;
  }
}

void StandardBridgeManager::check_should_exit()
{
  if (!is_parent_alive_ && active_bridges_.empty()) {
    if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
      RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
    }
    shutdown_requested_ = true;
    if (executor_) {
      executor_->cancel();
    }
  }
}

std::pair<std::string, std::string> StandardBridgeManager::extract_topic_info(
  const MqMsgBridge & req)
{
  std::string raw_name(
    &req.target.topic_name[0], strnlen(&req.target.topic_name[0], sizeof(req.target.topic_name)));

  std::string_view suffix =
    (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) ? SUFFIX_R2A : SUFFIX_A2R;

  return {raw_name, raw_name + std::string(suffix)};
}

}  // namespace agnocast
