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
  event_loop_(logger_),
  loader_(logger_)
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
  active_r2a_service_bridges_.clear();
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
    check_active_service_bridges();
    check_should_exit();
  }
}

void StandardBridgeManager::start_ros_execution()
{
  std::string node_name = "agnocast_bridge_node_" + std::to_string(getpid());
  container_node_ = std::make_shared<rclcpp::Node>(node_name);

  executor_ = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>();
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

    if (req.is_service) {
      // TODO(bdm-k): For debugging purposes. Remove this later.
      RCLCPP_INFO(logger_, "Received service bridge request for name='%s'", req.srv_target.name);

      activate_service_bridge(req);

      return;
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
  const auto bridge_target_key = derive_bridge_target_key(req);
  if (active_bridges_.count(bridge_target_key) != 0U) {
    return;
  }

  auto & info = managed_bridges_[derive_topic_or_service_name(req)];
  bool is_r2a = (req.direction == BridgeDirection::ROS2_TO_AGNOCAST);
  (is_r2a ? info.req_r2a : info.req_a2r) = req;
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

bool StandardBridgeManager::activate_bridge(const MqMsgBridge & req)
{
  bool is_r2a = (req.direction == BridgeDirection::ROS2_TO_AGNOCAST);
  std::string topic_name = derive_topic_or_service_name(req);
  std::string bridge_target_key = derive_bridge_target_key(req);

  if (active_bridges_.count(bridge_target_key) != 0U) {
    return true;
  }

  try {
    rclcpp::QoS target_qos = is_r2a ? get_subscriber_qos(topic_name, req.pubsub_target.target_id)
                                    : get_publisher_qos(topic_name, req.pubsub_target.target_id);

    auto bridge = loader_.create(req, bridge_target_key, container_node_, target_qos);

    if (!bridge) {
      RCLCPP_ERROR(logger_, "Failed to create bridge for '%s'", bridge_target_key.c_str());
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
    active_bridges_[bridge_target_key] = bridge;

    auto cast_bridge = std::static_pointer_cast<agnocast::BridgeBase>(bridge);

    auto callback_group = cast_bridge->get_callback_group();
    if (callback_group) {
      executor_->add_callback_group(
        callback_group, container_node_->get_node_base_interface(), true);
    }

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to activate bridge for topic '%s': %s", bridge_target_key.c_str(), e.what());
    return false;
  }
}

bool StandardBridgeManager::activate_service_bridge(const MqMsgBridge & req)
{
  std::string bridge_target_key = derive_bridge_target_key(req);

  if (active_r2a_service_bridges_.count(bridge_target_key) != 0U) {
    return true;
  }

  try {
    auto bridge = loader_.create_service(req, bridge_target_key, container_node_);

    // TODO(bdm-k): For debugging purposes. Remove this later.
    RCLCPP_INFO(logger_, "Successfully created service bridge for '%s'", req.srv_target.name);

    if (!bridge) {
      RCLCPP_ERROR(logger_, "Failed to create service bridge for '%s'", bridge_target_key.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return false;
    }

    active_r2a_service_bridges_[bridge_target_key] = bridge;

    auto cast_bridge = std::static_pointer_cast<agnocast::ServiceBridgeBase>(bridge);

    auto [cb0, cb1] = cast_bridge->get_callback_groups();
    executor_->add_callback_group(cb0, container_node_->get_node_base_interface(), true);
    executor_->add_callback_group(cb1, container_node_->get_node_base_interface(), true);

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to activate service bridge for '%s': %s", bridge_target_key.c_str(),
      e.what());
    return false;
  }
}

void StandardBridgeManager::send_delegation(const MqMsgBridge & req, pid_t owner_pid)
{
  std::string mq_name = create_mq_name_for_bridge(owner_pid);

  mqd_t mq = mq_open(mq_name.c_str(), O_WRONLY | O_NONBLOCK);
  if (mq == -1) {
    RCLCPP_WARN(
      logger_, "Failed to open delegation MQ '%s': %s, try again later.", mq_name.c_str(),
      strerror(errno));
    return;
  }

  if (mq_send(mq, reinterpret_cast<const char *>(&req), sizeof(req), 0) < 0) {
    RCLCPP_WARN(
      logger_, "Failed to send delegation request to MQ '%s': %s, try again later.",
      mq_name.c_str(), strerror(errno));
    mq_close(mq);
    return;
  }

  mq_close(mq);
}

void StandardBridgeManager::process_managed_bridge(
  const std::string & topic_name, const std::optional<MqMsgBridge> & req)
{
  if (!req) {
    return;
  }

  bool is_r2a = (req->direction == BridgeDirection::ROS2_TO_AGNOCAST);

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
      if (!activate_bridge(*req)) {
        // Rollback: remove bridge from kernel if activation failed
        rollback_bridge_from_kernel(topic_name, is_r2a);
      }
      break;

    case AddBridgeResult::EXIST:
      if (!is_active_in_owner) {
        send_delegation(*req, owner_pid);
      }
      break;

    case AddBridgeResult::ERROR:
      RCLCPP_ERROR(logger_, "Failed to add bridge for '%s'", topic_name.c_str());
      break;
  }
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
  std::vector<std::pair<std::string, bool>> to_remove;  // (key, keep_managed)
  to_remove.reserve(active_bridges_.size());

  for (const auto & [key, bridge] : active_bridges_) {
    if (key.size() <= SUFFIX_LEN) {
      continue;
    }

    std::string_view key_view = key;
    std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
    std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

    bool is_r2a = (suffix == SUFFIX_R2A);
    std::string topic_name_str(topic_name_view);

    int count = 0;
    bool is_demanded_by_ros2 = false;
    if (is_r2a) {
      count = get_agnocast_subscriber_count(topic_name_str).count;
      is_demanded_by_ros2 = has_external_ros2_publisher(container_node_.get(), topic_name_str);
      if (!update_ros2_publisher_num(container_node_.get(), topic_name_str)) {
        to_remove.emplace_back(key, false);
        continue;
      }
    } else {
      count = get_agnocast_publisher_count(topic_name_str).count;
      is_demanded_by_ros2 = has_external_ros2_subscriber(container_node_.get(), topic_name_str);
      if (!update_ros2_subscriber_num(container_node_.get(), topic_name_str)) {
        to_remove.emplace_back(key, false);
        continue;
      }
    }

    if (count <= 0) {
      if (count < 0) {
        RCLCPP_ERROR(
          logger_, "Failed to get connection count for %s. Removing bridge.", key.c_str());
      }
      to_remove.emplace_back(key, false);
    } else if (!is_demanded_by_ros2) {
      to_remove.emplace_back(key, true);  // keep_managed when only ROS 2 demand is missing
    }
  }

  for (const auto & [key, keep_managed] : to_remove) {
    remove_active_bridge(key, keep_managed);
  }
}

void StandardBridgeManager::check_active_service_bridges()
{
  auto it = active_r2a_service_bridges_.begin();
  while (it != active_r2a_service_bridges_.end()) {
    const std::string & key = it->first;

    std::string_view key_view(key);
    std::string_view service_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

    std::string service_name(service_name_view);

    auto result = get_agnocast_subscriber_count(create_service_request_topic_name(service_name));
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get subscriber count for topic '%s'. Requesting shutdown.",
        create_service_request_topic_name(service_name).c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return;
    }

    if (result.count <= 0) {
      // TODO(bdm-k): For debugging purposes. Remove this later.
      RCLCPP_INFO(
        logger_, "Removing service bridge for '%s' because the service is down.",
        service_name.c_str());

      it = active_r2a_service_bridges_.erase(it);
    } else {
      ++it;
    }
  }
}

void StandardBridgeManager::check_managed_bridges()
{
  for (auto it = managed_bridges_.begin(); it != managed_bridges_.end();) {
    if (shutdown_requested_) {
      break;
    }

    const auto & topic_name = it->first;
    auto & info = it->second;

    // Clean up requests when Agnocast entity no longer exists (count == 0)
    // Note: count < 0 indicates an error, so we keep the request in that case
    if (info.req_r2a && get_agnocast_subscriber_count(topic_name).count == 0) {
      info.req_r2a.reset();
    }
    if (info.req_a2r && get_agnocast_publisher_count(topic_name).count == 0) {
      info.req_a2r.reset();
    }

    if (!info.req_r2a && !info.req_a2r) {
      it = managed_bridges_.erase(it);
      continue;
    }

    process_managed_bridge(topic_name, info.req_r2a);
    process_managed_bridge(topic_name, info.req_a2r);
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

void StandardBridgeManager::remove_active_bridge(
  const std::string & bridge_target_key, bool keep_managed)
{
  if (bridge_target_key.size() <= SUFFIX_LEN) {
    return;
  }

  if (active_bridges_.count(bridge_target_key) == 0) {
    return;
  }

  std::string_view key_view(bridge_target_key);
  std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
  std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

  bool is_r2a = (suffix == SUFFIX_R2A);

  struct ioctl_remove_bridge_args remove_bridge_args
  {
  };
  remove_bridge_args.topic_name = {topic_name_view.data(), topic_name_view.size()};
  remove_bridge_args.is_r2a = is_r2a;

  if (ioctl(agnocast_fd, AGNOCAST_REMOVE_BRIDGE_CMD, &remove_bridge_args) != 0) {
    RCLCPP_ERROR(
      logger_, "AGNOCAST_REMOVE_BRIDGE_CMD failed for topic '%s': %s",
      std::string(topic_name_view).c_str(), strerror(errno));
  }

  active_bridges_.erase(bridge_target_key);

  if (keep_managed) {
    return;
  }

  std::string raw_topic_name(topic_name_view);
  auto it = managed_bridges_.find(raw_topic_name);

  if (it != managed_bridges_.end()) {
    if (is_r2a) {
      it->second.req_r2a.reset();
    } else {
      it->second.req_a2r.reset();
    }

    if (!it->second.req_r2a && !it->second.req_a2r) {
      managed_bridges_.erase(it);
    }
  }
}

std::string StandardBridgeManager::derive_topic_or_service_name(const MqMsgBridge & req)
{
  std::string raw_name{
    static_cast<const char *>(req.is_service ? req.srv_target.name : req.pubsub_target.topic_name)};
  return raw_name;
}

std::string StandardBridgeManager::derive_bridge_target_key(const MqMsgBridge & req)
{
  std::string raw_name{
    static_cast<const char *>(req.is_service ? req.srv_target.name : req.pubsub_target.topic_name)};
  const bool is_r2a = (req.direction == BridgeDirection::ROS2_TO_AGNOCAST);
  const std::string_view suffix = req.is_service ? (is_r2a ? SUFFIX_SRV_R2A : SUFFIX_SRV_A2R)
                                                 : (is_r2a ? SUFFIX_R2A : SUFFIX_A2R);
  return raw_name + std::string(suffix);
}

}  // namespace agnocast
