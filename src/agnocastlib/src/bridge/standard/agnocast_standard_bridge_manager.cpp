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
  // Optimization: Fail-fast to avoid rclcpp::init overhead.
  // Note that the process ensures correct termination even without this check.
  if (kill(target_pid_, 0) != 0) {
    throw std::runtime_error("Target parent process is already dead.");
  }

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

  executor_ = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>();
  executor_->add_node(container_node_);

  executor_thread_ = std::thread([this]() {
    try {
      this->executor_->spin();
    } catch (const std::exception & e) {
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

  auto & info = managed_bridges_[topic_name];
  bool is_r2a = (req.direction == BridgeDirection::ROS2_TO_AGNOCAST);
  (is_r2a ? info.req_r2a : info.req_a2r) = req;
}

StandardBridgeManager::BridgeKernelResult StandardBridgeManager::try_add_bridge_to_kernel(
  const std::string & topic_name, bool is_r2a)
{
  struct ioctl_add_bridge_args add_bridge_args
  {
  };
  add_bridge_args.pid = getpid();
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

void StandardBridgeManager::activate_bridge(const MqMsgBridge & req, const std::string & topic_name)
{
  bool is_r2a = (req.direction == BridgeDirection::ROS2_TO_AGNOCAST);
  std::string_view suffix = is_r2a ? SUFFIX_R2A : SUFFIX_A2R;
  std::string topic_name_with_direction = topic_name + std::string(suffix);

  if (active_bridges_.count(topic_name_with_direction) != 0U) {
    return;
  }

  if (
    (is_r2a ? get_agnocast_subscriber_count(topic_name).count
            : get_agnocast_publisher_count(topic_name).count) <= 0) {
    return;
  }

  try {
    rclcpp::QoS target_qos = is_r2a ? get_subscriber_qos(topic_name, req.target.target_id)
                                    : get_publisher_qos(topic_name, req.target.target_id);

    auto bridge = loader_.create(req, topic_name_with_direction, container_node_, target_qos);

    if (!bridge) {
      RCLCPP_ERROR(logger_, "Failed to create bridge for '%s'", topic_name_with_direction.c_str());
      shutdown_requested_ = true;
      return;
    }

    active_bridges_[topic_name_with_direction] = bridge;

    auto cast_bridge = std::static_pointer_cast<agnocast::BridgeBase>(bridge);

    auto callback_group = cast_bridge->get_callback_group();
    if (callback_group) {
      executor_->add_callback_group(
        callback_group, container_node_->get_node_base_interface(), true);
    }

  } catch (const std::exception &) {
    return;
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
  auto [status, owner_pid, kernel_has_r2a, kernel_has_a2r] =
    try_add_bridge_to_kernel(topic_name, is_r2a);
  bool is_active_in_owner = is_r2a ? kernel_has_r2a : kernel_has_a2r;

  switch (status) {
    case AddBridgeResult::SUCCESS:
      activate_bridge(*req, topic_name);
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
  std::vector<std::string> to_remove;
  to_remove.reserve(active_bridges_.size());

  for (const auto & [key, bridge] : active_bridges_) {
    if (key.size() <= SUFFIX_LEN) {
      continue;
    }

    std::string_view key_view = key;
    std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
    std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

    bool is_r2a = (suffix == SUFFIX_R2A);

    int count = 0;
    if (is_r2a) {
      count = get_agnocast_subscriber_count(std::string(topic_name_view)).count;
    } else {
      count = get_agnocast_publisher_count(std::string(topic_name_view)).count;
      if (!update_ros2_subscriber_num(std::string(topic_name_view))) {
        to_remove.push_back(key);
        continue;
      }
    }

    if (count <= 0) {
      if (count < 0) {
        RCLCPP_ERROR(
          logger_, "Failed to get connection count for %s. Removing bridge.", key.c_str());
      }
      to_remove.push_back(key);
    }
  }

  for (const auto & key : to_remove) {
    remove_active_bridge(key);
  }
}

void StandardBridgeManager::check_managed_bridges()
{
  for (auto & managed_bridge : managed_bridges_) {
    if (shutdown_requested_) {
      break;
    }

    const auto & topic_name = managed_bridge.first;
    auto & info = managed_bridge.second;

    process_managed_bridge(topic_name, info.req_r2a);
    process_managed_bridge(topic_name, info.req_a2r);
  }
}

void StandardBridgeManager::check_should_exit()
{
  if (!is_parent_alive_ && active_bridges_.empty()) {
    shutdown_requested_ = true;
    if (executor_) {
      executor_->cancel();
    }
  }
}

bool StandardBridgeManager::update_ros2_subscriber_num(const std::string & topic_name)
{
  size_t ros2_count = container_node_->count_subscribers(topic_name);

  struct ioctl_set_ros2_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.ros2_subscriber_num = static_cast<uint32_t>(ros2_count);

  if (ioctl(agnocast_fd, AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger_, "AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    return false;
  }
  return true;
}

void StandardBridgeManager::remove_active_bridge(const std::string & topic_name_with_direction)
{
  if (topic_name_with_direction.size() <= SUFFIX_LEN) {
    return;
  }

  if (active_bridges_.count(topic_name_with_direction) == 0) {
    return;
  }

  std::string_view key_view(topic_name_with_direction);
  std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
  std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

  bool is_r2a = (suffix == SUFFIX_R2A);

  struct ioctl_remove_bridge_args remove_bridge_args
  {
  };
  remove_bridge_args.pid = getpid();
  remove_bridge_args.topic_name = {topic_name_view.data(), topic_name_view.size()};
  remove_bridge_args.is_r2a = is_r2a;

  if (ioctl(agnocast_fd, AGNOCAST_REMOVE_BRIDGE_CMD, &remove_bridge_args) != 0) {
    RCLCPP_ERROR(
      logger_, "AGNOCAST_REMOVE_BRIDGE_CMD failed for topic '%s': %s",
      std::string(topic_name_view).c_str(), strerror(errno));
  }

  active_bridges_.erase(topic_name_with_direction);

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
