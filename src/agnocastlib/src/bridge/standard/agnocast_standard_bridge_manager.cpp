#include "agnocast/bridge/standard/agnocast_standard_bridge_manager.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

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

  active_pubsub_bridges_.clear();
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
    check_managed_pubsub_bridges();
    check_active_pubsub_bridges();
    check_and_remove_service_bridges();
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
    if (req.is_service) {
      create_service_bridge_if_needed(req);
    } else {
      register_pubsub_request(req);
    }
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

void StandardBridgeManager::register_pubsub_request(const MqMsgBridge & req)
{
  // Locally, unique keys include the direction. However, we register the raw topic name (without
  // direction) to the kernel to enforce single-process ownership for the entire topic.

  const std::string topic_name = static_cast<const char *>(req.pubsub_target.topic_name);
  std::string_view suffix =
    (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) ? SUFFIX_PUBSUB_R2A : SUFFIX_PUBSUB_A2R;
  const std::string topic_name_with_direction = topic_name + std::string(suffix);

  if (active_pubsub_bridges_.count(topic_name_with_direction) != 0U) {
    return;
  }

  auto it = managed_pubsub_bridges_.find(topic_name);
  if (it == managed_pubsub_bridges_.end()) {
    if (*static_cast<const char *>(req.factory.shared_lib_path) == '\0') {
      RCLCPP_WARN(
        logger_,
        "Skipping %s bridge request for new topic '%s' due to missing factory information. "
        "This occurs when delegating a request to a bridge manager that has already removed "
        "the topic from its managed bridges.",
        req.direction == BridgeDirection::ROS2_TO_AGNOCAST ? "R2A" : "A2R", topic_name.c_str());
      return;
    }

    auto & entry = managed_pubsub_bridges_[topic_name];

    if (req.factory.in_main_executable) {
      entry.factory_spec.shared_lib_path = std::nullopt;
    } else {
      entry.factory_spec.shared_lib_path =
        std::string(static_cast<const char *>(req.factory.shared_lib_path));
    }
    entry.factory_spec.fn_offset_r2a = req.factory.fn_offset_r2a;
    entry.factory_spec.fn_offset_a2r = req.factory.fn_offset_a2r;

    if (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
      entry.target_id_r2a = req.pubsub_target.target_id;
      entry.is_requested_r2a = true;
      entry.reset_a2r();
    } else {
      entry.target_id_a2r = req.pubsub_target.target_id;
      entry.is_requested_a2r = true;
      entry.reset_r2a();
    }
  } else {
    auto & entry = it->second;

    if (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
      entry.target_id_r2a = req.pubsub_target.target_id;
      entry.is_requested_r2a = true;
    } else {
      entry.target_id_a2r = req.pubsub_target.target_id;
      entry.is_requested_a2r = true;
    }
  }
}

StandardBridgeManager::BridgeKernelResult StandardBridgeManager::try_add_pubsub_bridge_to_kernel(
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

void StandardBridgeManager::rollback_pubsub_bridge_from_kernel(
  const std::string & topic_name, bool is_r2a)
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

bool StandardBridgeManager::activate_pubsub_bridge(const DirectedPubsubBridgeRef bridge_ref)
{
  const auto & [topic_name, entry, direction] = bridge_ref;

  bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);
  std::string_view suffix = is_r2a ? SUFFIX_PUBSUB_R2A : SUFFIX_PUBSUB_A2R;
  std::string topic_name_with_direction = topic_name + std::string(suffix);

  if (active_pubsub_bridges_.count(topic_name_with_direction) != 0U) {
    return true;
  }

  try {
    rclcpp::QoS target_qos = is_r2a ? get_subscriber_qos(topic_name, entry.target_id_r2a)
                                    : get_publisher_qos(topic_name, entry.target_id_a2r);

    auto bridge =
      loader_->start_pubsub_bridge(topic_name, direction, entry.factory_spec, target_qos);

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
    active_pubsub_bridges_[topic_name_with_direction] = bridge;

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to activate bridge for topic '%s': %s", topic_name_with_direction.c_str(),
      e.what());
    return false;
  }
}

void StandardBridgeManager::send_pubsub_delegation(
  const DirectedPubsubBridgeRef bridge_ref, pid_t owner_pid)
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
  const topic_local_id_t target_id =
    (direction == BridgeDirection::ROS2_TO_AGNOCAST) ? entry.target_id_r2a : entry.target_id_a2r;

  auto [req, reason] = BridgeRequestMsgBuilder(BridgeRequestMsgBuilder::Mode::Standard, logger_)
                         .set_direction(direction)
                         .set_is_service(false)
                         .set_topic_name(topic_name.c_str())
                         .set_pubsub_target_id(target_id)
                         .build_standard_message();
  // req.factory can be left zeroed because it is not going to be used.

  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger_, "Failed to build delegation request for topic '%s': %s", topic_name.c_str(),
      reason.c_str());
    if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
      RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
    }
    shutdown_requested_ = true;
    return;
  }
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

void StandardBridgeManager::process_managed_pubsub_bridge(const DirectedPubsubBridgeRef bridge_ref)
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
    try_add_pubsub_bridge_to_kernel(topic_name, is_r2a);
  bool is_active_in_owner = is_r2a ? kernel_has_r2a : kernel_has_a2r;

  switch (status) {
    case AddBridgeResult::SUCCESS:
      if (!activate_pubsub_bridge(bridge_ref)) {
        // Rollback: remove bridge from kernel if activation failed
        rollback_pubsub_bridge_from_kernel(topic_name, is_r2a);
      }
      break;

    case AddBridgeResult::EXIST:
      if (!is_active_in_owner) {
        send_pubsub_delegation(bridge_ref, owner_pid);
      }
      break;

    case AddBridgeResult::ERROR:
      RCLCPP_ERROR(logger_, "Failed to add bridge for '%s'", topic_name.c_str());
      break;
  }
}

bool StandardBridgeManager::should_remove_pubsub_bridge(const std::string & topic_name, bool is_r2a)
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

void StandardBridgeManager::create_service_bridge_if_needed(const MqMsgBridge & req)
{
  if (req.direction != BridgeDirection::ROS2_TO_AGNOCAST) {
    // A2R service bridge is not implemented yet.
    return;
  }

  const std::string service_name = static_cast<const char *>(req.srv_target.service_name);
  const std::string shadow_node_namespace =
    static_cast<const char *>(req.srv_target.shadow_node_namespace);
  const std::string shadow_node_name = static_cast<const char *>(req.srv_target.shadow_node_name);
  if (active_r2a_service_bridges_.count(service_name) != 0U) {
    return;
  }

  // Build the bridge factory spec.
  BridgeFactorySpec factory_spec;
  if (req.factory.in_main_executable) {
    factory_spec.shared_lib_path = std::nullopt;
  } else {
    factory_spec.shared_lib_path =
      std::string(static_cast<const char *>(req.factory.shared_lib_path));
  }
  factory_spec.fn_offset_r2a = req.factory.fn_offset_r2a;
  factory_spec.fn_offset_a2r = req.factory.fn_offset_a2r;

  try {
    // Check that the target service does not already exist in ROS 2.
    const auto services = container_node_->get_service_names_and_types();
    bool exists = std::any_of(services.begin(), services.end(), [&service_name](const auto & s) {
      return s.first == service_name;
    });
    if (exists) {
      RCLCPP_WARN(
        logger_,
        "Found a ROS 2 service with the same name while creating the R2A service bridge: '%s'",
        service_name.c_str());
    }

    const rclcpp::QoS service_qos = get_service_qos(service_name);

    std::shared_ptr<rcl_node_t> shadow_node;
    if (req.srv_target.create_shadow_node && !shadow_node_name.empty()) {
      shadow_node = find_or_create_shadow_node(
        active_r2a_service_bridges_, shadow_node_namespace, shadow_node_name);
    }

    auto bridge = loader_->start_service_bridge(
      service_name, BridgeDirection::ROS2_TO_AGNOCAST, factory_spec, service_qos);

    if (!bridge) {
      RCLCPP_ERROR(logger_, "Bridge loader failed for '%s'", service_name.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      shutdown_requested_ = true;
      return;
    }

    active_r2a_service_bridges_.emplace(
      service_name, R2AServiceBridgeItem(std::move(bridge), std::move(shadow_node)));
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to create service bridge for '%s': %s", service_name.c_str(), e.what());
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown error creating service bridge for '%s'", service_name.c_str());
  }
}

void StandardBridgeManager::check_parent_alive()
{
  if (!is_parent_alive_) {
    return;
  }
  if (kill(target_pid_, 0) != 0) {
    is_parent_alive_ = false;
    managed_pubsub_bridges_.clear();
  }
}

void StandardBridgeManager::check_active_pubsub_bridges()
{
  for (auto it = active_pubsub_bridges_.begin(); it != active_pubsub_bridges_.end();) {
    const std::string & key = it->first;
    const std::shared_ptr<PubsubBridgeBase> & bridge = it->second;
    if (key.size() <= SUFFIX_LEN) {
      ++it;
      continue;
    }

    std::string_view key_view = key;
    std::string_view suffix = key_view.substr(key_view.size() - SUFFIX_LEN);
    std::string_view topic_name_view = key_view.substr(0, key_view.size() - SUFFIX_LEN);

    bool is_r2a = (suffix == SUFFIX_PUBSUB_R2A);
    std::string topic_name_str(topic_name_view);

    if (!should_remove_pubsub_bridge(topic_name_str, is_r2a)) {
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
    it = active_pubsub_bridges_.erase(it);
  }
}

void StandardBridgeManager::check_and_remove_service_bridges()
{
  for (auto it = active_r2a_service_bridges_.begin(); it != active_r2a_service_bridges_.end();) {
    const std::string & service_name = it->first;

    std::string reason;
    if (is_agnocast_service_alive(service_name, reason)) {
      ++it;
      continue;
    }

    RCLCPP_WARN(
      logger_, "Removing R2A service bridge for '%s': %s", service_name.c_str(), reason.c_str());

    auto [ros_cb, agno_cb] = it->second.bridge->get_callback_groups();
    if (ros_cb) {
      executor_->stop_callback_group(ros_cb);
    }
    if (agno_cb) {
      executor_->stop_callback_group(agno_cb);
    }
    it = active_r2a_service_bridges_.erase(it);
  }
}

void StandardBridgeManager::check_managed_pubsub_bridges()
{
  for (auto it = managed_pubsub_bridges_.begin(); it != managed_pubsub_bridges_.end();) {
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
      it = managed_pubsub_bridges_.erase(it);
      continue;
    }

    process_managed_pubsub_bridge(
      DirectedPubsubBridgeRef{topic_name, entry, BridgeDirection::ROS2_TO_AGNOCAST});
    process_managed_pubsub_bridge(
      DirectedPubsubBridgeRef{topic_name, entry, BridgeDirection::AGNOCAST_TO_ROS2});
    ++it;
  }
}

void StandardBridgeManager::check_should_exit()
{
  if (!is_parent_alive_ && active_pubsub_bridges_.empty() && active_r2a_service_bridges_.empty()) {
    if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
      RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
    }
    shutdown_requested_ = true;
    if (executor_) {
      executor_->cancel();
    }
  }
}

}  // namespace agnocast
