
#include "agnocast/bridge/agnocast_bridge_manager.hpp"

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "agnocast/cie_client_utils.hpp"
#include "agnocast/vendor/rclcpp/generic_service.hpp"
#include "rclcpp/version.h"

#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace agnocast
{

BridgeManager::BridgeManager()
: logger_(rclcpp::get_logger("agnocast_bridge_manager")),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  self_ipc_ns_inode_(get_self_ipc_ns_inode()),
  event_loop_(logger_),
  loader_(std::make_shared<BridgeLoader>(logger_))
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  rclcpp::InitOptions init_options{};
  init_options.shutdown_on_signal = false;
  rclcpp::init(0, nullptr, init_options);

  const char * show_env = std::getenv("AGNOCAST_BRIDGE_SHOW");
  show_enabled_ = (show_env != nullptr && std::string(show_env) == "1");
  const char * model_env = std::getenv("AGNOCAST_BRIDGE_EXEC_MODEL");
  if (model_env != nullptr && std::string(model_env) == "grouped") {
    exec_model_ = ExecModel::Grouped;
  }
  const char * stats_env = std::getenv("AGNOCAST_BRIDGE_SHOW_STATS");
  if (stats_env != nullptr) {
    show_stats_path_ = stats_env;
  }
}

BridgeManager::~BridgeManager()
{
  request_shutdown();

  // Join before any member is destroyed: the worker uses container_node_,
  // executor_ and the pending_msgs_ mutex/CV, which live only until this returns.
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void BridgeManager::run()
{
  constexpr int EVENT_LOOP_TIMEOUT_MS = 1000;

  std::string proc_name = "agno_pbr_" + std::to_string(getpid());
  prctl(PR_SET_NAME, proc_name.c_str(), 0, 0, 0);

  start_ros_execution();
  start_worker_thread();

  event_loop_.set_message_handler(
    [this](const void * data, std::size_t size) { this->parse_and_enqueue(data, size); });
  event_loop_.set_signal_handler([this]() { this->on_signal(); });
  event_loop_.set_socket_handler([this]() { return this->on_socket_request(); });

  while (!shutdown_requested_) {
    if (!event_loop_.spin_once(EVENT_LOOP_TIMEOUT_MS)) {
      RCLCPP_ERROR(logger_, "Event loop spin failed.");
      request_shutdown();
      break;
    }
  }
}

void BridgeManager::worker_loop()
{
  // Also the maintenance polling interval, not just a safety net for the CV: the
  // sweep below must run periodically (bridge teardown, daemon lease expiry) even
  // while no message arrives. Do not drop it in favour of a plain wait().
  constexpr auto WORKER_TIMEOUT = std::chrono::milliseconds(1000);

  while (!shutdown_requested_) {
    std::deque<BridgeMsg> batch;
    {
      std::unique_lock<std::mutex> lk(pending_msgs_mtx_);
      pending_msgs_cv_.wait_for(lk, WORKER_TIMEOUT, [this]() {
        return shutdown_requested_.load(std::memory_order_relaxed) || !pending_msgs_.empty();
      });
      batch.swap(pending_msgs_);
    }

    for (const auto & msg : batch) {
      if (shutdown_requested_.load(std::memory_order_relaxed)) {
        break;
      }
      dispatch_bridge_message(msg);
    }

    // Gate the creating steps and re-check between them: each can dlopen a plugin
    // and build DDS entities, so shutdown may arrive mid-sweep. The reclaiming
    // steps are left ungated; letting them finish keeps the state consistent.
    const auto shutting_down = [this]() {
      return shutdown_requested_.load(std::memory_order_relaxed);
    };

    if (shutting_down()) {
      break;
    }
    check_and_create_pubsub_bridges();

    if (shutting_down()) {
      break;
    }
    create_daemon_forced_bridges();

    check_and_remove_pubsub_bridges();

    if (shutting_down()) {
      break;
    }
    check_and_update_service_bridges();

    if (shutting_down()) {
      break;
    }
    sync_shown_parts();

    check_and_remove_request_cache();
    check_and_request_shutdown();
  }
}

void BridgeManager::start_ros_execution()
{
  const std::string node_name = get_bridge_node_name(self_ipc_ns_inode_);
  container_node_ = std::make_shared<rclcpp::Node>(node_name);

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
      request_shutdown();
      RCLCPP_ERROR(logger_, "Executor Thread CRASHED: %s", e.what());
    }
  });
}

void BridgeManager::start_worker_thread()
{
  worker_thread_ = std::thread([this]() { this->worker_loop(); });
}

void BridgeManager::parse_and_enqueue(const void * data, std::size_t size)
{
  if (shutdown_requested_.load(std::memory_order_relaxed)) {
    return;
  }

  if (size < offsetof(BridgeMsg, payload)) {
    RCLCPP_WARN(
      logger_,
      "bridge msg too small to carry a discriminator: got %zu bytes, expected at least %zu", size,
      offsetof(BridgeMsg, payload));
    return;
  }

  BridgeMsg msg{};
  const size_t copy_size = std::min(size, sizeof(BridgeMsg));
  std::memcpy(&msg, data, copy_size);

  const auto validate_variant_size = [&](size_t expected) -> bool {
    if (size < expected) {
      RCLCPP_WARN(
        logger_, "bridge msg (type=%u) truncated: got %zu bytes, expected at least %zu",
        static_cast<uint32_t>(msg.type), size, expected);
      return false;
    }
    return true;
  };

  switch (msg.type) {
    case BridgeMsgType::Service:
      if (!validate_variant_size(bridge_msg_wire_size<BridgeMsgServicePayload>())) {
        return;
      }
      break;
    case BridgeMsgType::PubSub:
      if (!validate_variant_size(bridge_msg_wire_size<BridgeMsgPubSubPayload>())) {
        return;
      }
      break;
    case BridgeMsgType::DaemonPubSub:
      if (!validate_variant_size(bridge_msg_wire_size<BridgeMsgDaemonPubSubPayload>())) {
        return;
      }
      break;
    case BridgeMsgType::DaemonService:
      if (!validate_variant_size(bridge_msg_wire_size<BridgeMsgDaemonServicePayload>())) {
        return;
      }
      break;
    default:
      RCLCPP_WARN(
        logger_, "Received bridge message with unknown type: %u", static_cast<uint32_t>(msg.type));
      return;
  }

  constexpr size_t PENDING_MSGS_WARN_THRESHOLD = 2000;
  bool exceeded_threshold = false;
  {
    std::lock_guard<std::mutex> lk(pending_msgs_mtx_);
    pending_msgs_.push_back(msg);
    exceeded_threshold = pending_msgs_.size() > PENDING_MSGS_WARN_THRESHOLD;
  }
  pending_msgs_cv_.notify_one();

  if (exceeded_threshold) {
    constexpr int BACKLOG_WARN_INTERVAL_MS = 10000;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, BACKLOG_WARN_INTERVAL_MS,
      "Pending bridge message backlog exceeded %zu; worker thread may not be keeping up.",
      PENDING_MSGS_WARN_THRESHOLD);
  }
}

void BridgeManager::dispatch_bridge_message(const BridgeMsg & msg)
{
  switch (msg.type) {
    case BridgeMsgType::Service: {
      const auto & payload = msg.payload.service;
      std::string service_name = static_cast<const char *>(payload.service_name);
      ServiceBridgeItem sb_item;

      auto it = active_service_bridges_.find(service_name);
      if (it != active_service_bridges_.end()) {
        sb_item = std::move(it->second);
        active_service_bridges_.erase(it);
      }

      sb_item.handle_request(payload);
      active_service_bridges_.emplace(service_name, std::move(sb_item));

      if (show_enabled_ && payload.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
        show_service(payload);
      }
      break;
    }
    case BridgeMsgType::PubSub: {
      const auto & payload = msg.payload.pubsub;
      std::string topic_name = static_cast<const char *>(payload.topic_name);
      topic_local_id_t target_id = payload.target_id;
      std::string message_type = static_cast<const char *>(payload.message_type);

      request_cache_[topic_name][target_id] = payload;

      create_pubsub_bridge_if_needed(
        topic_name, request_cache_[topic_name], message_type, payload.direction);
      break;
    }
    case BridgeMsgType::DaemonPubSub: {
      register_daemon_pubsub_request(msg.payload.daemon_pubsub);
      break;
    }
    case BridgeMsgType::DaemonService: {
      register_daemon_service_request(msg.payload.daemon_service);
      break;
    }
    default: {
      // parse_and_enqueue() must reject every unknown type before enqueuing,
      // so reaching here means the two switches drifted out of sync.
      RCLCPP_ERROR(
        logger_,
        "Agnocast internal implementation error: dispatch_bridge_message got unknown type %u",
        static_cast<uint32_t>(msg.type));
      break;
    }
  }
}

void BridgeManager::register_daemon_pubsub_request(const BridgeMsgDaemonPubSubPayload & req)
{
  const std::string topic_name = static_cast<const char *>(req.topic_name);
  const std::string message_type = static_cast<const char *>(req.type_name);

  const auto forced_until = daemon_force_deadline(std::chrono::steady_clock::now());
  auto & forced =
    (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) ? daemon_forced_r2a_ : daemon_forced_a2r_;
  forced.insert_or_assign(
    topic_name, DaemonForcedRequest{message_type, daemon_request_qos(req), forced_until});
}

void BridgeManager::register_daemon_service_request(const BridgeMsgDaemonServicePayload & req)
{
  const std::string service_name = static_cast<const char *>(req.service_name);

  auto it = active_service_bridges_.find(service_name);
  if (it == active_service_bridges_.end()) {
    return;
  }
  it->second.handle_daemon_request();
}

bool BridgeManager::is_daemon_forced(
  const std::string & topic_name, BridgeDirection direction) const
{
  const auto & forced =
    (direction == BridgeDirection::ROS2_TO_AGNOCAST) ? daemon_forced_r2a_ : daemon_forced_a2r_;
  const auto it = forced.find(topic_name);
  return it != forced.end() &&
         is_daemon_force_active(it->second.forced_until, std::chrono::steady_clock::now());
}

void BridgeManager::create_daemon_forced_bridges()
{
  for (const bool is_r2a : {true, false}) {
    auto & forced = is_r2a ? daemon_forced_r2a_ : daemon_forced_a2r_;
    for (auto it = forced.begin(); it != forced.end();) {
      const std::string & topic_name = it->first;

      // Lease expired: drop the force and let the normal on-demand lifecycle resume.
      if (std::chrono::steady_clock::now() >= it->second.forced_until) {
        it = forced.erase(it);
        continue;
      }

      const bool already_active = is_r2a ? active_pubsub_r2a_bridges_.count(topic_name) > 0
                                         : active_pubsub_a2r_bridges_.count(topic_name) > 0;
      // The same-graph DDS counterpart check is skipped while forced, but a live
      // local Agnocast endpoint is still required for the bridge to carry traffic.
      const auto count = is_r2a ? get_agnocast_subscriber_count(topic_name).count
                                : get_agnocast_publisher_count(topic_name).count;
      if (!already_active && count > 0) {
        activate_daemon_forced_bridge(topic_name, it->second.message_type, it->second.qos, is_r2a);
      }
      ++it;
    }
  }
}

void BridgeManager::activate_daemon_forced_bridge(
  const std::string & topic_name, const std::string & message_type, const rclcpp::QoS & qos,
  bool is_r2a)
{
  try {
    PubsubBridgeResult result =
      is_r2a ? loader_->create_r2a_pubsub_bridge(container_node_, topic_name, message_type, qos)
             : loader_->create_a2r_pubsub_bridge(container_node_, topic_name, message_type, qos);
    if (!result.entity_handle) {
      return;
    }
    set_bridge_direction(result.callback_group, is_r2a);

    if (is_r2a) {
      if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
      }
      if (result.callback_group) {
        executor_->add_callback_group(
          result.callback_group, container_node_->get_node_base_interface());
      }
      active_pubsub_r2a_bridges_[topic_name] = result;
    } else {
      if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
      }
      if (result.callback_group) {
        executor_->add_callback_group(
          result.callback_group, container_node_->get_node_base_interface());
      }
      active_pubsub_a2r_bridges_[topic_name] = result;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to create daemon-forced bridge for '%s': %s", topic_name.c_str(), e.what());
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Unknown error creating daemon-forced bridge for '%s'", topic_name.c_str());
  }
}

void BridgeManager::request_shutdown()
{
  // Write the flag under the same mutex that the worker's cv predicate reads.
  // Without this pairing, a shutdown could be missed if the worker evaluates
  // the predicate just before this store and then sleeps.
  {
    std::lock_guard<std::mutex> lk(pending_msgs_mtx_);
    shutdown_requested_ = true;
  }
  pending_msgs_cv_.notify_all();

  if (executor_) {
    executor_->cancel();
  }

  // Wake up the event loop immediately so that spin_once() returns and the run() loop can observe
  // shutdown_requested_.
  event_loop_.wakeup();
}

void BridgeManager::on_signal()
{
  if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
    RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
  }
  request_shutdown();
}

std::string BridgeManager::on_socket_request() const
{
  return R"({"type":"bridge","ipc_ns":)" + std::to_string(self_ipc_ns_inode_) + R"(,"pid":)" +
         std::to_string(getpid()) + "}";
}

void BridgeManager::check_and_create_pubsub_bridges()
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

    create_pubsub_bridge_if_needed(
      topic_name, requests, message_type, BridgeDirection::ROS2_TO_AGNOCAST);
    create_pubsub_bridge_if_needed(
      topic_name, requests, message_type, BridgeDirection::AGNOCAST_TO_ROS2);

    if (requests.empty()) {
      cache_it = request_cache_.erase(cache_it);
    } else {
      ++cache_it;
    }
  }
}

void BridgeManager::check_and_remove_pubsub_bridges()
{
  auto r2a_it = active_pubsub_r2a_bridges_.begin();
  while (r2a_it != active_pubsub_r2a_bridges_.end()) {
    const std::string & topic_name = r2a_it->first;
    auto result = get_agnocast_subscriber_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_publisher(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get subscriber count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      request_shutdown();
      return;
    }

    // A daemon-forced cross-NS bridge is kept alive without a same-graph DDS
    // counterpart; only a vanished local Agnocast endpoint (count <= 0) removes it.
    const bool keep_forced = is_daemon_forced(topic_name, BridgeDirection::ROS2_TO_AGNOCAST);
    if (result.count <= 0 || (!is_demanded_by_ros2 && !keep_forced)) {
      if (r2a_it->second.callback_group) {
        // Mirror the add at creation. Remove before stop so the monitoring loop cannot re-spawn
        // the group mid-teardown.
        executor_->remove_callback_group(r2a_it->second.callback_group);
        executor_->stop_callback_group(r2a_it->second.callback_group);
      }
      r2a_it = active_pubsub_r2a_bridges_.erase(r2a_it);
    } else {
      if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
      }
      ++r2a_it;
    }
  }

  auto a2r_it = active_pubsub_a2r_bridges_.begin();
  while (a2r_it != active_pubsub_a2r_bridges_.end()) {
    const std::string & topic_name = a2r_it->first;
    auto result = get_agnocast_publisher_count(topic_name);
    bool is_demanded_by_ros2 = has_external_ros2_subscriber(container_node_.get(), topic_name);
    if (result.count == -1) {
      RCLCPP_ERROR(
        logger_, "Failed to get publisher count for topic '%s'. Requesting shutdown.",
        topic_name.c_str());
      if (ioctl(agnocast_fd, AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD) < 0) {
        RCLCPP_ERROR(logger_, "Failed to notify bridge shutdown: %s", strerror(errno));
      }
      request_shutdown();
      return;
    }

    const bool keep_forced = is_daemon_forced(topic_name, BridgeDirection::AGNOCAST_TO_ROS2);
    if (result.count <= 0 || (!is_demanded_by_ros2 && !keep_forced)) {
      if (a2r_it->second.callback_group) {
        // Mirror the add at creation. Remove before stop so the monitoring loop cannot re-spawn
        // the group mid-teardown.
        executor_->remove_callback_group(a2r_it->second.callback_group);
        executor_->stop_callback_group(a2r_it->second.callback_group);
      }
      a2r_it = active_pubsub_a2r_bridges_.erase(a2r_it);
    } else {
      if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
        RCLCPP_ERROR(
          logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
      }
      ++a2r_it;
    }
  }
}

void BridgeManager::check_and_update_service_bridges()
{
  ServiceBridgeDeps deps{container_node_, executor_, logger_, loader_};
#if RCLCPP_VERSION_MAJOR < 28
  deps.ros2_client_service_names =
    collect_external_ros2_client_service_names(container_node_.get());
#endif

  auto it = active_service_bridges_.begin();
  while (it != active_service_bridges_.end()) {
    it->second.check_and_update(deps);

    if (it->second.state() != ServiceBridgeState::NONE) {
      ++it;
      continue;
    }

    it = active_service_bridges_.erase(it);
  }
}

void BridgeManager::check_and_remove_request_cache()
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

void BridgeManager::check_and_request_shutdown()
{
  struct ioctl_check_and_request_bridge_shutdown_args args = {};
  if (ioctl(agnocast_fd, AGNOCAST_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN_CMD, &args) < 0) {
    RCLCPP_ERROR(logger_, "Failed to check bridge shutdown from kernel module.");
    return;
  }

  if (args.ret_should_shutdown) {
    request_shutdown();
  }
}

bool BridgeManager::should_create_pubsub_bridge(
  const std::string & topic_name, BridgeDirection direction) const
{
  if (direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    if (active_pubsub_r2a_bridges_.count(topic_name) > 0) {
      return false;
    }

    const auto stats = get_agnocast_subscriber_count(topic_name);
    if (stats.count <= 0) {
      return false;
    }

    return has_external_ros2_publisher(container_node_.get(), topic_name);
  }
  if (active_pubsub_a2r_bridges_.count(topic_name) > 0) {
    return false;
  }

  const auto stats = get_agnocast_publisher_count(topic_name);
  if (stats.count <= 0) {
    return false;
  }

  return has_external_ros2_subscriber(container_node_.get(), topic_name);
}

void BridgeManager::create_pubsub_bridge_if_needed(
  const std::string & topic_name, RequestMap & requests, const std::string & message_type,
  BridgeDirection direction)
{
  if (!should_create_pubsub_bridge(topic_name, direction)) {
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

    PubsubBridgeResult result;
    if (is_r2a) {
      auto qos = get_subscriber_qos(topic_name, qos_source_id);
      result = loader_->create_r2a_pubsub_bridge(container_node_, topic_name, message_type, qos);
    } else {
      auto qos = get_publisher_qos(topic_name, qos_source_id);
      result = loader_->create_a2r_pubsub_bridge(container_node_, topic_name, message_type, qos);
    }

    if (result.entity_handle) {
      set_bridge_direction(result.callback_group, is_r2a);
      if (is_r2a) {
        if (!update_ros2_publisher_num(container_node_.get(), topic_name)) {
          RCLCPP_ERROR(
            logger_, "Failed to update ROS 2 publisher count for topic '%s'.", topic_name.c_str());
        }
        if (result.callback_group) {
          executor_->add_callback_group(
            result.callback_group, container_node_->get_node_base_interface());
        }
        active_pubsub_r2a_bridges_[topic_name] = result;
      } else {
        if (!update_ros2_subscriber_num(container_node_.get(), topic_name)) {
          RCLCPP_ERROR(
            logger_, "Failed to update ROS 2 subscriber count for topic '%s'.", topic_name.c_str());
        }
        if (result.callback_group) {
          executor_->add_callback_group(
            result.callback_group, container_node_->get_node_base_interface());
        }
        active_pubsub_a2r_bridges_[topic_name] = result;
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

void BridgeManager::remove_invalid_requests(
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
      req_it = request_map.erase(req_it);
    }
  }
}

namespace
{

struct ListedTopic
{
  std::string name;
  uint32_t domain_id;
};

std::vector<ListedTopic> list_agnocast_topics()
{
  std::vector<char> names(static_cast<size_t>(MAX_TOPIC_NUM) * TOPIC_NAME_BUFFER_SIZE);
  std::vector<uint32_t> domains(MAX_TOPIC_NUM);
  union ioctl_topic_list_args args = {};
  args.topic_name_buffer_addr = reinterpret_cast<uint64_t>(names.data());
  args.domain_id_buffer_addr = reinterpret_cast<uint64_t>(domains.data());
  args.topic_name_buffer_size = MAX_TOPIC_NUM;
  std::vector<ListedTopic> topics;
  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_LIST_CMD, &args) < 0) {
    return topics;
  }
  topics.reserve(args.ret_topic_num);
  for (uint32_t i = 0; i < args.ret_topic_num; i++) {
    const char * name = names.data() + static_cast<size_t>(i) * TOPIC_NAME_BUFFER_SIZE;
    topics.push_back({std::string(name, strnlen(name, TOPIC_NAME_BUFFER_SIZE)), domains[i]});
  }
  return topics;
}

// Called only from the worker thread. The buffer is kept and grown on -ENOBUFS, so the usual few
// endpoints per topic do not cost a buffer sized for the per-topic maximum.
std::vector<topic_endpoint_ret> get_topic_endpoints(const std::string & topic_name, uint32_t domain)
{
  constexpr size_t MAX_ENDPOINT_NUM = MAX_PUBLISHER_NUM + MAX_SUBSCRIBER_NUM;
  static std::vector<topic_endpoint_ret> buffer(64);
  for (;;) {
    union ioctl_topic_endpoints_args args = {};
    args.topic_name = {topic_name.c_str(), topic_name.size()};
    args.endpoint_ret_buffer_addr = reinterpret_cast<uint64_t>(buffer.data());
    args.endpoint_ret_buffer_size = static_cast<uint32_t>(buffer.size());
    args.domain_id = domain;
    if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_ENDPOINTS_CMD, &args) == 0) {
      return {buffer.begin(), buffer.begin() + args.ret_endpoint_num};
    }
    if (errno != ENOBUFS || buffer.size() >= MAX_ENDPOINT_NUM) {
      return {};
    }
    buffer.resize(std::min(buffer.size() * 4, MAX_ENDPOINT_NUM));
  }
}

int count_dir_entries(const char * path)
{
  DIR * dir = opendir(path);
  if (dir == nullptr) {
    return -1;
  }
  int num = 0;
  while (const struct dirent * entry = readdir(dir)) {
    if (entry->d_name[0] != '.') {
      num++;
    }
  }
  closedir(dir);
  return num;
}

long read_rss_kb()
{
  std::ifstream statm("/proc/self/statm");
  long size = 0;
  long resident = 0;
  statm >> size >> resident;
  return resident * (sysconf(_SC_PAGESIZE) / 1024);
}

}  // namespace

std::shared_ptr<rclcpp::Node> BridgeManager::find_or_create_shadow_node(
  const std::string & full_node_name)
{
  auto identity = split_full_node_name(full_node_name);
  auto it = shadow_nodes_.find(identity);
  if (it != shadow_nodes_.end()) {
    if (auto node = it->second.lock()) {
      return node;
    }
  }

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_rosout(false);
  options.use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>(identity.second, identity.first, options);
  shadow_nodes_[identity] = node;
  register_shadow_rcl_node(identity, node->get_node_base_interface()->get_shared_rcl_node_handle());
  return node;
}

// PerPart: one group, and so one CIE thread, per part.
// Grouped: the owner's one group, which every owner's group shares a CIE thread with.
rclcpp::CallbackGroup::SharedPtr BridgeManager::create_shown_part_group(
  const std::shared_ptr<rclcpp::Node> & owner)
{
  if (exec_model_ == ExecModel::Grouped) {
    auto & owner_group = owner_groups_[owner.get()];
    if (!owner_group.group) {
      owner_group.group =
        owner->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      set_shared_executor_key(owner_group.group, "agnocast_bridge_dds_default");
    }
    return owner_group.group;
  }

  auto group = owner->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  set_bridge_direction(group, true);
  return group;
}

// Called once the part's entity exists: CIE decides the executor kind from the group's contents.
void BridgeManager::attach_shown_part_group(const ShownPart & part)
{
  if (!part.group) {
    return;
  }
  if (exec_model_ == ExecModel::Grouped) {
    auto & owner_group = owner_groups_[part.owner.get()];
    if (owner_group.part_num++ > 0) {
      return;
    }
  }
  executor_->add_callback_group(part.group, part.owner->get_node_base_interface());
}

void BridgeManager::detach_shown_part_group(const ShownPart & part)
{
  if (!part.group) {
    return;
  }
  if (exec_model_ == ExecModel::Grouped) {
    auto it = owner_groups_.find(part.owner.get());
    if (it == owner_groups_.end() || --it->second.part_num > 0) {
      return;
    }
    executor_->remove_callback_group(part.group);
    executor_->detach_shared_callback_group(part.group);
    owner_groups_.erase(it);
    return;
  }
  executor_->remove_callback_group(part.group);
  executor_->stop_callback_group(part.group);
}

void BridgeManager::show_service(const BridgeMsgServicePayload & payload)
{
  const std::string service_name = static_cast<const char *>(payload.service_name);
  const std::string service_type = static_cast<const char *>(payload.service_type);
  if (shown_services_.count(service_name) > 0) {
    return;
  }

  try {
    ShownPart part;
    if (payload.create_shadow_node) {
      const std::string ns = static_cast<const char *>(payload.shadow_node_namespace);
      const std::string name = static_cast<const char *>(payload.shadow_node_name);
      part.owner = find_or_create_shadow_node((ns == "/" ? "" : ns) + "/" + name);
    } else {
      part.owner = container_node_;
    }
    part.group = create_shown_part_group(part.owner);
    part.server = agnocast::vendor_rclcpp::GenericService::create_generic_service(
      part.owner.get(), service_name, service_type,
      [this](
        const std::shared_ptr<agnocast::vendor_rclcpp::GenericService> &,
        const std::shared_ptr<rmw_request_id_t> &, const std::shared_ptr<void> &) {
        shown_server_request_count_.fetch_add(1, std::memory_order_relaxed);
      },
      rclcpp::ServicesQoS(), part.group);
    attach_shown_part_group(part);
    shown_services_.emplace(service_name, std::move(part));
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to show service '%s': %s", service_name.c_str(), e.what());
  }
}

void BridgeManager::sync_shown_parts()
{
  if (!show_enabled_) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  const uint32_t own_domain = static_cast<uint32_t>(
    container_node_->get_node_base_interface()->get_context()->get_domain_id());

  std::set<ShownKey> seen;
  for (const auto & topic : list_agnocast_topics()) {
    if (topic.domain_id != own_domain || topic.name.rfind("/AGNOCAST_SRV_", 0) == 0) {
      continue;
    }

    for (const auto & ep : get_topic_endpoints(topic.name, topic.domain_id)) {
      const std::string type_name(ep.type_name, strnlen(ep.type_name, TYPE_NAME_BUFFER_SIZE));
      if (ep.is_bridge || type_name.empty()) {
        continue;
      }

      ShownKey key{topic.name, ep.id, ep.is_publisher};
      seen.insert(key);
      if (shown_pubsub_.count(key) > 0 || failed_pubsub_.count(key) > 0) {
        continue;
      }

      const std::string node_name(ep.node_name, strnlen(ep.node_name, NODE_NAME_BUFFER_SIZE));
      try {
        ShownPart part;
        part.owner = ep.is_rclcpp_node ? container_node_ : find_or_create_shadow_node(node_name);
        const size_t depth = std::max<uint32_t>(ep.qos_depth, 1);
        if (ep.is_publisher) {
          rclcpp::QoS qos{rclcpp::KeepLast(depth)};
          qos.reliable();
          if (ep.qos_is_transient_local) {
            qos.transient_local();
          }
          part.writer = part.owner->create_generic_publisher(topic.name, type_name, qos);
        } else {
          rclcpp::QoS qos{rclcpp::KeepLast(depth)};
          if (ep.qos_is_reliable) {
            qos.reliable();
          } else {
            qos.best_effort();
          }
          if (ep.qos_is_transient_local) {
            qos.transient_local();
          }
          part.group = create_shown_part_group(part.owner);
          rclcpp::SubscriptionOptions options;
          options.callback_group = part.group;
          options.ignore_local_publications = true;
          part.reader = part.owner->create_generic_subscription(
            topic.name, type_name, qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage>) {
              shown_reader_msg_count_.fetch_add(1, std::memory_order_relaxed);
            },
            options);
          attach_shown_part_group(part);
        }
        shown_pubsub_.emplace(key, std::move(part));
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger_, "Failed to show %s of '%s' (type '%s', node '%s'): %s",
          ep.is_publisher ? "publisher" : "subscriber", topic.name.c_str(), type_name.c_str(),
          node_name.c_str(), e.what());
        failed_pubsub_.emplace(key, e.what());
      }
    }
  }

  for (auto it = shown_pubsub_.begin(); it != shown_pubsub_.end();) {
    if (seen.count(it->first) > 0) {
      ++it;
      continue;
    }
    detach_shown_part_group(it->second);
    it = shown_pubsub_.erase(it);
  }
  for (auto it = failed_pubsub_.begin(); it != failed_pubsub_.end();) {
    it = (seen.count(it->first) > 0) ? std::next(it) : failed_pubsub_.erase(it);
  }

  for (auto it = shown_services_.begin(); it != shown_services_.end();) {
    std::string reason;
    if (is_agnocast_service_alive(it->first, reason)) {
      ++it;
      continue;
    }
    detach_shown_part_group(it->second);
    it = shown_services_.erase(it);
  }

  report_shown_parts(std::chrono::steady_clock::now() - start);
}

void BridgeManager::report_shown_parts(std::chrono::steady_clock::duration sync_time)
{
  size_t writers = 0;
  size_t readers = 0;
  for (const auto & [key, part] : shown_pubsub_) {
    (part.writer ? writers : readers)++;
  }
  // Threads are logged too: CIE spawns a group's thread on its next monitoring pass, after the
  // part.
  const size_t part_num = shown_pubsub_.size() + shown_services_.size() + failed_pubsub_.size();
  const int thread_num = count_dir_entries("/proc/self/task");
  if (part_num == last_logged_part_num_ && thread_num == last_logged_thread_num_) {
    return;
  }
  last_logged_part_num_ = part_num;
  last_logged_thread_num_ = thread_num;

  size_t shadows = 0;
  for (const auto & [identity, weak_node] : shadow_nodes_) {
    shadows += weak_node.expired() ? 0 : 1;
  }
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
  const auto sync_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sync_time).count();
  char line[512];
  snprintf(
    line, sizeof(line),
    "{\"t_ns\":%lld,\"model\":\"%s\",\"writers\":%zu,\"readers\":%zu,\"servers\":%zu,"
    "\"failed\":%zu,\"shadows\":%zu,\"threads\":%d,\"fds\":%d,\"rss_kb\":%ld,\"sync_ms\":%lld,"
    "\"reader_msgs\":%llu,\"server_reqs\":%llu}",
    static_cast<long long>(now_ns), exec_model_ == ExecModel::PerPart ? "per_part" : "grouped",
    writers, readers, shown_services_.size(), failed_pubsub_.size(), shadows, thread_num,
    count_dir_entries("/proc/self/fd"), read_rss_kb(), static_cast<long long>(sync_ms),
    static_cast<unsigned long long>(shown_reader_msg_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(shown_server_request_count_.load(std::memory_order_relaxed)));
  RCLCPP_INFO(logger_, "show: %s", line);
  if (!show_stats_path_.empty()) {
    std::ofstream out(show_stats_path_, std::ios::app);
    out << line << '\n';
  }
}

}  // namespace agnocast
