#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace agnocast
{

class PerformanceBridgeManager
{
public:
  PerformanceBridgeManager();
  ~PerformanceBridgeManager();

  void run();

private:
  using RequestMap = std::unordered_map<topic_local_id_t, BridgeMsgPubSubPayload>;

  // A cross-NS bridge request from the per-NS daemon. Unlike an intra-NS request,
  // there is no local endpoint to resolve the plugin / QoS from, so the type and
  // QoS travel in the request itself. While unexpired, the bridge is created
  // without a same-graph DDS counterpart (the peer NS's bridge creates it) and is
  // shielded from demand-based teardown.
  struct DaemonForcedRequest
  {
    std::string message_type;
    rclcpp::QoS qos;
    std::chrono::steady_clock::time_point forced_until;
  };

  struct R2AServiceBridgeItem
  {
    PerformanceServiceBridgeResult result;
    std::shared_ptr<rcl_node_t> shadow_node;

    R2AServiceBridgeItem(
      PerformanceServiceBridgeResult && result, std::shared_ptr<rcl_node_t> && shadow_node)
    : result(std::move(result)), shadow_node(std::move(shadow_node))
    {
    }
  };

  rclcpp::Logger logger_;
  uint64_t self_ipc_ns_inode_;
  PerformanceBridgeIpcEventLoop event_loop_;
  PerformanceBridgeLoader loader_;

  std::shared_ptr<rclcpp::Node> container_node_;
  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  std::atomic_bool shutdown_requested_ = false;

  std::unordered_map<std::string, PerformancePubsubBridgeResult> active_pubsub_r2a_bridges_;
  std::unordered_map<std::string, PerformancePubsubBridgeResult> active_pubsub_a2r_bridges_;
  std::unordered_map<std::string, RequestMap> request_cache_;

  // Daemon-forced cross-NS requests, keyed by topic, split by direction.
  std::unordered_map<std::string, DaemonForcedRequest> daemon_forced_r2a_;
  std::unordered_map<std::string, DaemonForcedRequest> daemon_forced_a2r_;

  std::unordered_map<std::string, R2AServiceBridgeItem> active_r2a_service_bridges_;

  void start_ros_execution();

  void on_bridge_message(const void * data, std::size_t size);
  void on_signal();
  std::string on_socket_request() const;

  void register_daemon_pubsub_request(const BridgeMsgDaemonPubSubPayload & req);
  bool is_daemon_forced(const std::string & topic_name, BridgeDirection direction) const;
  void create_daemon_forced_bridges();
  void activate_daemon_forced_bridge(
    const std::string & topic_name, const std::string & message_type, const rclcpp::QoS & qos,
    bool is_r2a);

  void check_and_create_pubsub_bridges();
  void check_and_remove_pubsub_bridges();
  void check_and_remove_service_bridges();
  void check_and_remove_request_cache();
  void check_and_request_shutdown();

  bool should_create_pubsub_bridge(const std::string & topic_name, BridgeDirection direction) const;
  void create_pubsub_bridge_if_needed(
    const std::string & topic_name, RequestMap & requests, const std::string & message_type,
    BridgeDirection direction);
  static void remove_invalid_requests(const std::string & topic_name, RequestMap & request_map);

  void create_service_bridge_if_needed(
    const BridgeMsgServicePayload & target, BridgeDirection direction);
};

}  // namespace agnocast
