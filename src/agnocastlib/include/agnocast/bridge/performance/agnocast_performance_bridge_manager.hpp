#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <thread>

namespace agnocast
{

class PerformanceBridgeManager
{
public:
  PerformanceBridgeManager();
  ~PerformanceBridgeManager();

  void run();

private:
  using RequestMap = std::unordered_map<topic_local_id_t, MqMsgPerformanceBridge>;

  rclcpp::Logger logger_;
  PerformanceBridgeIpcEventLoop event_loop_;
  PerformanceBridgeLoader loader_;

  std::shared_ptr<rclcpp::Node> container_node_;
  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  std::atomic_bool shutdown_requested_ = false;

  std::unordered_map<std::string, PerformancePubsubBridgeResult> active_pubsub_r2a_bridges_;
  std::unordered_map<std::string, PerformancePubsubBridgeResult> active_pubsub_a2r_bridges_;
  std::unordered_map<std::string, PerformanceServiceBridgeResult> active_r2a_service_bridges_;
  std::unordered_map<std::string, RequestMap> request_cache_;

  void start_ros_execution();

  void on_mq_request(int fd);
  void on_signal();

  void check_and_create_pubsub_bridges();
  void check_and_remove_pubsub_bridges();
  void check_and_remove_service_bridges();
  void check_and_remove_request_cache();
  void check_and_request_shutdown();

  bool should_create_pubsub_bridge(const std::string & topic_name, BridgeDirection direction) const;
  void create_pubsub_bridge_if_needed(
    const std::string & topic_name, RequestMap & requests, const std::string & message_type,
    BridgeDirection direction);
  void create_service_bridge_if_needed(
    const ServiceBridgeTargetInfo & target, BridgeDirection direction);
  static void remove_invalid_requests(const std::string & topic_name, RequestMap & request_map);
};

}  // namespace agnocast
