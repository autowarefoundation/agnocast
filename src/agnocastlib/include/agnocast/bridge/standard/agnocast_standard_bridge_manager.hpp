#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace agnocast
{

class StandardBridgeManager
{
public:
  explicit StandardBridgeManager(pid_t target_pid);
  ~StandardBridgeManager();

  StandardBridgeManager(const StandardBridgeManager &) = delete;
  StandardBridgeManager & operator=(const StandardBridgeManager &) = delete;

  void run();

private:
  enum class AddBridgeResult { SUCCESS, EXIST, ERROR };

  struct BridgeKernelResult
  {
    AddBridgeResult status;
    pid_t owner_pid;
    bool has_r2a;
    bool has_a2r;
  };

  struct ManagedPubsubBridgeEntry
  {
    BridgeFactorySpec factory_spec;
    // Use -1 when not requested. Valid topic_local_id_t values are [0..MAX_TOPIC_LOCAL_ID).
    topic_local_id_t target_id_r2a;
    topic_local_id_t target_id_a2r;
    bool is_requested_r2a;
    bool is_requested_a2r;

    void reset_r2a()
    {
      target_id_r2a = -1;
      is_requested_r2a = false;
    }
    void reset_a2r()
    {
      target_id_a2r = -1;
      is_requested_a2r = false;
    }
  };

  struct DirectedPubsubBridgeRef
  {
    const std::string & topic_name;
    const ManagedPubsubBridgeEntry & entry;
    BridgeDirection direction;
  };

  struct R2AServiceBridgeItem
  {
    std::shared_ptr<ServiceBridgeBase> bridge;
    std::shared_ptr<rcl_node_t> shadow_node;

    R2AServiceBridgeItem(
      std::shared_ptr<ServiceBridgeBase> bridge, std::shared_ptr<rcl_node_t> shadow_node)
    : bridge(std::move(bridge)), shadow_node(std::move(shadow_node))
    {
    }
  };

  const pid_t target_pid_;
  rclcpp::Logger logger_;

  StandardBridgeIpcEventLoop event_loop_;
  std::unique_ptr<StandardBridgeLoader> loader_;

  bool is_parent_alive_ = true;
  std::atomic_bool shutdown_requested_ = false;

  rclcpp::Node::SharedPtr container_node_;
  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  std::map<std::string, std::shared_ptr<PubsubBridgeBase>> active_pubsub_bridges_;
  std::map<std::string, ManagedPubsubBridgeEntry> managed_pubsub_bridges_;

  std::map<std::string, R2AServiceBridgeItem> active_r2a_service_bridges_;

  void start_ros_execution();

  void on_mq_request(mqd_t fd);
  void on_signal();

  void register_pubsub_request(const MqMsgBridge & req);

  static BridgeKernelResult try_add_pubsub_bridge_to_kernel(
    const std::string & topic_name, bool is_r2a);
  void rollback_pubsub_bridge_from_kernel(const std::string & topic_name, bool is_r2a);
  bool activate_pubsub_bridge(const DirectedPubsubBridgeRef bridge_ref);
  void send_pubsub_delegation(const DirectedPubsubBridgeRef bridge_ref, pid_t owner_pid);
  void process_managed_pubsub_bridge(const DirectedPubsubBridgeRef bridge_ref);
  bool should_remove_pubsub_bridge(const std::string & topic_name, bool is_r2a);

  void create_service_bridge_if_needed(const MqMsgBridge & req);

  void check_parent_alive();
  void check_active_pubsub_bridges();
  void check_and_remove_service_bridges();
  void check_managed_pubsub_bridges();
  void check_should_exit();
};

}  // namespace agnocast
