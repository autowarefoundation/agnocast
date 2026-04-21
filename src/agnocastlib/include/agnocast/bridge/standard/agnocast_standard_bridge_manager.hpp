#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <memory>
#include <optional>

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

  struct BridgeInfo
  {
    // If set to std::nullopt, the factory functions reside in the main executable.
    std::optional<std::string> shared_lib_path;
    uintptr_t fn_offset_r2a;
    uintptr_t fn_offset_a2r;
    topic_local_id_t target_id_r2a;
    topic_local_id_t target_id_a2r;
    bool is_requested_r2a;
    bool is_requested_a2r;

    void reset_r2a()
    {
      target_id_r2a = 0;
      is_requested_r2a = false;
    }
    void reset_a2r()
    {
      target_id_a2r = 0;
      is_requested_a2r = false;
    }
  };

  const pid_t target_pid_;
  rclcpp::Logger logger_;

  StandardBridgeIpcEventLoop event_loop_;
  StandardBridgeLoader loader_;

  bool is_parent_alive_ = true;
  std::atomic_bool shutdown_requested_ = false;

  rclcpp::Node::SharedPtr container_node_;
  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  std::map<std::string, std::shared_ptr<BridgeBase>> active_bridges_;
  std::map<std::string, BridgeInfo> managed_bridges_;

  void start_ros_execution();

  void on_mq_request(mqd_t fd);
  void on_signal();

  void register_request(const MqMsgBridge & req);

  static BridgeKernelResult try_add_bridge_to_kernel(const std::string & topic_name, bool is_r2a);
  void rollback_bridge_from_kernel(const std::string & topic_name, bool is_r2a);
  bool activate_bridge(
    const std::string & topic_name, const BridgeInfo & info, BridgeDirection direction);
  void send_delegation(
    const std::string & topic_name, const BridgeInfo & info, BridgeDirection direction,
    pid_t owner_pid);
  void process_managed_bridge(
    const std::string & topic_name, const BridgeInfo & info, BridgeDirection direction);
  bool should_remove_bridge(const std::string & topic_name, bool is_r2a);

  void check_parent_alive();
  void check_active_bridges();
  void check_managed_bridges();
  void check_should_exit();

  static std::pair<std::string, std::string> extract_topic_info(const MqMsgBridge & req);
};

}  // namespace agnocast
