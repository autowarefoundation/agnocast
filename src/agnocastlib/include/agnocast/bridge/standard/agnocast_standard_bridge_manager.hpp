#pragma once

#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"
#include "rclcpp/rclcpp.hpp"

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
    std::optional<MqMsgBridge> req_r2a;
    std::optional<MqMsgBridge> req_a2r;
  };

  const pid_t target_pid_;
  rclcpp::Logger logger_;

  StandardBridgeIpcEventLoop event_loop_;
  StandardBridgeLoader loader_;

  bool is_parent_alive_ = true;
  bool shutdown_requested_ = false;

  rclcpp::Node::SharedPtr container_node_;
  std::shared_ptr<agnocast::MultiThreadedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  std::map<std::string, std::shared_ptr<void>> active_bridges_;
  std::map<std::string, BridgeInfo> managed_bridges_;

  void start_ros_execution();

  void on_mq_request(mqd_t fd);
  void on_signal();

  void register_request(const MqMsgBridge & req);

  static BridgeKernelResult try_add_bridge_to_kernel(const std::string & topic_name, bool is_r2a);
  void activate_bridge(const MqMsgBridge & req, const std::string & topic_name_with_direction);
  void send_delegation(const MqMsgBridge & req, pid_t owner_pid);
  void process_managed_bridge(
    const std::string & topic_name, const std::optional<MqMsgBridge> & req);

  void check_parent_alive();
  void check_active_bridges();
  void check_managed_bridges();
  void check_should_exit();

  void remove_active_bridge(const std::string & topic_name_with_direction);
  bool update_ros2_subscriber_num(const std::string & topic_name);

  static std::pair<std::string, std::string> extract_topic_info(const MqMsgBridge & req);
};

}  // namespace agnocast
