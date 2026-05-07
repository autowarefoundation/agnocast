#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"
#include "agnocast_cie_thread_configurator/thread_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"
#include "agnocast_cie_config_msgs/srv/reapply_config.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  using ThreadConfig = agnocast_cie_thread_configurator::ThreadConfig;

  // Concurrency:
  // - callback_group_configs_ / id_to_callback_group_config_: written by the
  //   subscription callbacks AND the reapply service handler, all on the same
  //   SingleThreadedExecutor — no mutex needed.
  // - non_ros_thread_configs_ / id_to_non_ros_thread_config_: written by both
  //   the NonRosThreadInfoListener reader thread and the reapply handler;
  //   all access must hold non_ros_state_mutex_.
  // - print_all_unapplied(): called only after stop() + executor return, so
  //   reads need no lock.

public:
  explicit ThreadConfiguratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ThreadConfiguratorNode();
  void stop() noexcept;
  void print_all_unapplied();

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void validate_hardware_info(const YAML::Node & yaml);
  void validate_rt_throttling(const YAML::Node & yaml);
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int> & cpus);
  bool issue_syscalls(const ThreadConfig & config);
  void callback_group_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(agnocast_cie_thread_configurator::NonRosThreadInfo info);

  void on_reapply_config_request(
    const std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Request> request,
    std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Response> response);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoListener>
    non_ros_thread_listener_;
  rclcpp::Service<agnocast_cie_config_msgs::srv::ReapplyConfig>::SharedPtr reapply_service_;

  std::vector<ThreadConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> ThreadConfig*
  std::map<std::pair<size_t, std::string>, ThreadConfig *> id_to_callback_group_config_;

  std::vector<ThreadConfig> non_ros_thread_configs_;
  // thread_name -> ThreadConfig*
  std::map<std::string, ThreadConfig *> id_to_non_ros_thread_config_;

  std::atomic<int> unapplied_num_{0};
  std::atomic<int> cgroup_num_{0};
  std::atomic<bool> configured_at_least_once_{false};

  const std::string config_file_;
  const size_t default_domain_id_;
  std::mutex non_ros_state_mutex_;
};
