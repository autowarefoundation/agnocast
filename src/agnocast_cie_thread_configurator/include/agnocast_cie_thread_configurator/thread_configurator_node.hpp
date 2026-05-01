#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"
#include "agnocast_cie_thread_configurator/thread_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  using ThreadConfig = agnocast_cie_thread_configurator::ThreadConfig;

  // Each ThreadConfig instance is touched by exactly one writer:
  // - callback_group_configs_ entries: written from CallbackGroupInfo
  //   subscription callbacks dispatched by the SingleThreadedExecutor
  //   (one subscription on the main node and one per extra per-domain
  //   node, all on the node's default callback group).
  // - non_ros_thread_configs_ entries: written from the
  //   NonRosThreadInfoListener's private reader thread.
  // The two vectors are disjoint, so there is no shared mutable state
  // between the executor thread and the listener thread.
  // print_all_unapplied() reads applied flags only after main.cpp calls
  // node->stop() (which joins the listener) and after executor->spin()
  // returns, so no concurrent access is possible at read time.

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

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  std::unique_ptr<agnocast_cie_thread_configurator::NonRosThreadInfoListener>
    non_ros_thread_listener_;

  std::vector<ThreadConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> ThreadConfig*
  std::map<std::pair<size_t, std::string>, ThreadConfig *> id_to_callback_group_config_;

  std::vector<ThreadConfig> non_ros_thread_configs_;
  // thread_name -> ThreadConfig*
  std::map<std::string, ThreadConfig *> id_to_non_ros_thread_config_;

  std::atomic<int> unapplied_num_{0};
  std::atomic<int> cgroup_num_{0};
  std::atomic<bool> configured_at_least_once_{false};
};
