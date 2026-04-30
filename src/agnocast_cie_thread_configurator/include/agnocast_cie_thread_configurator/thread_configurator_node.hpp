#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"
#include "agnocast_cie_config_msgs/msg/non_ros_thread_info.hpp"

#include <atomic>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  // Each ThreadConfig instance is reachable from exactly one MutuallyExclusive
  // callback group, so no two callbacks ever touch the same instance. Hence
  // thread_id and applied stay non-atomic; print_all_unapplied reads post-spin.
  struct ThreadConfig
  {
    std::string thread_str;  // callback_group_id or thread_name
    size_t domain_id = 0;
    int64_t thread_id = -1;
    std::vector<int> affinity;
    std::string policy;
    int priority = 0;

    // For SCHED_DEADLINE
    unsigned int runtime = 0;
    unsigned int period = 0;
    unsigned int deadline = 0;

    bool applied = false;
  };

public:
  explicit ThreadConfiguratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ThreadConfiguratorNode();
  void print_all_unapplied();

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void validate_hardware_info(const YAML::Node & yaml);
  void validate_rt_throttling(const YAML::Node & yaml);
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int> & cpus);
  bool issue_syscalls(const ThreadConfig & config);
  void callback_group_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(
    const agnocast_cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr cbg_non_ros_thread_;

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  rclcpp::Subscription<agnocast_cie_config_msgs::msg::NonRosThreadInfo>::SharedPtr
    non_ros_thread_sub_;

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
