#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_config_msgs/msg/callback_group_info.hpp"
#include "cie_config_msgs/msg/non_ros_thread_info.hpp"

#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  struct ThreadConfig
  {
    std::string thread_str;  // callback_group_id or thread_name
    size_t domain_id;
    int64_t thread_id = -1;
    std::vector<int> affinity;
    std::string policy;
    int priority;

    // For SCHED_DEADLINE
    unsigned int runtime;
    unsigned int period;
    unsigned int deadline;

    bool applied = false;
  };

public:
  explicit ThreadConfiguratorNode(const YAML::Node & yaml);
  ~ThreadConfiguratorNode();
  bool all_applied();
  void print_all_unapplied();

  bool exist_deadline_config();
  bool apply_deadline_configs();

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int> & cpus);
  bool issue_syscalls(const ThreadConfig & config);
  void callback_group_callback(
    size_t domain_id, const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  rclcpp::Subscription<cie_config_msgs::msg::NonRosThreadInfo>::SharedPtr non_ros_thread_sub_;

  std::vector<ThreadConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> ThreadConfig*
  std::map<std::pair<size_t, std::string>, ThreadConfig *> id_to_callback_group_config_;

  std::vector<ThreadConfig> non_ros_thread_configs_;
  // thread_name -> ThreadConfig*
  std::map<std::string, ThreadConfig *> id_to_non_ros_thread_config_;

  int unapplied_num_;
  int cgroup_num_;

  std::vector<ThreadConfig *> deadline_configs_;
};
