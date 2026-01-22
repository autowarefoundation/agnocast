#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  struct CallbackGroupConfig
  {
    std::string callback_group_id;
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
  bool issue_syscalls(const CallbackGroupConfig & config);
  void topic_callback(
    size_t domain_id, const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;

  std::vector<CallbackGroupConfig> callback_group_configs_;
  // (domain_id, callback_group_id) -> CallbackGroupConfig*
  std::map<std::pair<size_t, std::string>, CallbackGroupConfig *> id_to_callback_group_config_;
  int unapplied_num_;
  int cgroup_num_;

  std::vector<CallbackGroupConfig *> deadline_configs_;
};
