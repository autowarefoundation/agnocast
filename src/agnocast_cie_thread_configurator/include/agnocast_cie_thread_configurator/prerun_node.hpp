#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"
#include "agnocast_cie_config_msgs/msg/non_ros_thread_info.hpp"

#include <filesystem>
#include <mutex>
#include <set>
#include <string>

class PrerunNode : public rclcpp::Node
{
public:
  explicit PrerunNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void dump_yaml_config(std::filesystem::path path);

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void topic_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(
    const agnocast_cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr cbg_non_ros_thread_;

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  rclcpp::Subscription<agnocast_cie_config_msgs::msg::NonRosThreadInfo>::SharedPtr
    non_ros_thread_sub_;

  // (domain_id, callback_group_id) pairs. Guarded by domain_and_cbg_ids_mutex_
  // during callbacks; dump_yaml_config reads it post-spin without the lock.
  std::set<std::pair<size_t, std::string>> domain_and_cbg_ids_;
  std::mutex domain_and_cbg_ids_mutex_;
  // non-ROS thread names. Single subscription on a MutuallyExclusive group;
  // dump_yaml_config reads it post-spin. No concurrent access, no mutex needed.
  std::set<std::string> non_ros_thread_names_;
};
