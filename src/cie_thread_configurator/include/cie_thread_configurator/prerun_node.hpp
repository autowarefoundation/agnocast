#pragma once

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_config_msgs/msg/callback_group_info.hpp"
#include "cie_config_msgs/msg/non_ros_thread_info.hpp"

#include <filesystem>
#include <set>
#include <string>

class PrerunNode : public rclcpp::Node
{
public:
  explicit PrerunNode(const std::set<size_t> & domain_ids);
  void dump_yaml_config(std::filesystem::path path);

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  void topic_callback(
    size_t domain_id, const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg);

  std::vector<rclcpp::Node::SharedPtr> nodes_for_each_domain_;
  std::vector<rclcpp::Subscription<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr>
    subs_for_each_domain_;
  rclcpp::Subscription<cie_config_msgs::msg::NonRosThreadInfo>::SharedPtr non_ros_thread_sub_;

  // (domain_id, callback_group_id) pairs
  std::set<std::pair<size_t, std::string>> domain_and_cbg_ids_;
  // non-ROS thread names
  std::set<std::string> non_ros_thread_names_;
};
