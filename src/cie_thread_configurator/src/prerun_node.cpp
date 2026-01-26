#include "cie_thread_configurator/prerun_node.hpp"

#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

PrerunNode::PrerunNode(const std::set<size_t> & domain_ids) : Node("prerun_node")
{
  size_t default_domain_id = cie_thread_configurator::get_default_domain_id();

  // Create subscription for non-ROS thread info
  non_ros_thread_sub_ = this->create_subscription<cie_config_msgs::msg::NonRosThreadInfo>(
    "/cie_thread_configurator/non_ros_thread_info", 100,
    [this](const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg) {
      this->non_ros_thread_callback(msg);
    });

  // Create subscription for default domain on this node
  subs_for_each_domain_.push_back(
    this->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", 100,
      [this, default_domain_id](const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(default_domain_id, msg);
      }));

  // Create nodes and subscriptions for other domain IDs
  for (size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }

    auto node = cie_thread_configurator::create_node_for_domain(domain_id);
    nodes_for_each_domain_.push_back(node);

    auto sub = node->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", 100,
      [this, domain_id](const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(domain_id, msg);
      });
    subs_for_each_domain_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }
}

void PrerunNode::topic_callback(
  size_t domain_id, const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg)
{
  auto key = std::make_pair(domain_id, msg->callback_group_id);
  if (domain_and_cbg_ids_.find(key) != domain_and_cbg_ids_.end()) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received CallbackGroupInfo: domain=%zu | tid=%ld | %s", domain_id,
    msg->thread_id, msg->callback_group_id.c_str());

  domain_and_cbg_ids_.insert(key);
}

void PrerunNode::non_ros_thread_callback(
  const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg)
{
  if (non_ros_thread_names_.find(msg->thread_name) != non_ros_thread_names_.end()) {
    RCLCPP_ERROR(
      this->get_logger(), "Duplicate thread_name received: tid=%ld | %s", msg->thread_id,
      msg->thread_name.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received NonRosThreadInfo: tid=%ld | %s", msg->thread_id,
    msg->thread_name.c_str());

  non_ros_thread_names_.insert(msg->thread_name);
}

const std::vector<rclcpp::Node::SharedPtr> & PrerunNode::get_domain_nodes() const
{
  return nodes_for_each_domain_;
}

void PrerunNode::dump_yaml_config(std::filesystem::path path)
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  // Add hardware information section
  out << YAML::Key << "hardware_info";
  out << YAML::Value << YAML::BeginMap;

  auto hw_info = cie_thread_configurator::get_hardware_info();

  for (const auto & [key, value] : hw_info) {
    out << YAML::Key << key << YAML::Value << value;
  }

  out << YAML::EndMap;

  // Add callback_groups section
  out << YAML::Key << "callback_groups";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & [domain_id, callback_group_id] : domain_and_cbg_ids_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << callback_group_id;
    out << YAML::Key << "domain_id" << YAML::Value << domain_id;
    out << YAML::Key << "affinity" << YAML::Value << YAML::Null;
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "priority" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;

  // Add non_ros_threads section
  out << YAML::Key << "non_ros_threads";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & thread_name : non_ros_thread_names_) {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << thread_name;
    out << YAML::Key << "affinity" << YAML::Value << YAML::Null;
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "priority" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(path / "template.yaml");
  fout << out.c_str();
  fout.close();

  std::cout << "template.yaml is created in the current directory" << std::endl;
}
