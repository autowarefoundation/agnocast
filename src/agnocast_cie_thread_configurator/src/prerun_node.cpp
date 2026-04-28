#include "agnocast_cie_thread_configurator/prerun_node.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

PrerunNode::PrerunNode(const rclcpp::NodeOptions & options) : Node("prerun_node", options)
{
  // https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html#choosing-a-domain-id-short-version
  constexpr size_t max_domain_id = 101;

  const auto domains =
    this->declare_parameter<std::vector<int64_t>>("domains", std::vector<int64_t>{});
  std::set<size_t> domain_ids;
  for (const auto raw_domain_id : domains) {
    if (raw_domain_id < 0) {
      RCLCPP_WARN(
        this->get_logger(), "Negative domain ID %lld is invalid. Skipping.",
        static_cast<long long>(raw_domain_id));
      continue;
    }

    const size_t domain_id = static_cast<size_t>(raw_domain_id);
    if (domain_id > max_domain_id) {
      RCLCPP_WARN(
        this->get_logger(), "Domain ID %zu exceeds maximum valid value (%zu). Skipping.", domain_id,
        max_domain_id);
      continue;
    }

    domain_ids.insert(domain_id);
  }

  size_t default_domain_id = agnocast_cie_thread_configurator::get_default_domain_id();

  cbg_non_ros_thread_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto cbg_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
  // volatile: publisher context in spawn_non_ros2_thread is destroyed after publish,
  // so transient_local is ineffective.
  auto non_ros_thread_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();

  // Create subscription for non-ROS thread info
  rclcpp::SubscriptionOptions non_ros_opts;
  non_ros_opts.callback_group = cbg_non_ros_thread_;
  non_ros_thread_sub_ = this->create_subscription<agnocast_cie_config_msgs::msg::NonRosThreadInfo>(
    "/agnocast_cie_thread_configurator/non_ros_thread_info", non_ros_thread_qos,
    [this](const agnocast_cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg) {
      this->non_ros_thread_callback(msg);
    },
    non_ros_opts);

  // Create subscription for default domain on this node. Uses the node's default
  // callback group, mirroring the per-domain extra nodes below.
  subs_for_each_domain_.push_back(
    this->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this,
       default_domain_id](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(default_domain_id, msg);
      }));

  // Create nodes and subscriptions for other domain IDs
  for (size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }

    auto node = agnocast_cie_thread_configurator::create_node_for_domain(domain_id);
    nodes_for_each_domain_.push_back(node);

    auto sub = node->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this, domain_id](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->topic_callback(domain_id, msg);
      });
    subs_for_each_domain_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }
}

void PrerunNode::topic_callback(
  size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg)
{
  auto key = std::make_pair(domain_id, msg->callback_group_id);
  {
    std::lock_guard<std::mutex> lock(domain_and_cbg_ids_mutex_);
    if (domain_and_cbg_ids_.find(key) != domain_and_cbg_ids_.end()) {
      return;
    }
    domain_and_cbg_ids_.insert(key);
  }

  RCLCPP_INFO(
    this->get_logger(), "Received CallbackGroupInfo: domain=%zu | tid=%ld | %s", domain_id,
    msg->thread_id, msg->callback_group_id.c_str());
}

void PrerunNode::non_ros_thread_callback(
  const agnocast_cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg)
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

  auto hw_info = agnocast_cie_thread_configurator::get_hardware_info();

  for (const auto & [key, value] : hw_info) {
    out << YAML::Key << key << YAML::Value << value;
  }

  out << YAML::EndMap;

  // Add rt_throttling section
  out << YAML::Key << "rt_throttling";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "runtime_us" << YAML::Value << 950000;
  out << YAML::Key << "period_us" << YAML::Value << 1000000;
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
