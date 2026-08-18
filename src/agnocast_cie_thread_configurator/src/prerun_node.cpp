#include "agnocast_cie_thread_configurator/prerun_node.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "agnocast_cie_thread_configurator/system_scan.hpp"
#include "agnocast_cie_thread_configurator/thread_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{

// One template entry per comm (it manages every thread sharing that comm).
// The scan is sorted by (comm, tid), so the kept observed values are the
// lowest tid's.
std::vector<agnocast_cie_thread_configurator::KernelThreadInfo> dedup_by_comm(
  std::vector<agnocast_cie_thread_configurator::KernelThreadInfo> scanned)
{
  std::vector<agnocast_cie_thread_configurator::KernelThreadInfo> result;
  for (auto & info : scanned) {
    if (result.empty() || result.back().comm != info.comm) {
      result.push_back(std::move(info));
    }
  }
  return result;
}

}  // namespace

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

  auto cbg_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

  non_ros_thread_listener_ =
    std::make_unique<agnocast_cie_thread_configurator::NonRosThreadInfoListener>(
      [this](agnocast_cie_thread_configurator::NonRosThreadInfo info) {
        this->non_ros_thread_callback(std::move(info));
      },
      this->get_logger());

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

void PrerunNode::non_ros_thread_callback(agnocast_cie_thread_configurator::NonRosThreadInfo info)
{
  if (non_ros_thread_names_.find(info.name) != non_ros_thread_names_.end()) {
    RCLCPP_ERROR(
      this->get_logger(), "Duplicate thread_name received: tid=%ld | %s", info.tid,
      info.name.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received NonRosThreadInfo: tid=%ld | %s", info.tid, info.name.c_str());

  non_ros_thread_names_.insert(std::move(info.name));
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

  // An empty list means "do not manage affinity", same as null, but it shows
  // the shape the parser expects and stays a list through YAML round-trips
  // that turn every scalar into a string.
  const auto emit_unmanaged_affinity = [&out]() {
    out << YAML::Key << "affinity" << YAML::Value << YAML::Flow << YAML::BeginSeq << YAML::EndSeq;
  };

  // Add callback_groups section
  out << YAML::Key << "callback_groups";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & [domain_id, callback_group_id] : domain_and_cbg_ids_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << callback_group_id;
    out << YAML::Key << "domain_id" << YAML::Value << domain_id;
    emit_unmanaged_affinity();
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "nice" << YAML::Value << 0;
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
    emit_unmanaged_affinity();
    out << YAML::Key << "policy" << YAML::Value << "SCHED_OTHER";
    out << YAML::Key << "nice" << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;

  // Add kernel_threads and irqs sections: observed values become the initial
  // desired values (compare-before-set keeps unedited entries no-ops), and
  // UNMANAGEABLE marks values that cannot be provided or changed (YAML null
  // stays the user's opt-out).
  const std::string unmanageable(agnocast_cie_thread_configurator::k_unmanageable);
  const auto kernel_threads =
    dedup_by_comm(agnocast_cie_thread_configurator::scan_kernel_threads());
  const auto irqs = agnocast_cie_thread_configurator::scan_irqs();
  RCLCPP_INFO(
    this->get_logger(), "Scanned %zu kernel thread comms and %zu device-backed IRQs",
    kernel_threads.size(), irqs.size());

  out << YAML::Key << "kernel_threads";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & info : kernel_threads) {
    // A policy with no YAML representation: UNKNOWN(<n>), or SCHED_DEADLINE,
    // whose runtime/period/deadline cannot be recovered from /proc.
    const bool policy_representable =
      agnocast_cie_thread_configurator::policy_to_sched_const.count(info.policy) > 0 &&
      info.policy != "SCHED_DEADLINE";
    const auto cpus = agnocast_cie_thread_configurator::parse_cpu_list(info.affinity);

    out << YAML::BeginMap;
    out << YAML::Key << "comm" << YAML::Value << info.comm;
    if (policy_representable) {
      const bool is_cfs =
        info.policy == "SCHED_OTHER" || info.policy == "SCHED_BATCH" || info.policy == "SCHED_IDLE";
      out << YAML::Key << "policy" << YAML::Value << info.policy;
      if (is_cfs) {
        out << YAML::Key << "nice" << YAML::Value << info.nice;
      } else {
        out << YAML::Key << "priority" << YAML::Value << info.rt_priority;
      }
    } else {
      out << YAML::Key << "policy" << YAML::Value << unmanageable;
      out << YAML::Key << "priority" << YAML::Value << unmanageable;
    }
    if (info.no_setaffinity || !cpus) {
      out << YAML::Key << "affinity" << YAML::Value << unmanageable;
    } else {
      out << YAML::Key << "affinity" << YAML::Value << YAML::Flow << *cpus;
    }
    out << YAML::EndMap;
    out << YAML::Newline;
  }

  out << YAML::EndSeq;

  // Add irqs section
  out << YAML::Key << "irqs";
  out << YAML::Value << YAML::BeginSeq;

  for (const auto & info : irqs) {
    const auto cpus = agnocast_cie_thread_configurator::parse_cpu_list(info.affinity);

    out << YAML::BeginMap;
    out << YAML::Key << "irq" << YAML::Value << info.irq;
    out << YAML::Key << "name" << YAML::Value << info.name;
    if (cpus) {
      out << YAML::Key << "affinity" << YAML::Value << YAML::Flow << *cpus;
    } else {
      out << YAML::Key << "affinity" << YAML::Value << unmanageable;
    }
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

PrerunNode::~PrerunNode()
{
  stop();
}

void PrerunNode::stop() noexcept
{
  if (non_ros_thread_listener_) {
    non_ros_thread_listener_->stop();
  }
}
