#include "cie_thread_configurator/thread_configurator_node.hpp"

#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "cie_thread_configurator/sched_deadline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <error.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

ThreadConfiguratorNode::ThreadConfiguratorNode(const YAML::Node & yaml)
: Node("thread_configurator_node"), unapplied_num_(0), cgroup_num_(0)
{
  validate_rt_throttling(yaml);

  YAML::Node callback_groups = yaml["callback_groups"];
  YAML::Node non_ros_threads = yaml["non_ros_threads"];

  unapplied_num_ = callback_groups.size() + non_ros_threads.size();
  callback_group_configs_.resize(callback_groups.size());
  non_ros_thread_configs_.resize(non_ros_threads.size());

  size_t default_domain_id = cie_thread_configurator::get_default_domain_id();

  // For backward compatibility: remove trailing "Waitable@"s
  auto remove_trailing_waitable = [](std::string s) {
    static constexpr std::string_view suffix = "@Waitable";
    const std::size_t suffix_size = suffix.size();
    std::size_t s_size = s.size();

    while (s_size >= suffix_size &&
           std::char_traits<char>::compare(
             s.data() + (s_size - suffix_size), suffix.data(), suffix_size) == 0) {
      s_size -= suffix_size;
    }
    s.resize(s_size);

    return s;
  };

  std::set<size_t> domain_ids;
  for (size_t i = 0; i < callback_groups.size(); i++) {
    const auto & callback_group = callback_groups[i];
    auto & config = callback_group_configs_[i];

    config.thread_str = remove_trailing_waitable(callback_group["id"].as<std::string>());

    // Get domain_id from config, default to default_domain_id for backward compatibility
    if (callback_group["domain_id"]) {
      config.domain_id = callback_group["domain_id"].as<size_t>();
    } else {
      config.domain_id = default_domain_id;
    }
    domain_ids.insert(config.domain_id);

    for (auto & cpu : callback_group["affinity"]) {
      config.affinity.push_back(cpu.as<int>());
    }
    config.policy = callback_group["policy"].as<std::string>();

    if (config.policy == "SCHED_DEADLINE") {
      config.runtime = callback_group["runtime"].as<unsigned int>();
      config.period = callback_group["period"].as<unsigned int>();
      config.deadline = callback_group["deadline"].as<unsigned int>();
    } else {
      config.priority = callback_group["priority"].as<int>();
    }

    auto key = std::make_pair(config.domain_id, config.thread_str);
    id_to_callback_group_config_[key] = &config;
  }

  // Load non-ROS thread configurations
  for (size_t i = 0; i < non_ros_threads.size(); i++) {
    const auto & non_ros_thread = non_ros_threads[i];
    auto & config = non_ros_thread_configs_[i];

    config.thread_str = non_ros_thread["name"].as<std::string>();

    for (auto & cpu : non_ros_thread["affinity"]) {
      config.affinity.push_back(cpu.as<int>());
    }
    config.policy = non_ros_thread["policy"].as<std::string>();

    if (config.policy == "SCHED_DEADLINE") {
      config.runtime = non_ros_thread["runtime"].as<unsigned int>();
      config.period = non_ros_thread["period"].as<unsigned int>();
      config.deadline = non_ros_thread["deadline"].as<unsigned int>();
    } else {
      config.priority = non_ros_thread["priority"].as<int>();
    }

    id_to_non_ros_thread_config_[config.thread_str] = &config;
  }

  auto qos =
    rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5000)).reliable();

  // Create subscription for non-ROS thread info
  non_ros_thread_sub_ = this->create_subscription<cie_config_msgs::msg::NonRosThreadInfo>(
    "/cie_thread_configurator/non_ros_thread_info", qos,
    [this](const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg) {
      this->non_ros_thread_callback(msg);
    });

  // Create subscription for default domain on this node
  subs_for_each_domain_.push_back(
    this->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", qos,
      [this, default_domain_id](const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->callback_group_callback(default_domain_id, msg);
      }));

  // Create nodes and subscriptions for other domain IDs
  for (size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }

    auto node = cie_thread_configurator::create_node_for_domain(domain_id);
    nodes_for_each_domain_.push_back(node);

    auto sub = node->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", qos,
      [this, domain_id](const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->callback_group_callback(domain_id, msg);
      });
    subs_for_each_domain_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }
}

void ThreadConfiguratorNode::validate_rt_throttling(const YAML::Node & yaml)
{
  if (!yaml["rt_throttling"]) {
    return;
  }

  const auto & rt_bw = yaml["rt_throttling"];

  // Writing to /proc/sys/kernel/sched_rt_{period,runtime}_us requires root (uid 0).
  // Linux capabilities (CAP_SYS_ADMIN etc.) cannot bypass the proc sysctl DAC check.
  // Instead, we validate that the current kernel values match the config and guide the
  // user to apply them via /etc/sysctl.d/ if they differ.

  auto read_sysctl = [this](const std::string & path) -> std::optional<int> {
    std::ifstream file(path);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", path.c_str(), strerror(errno));
      return std::nullopt;
    }
    int value;
    if (!(file >> value)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read integer from %s", path.c_str());
      return std::nullopt;
    }
    return value;
  };

  bool mismatch = false;

  if (rt_bw["period_us"]) {
    int expected = rt_bw["period_us"].as<int>();
    auto actual = read_sysctl("/proc/sys/kernel/sched_rt_period_us");
    if (actual.has_value()) {
      if (actual.value() != expected) {
        RCLCPP_ERROR(
          this->get_logger(), "sched_rt_period_us mismatch: expected %d, actual %d", expected,
          actual.value());
        mismatch = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "sched_rt_period_us is already set to %d", expected);
      }
    }
  }

  if (rt_bw["runtime_us"]) {
    int expected = rt_bw["runtime_us"].as<int>();
    auto actual = read_sysctl("/proc/sys/kernel/sched_rt_runtime_us");
    if (actual.has_value()) {
      if (actual.value() != expected) {
        RCLCPP_ERROR(
          this->get_logger(), "sched_rt_runtime_us mismatch: expected %d, actual %d", expected,
          actual.value());
        mismatch = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "sched_rt_runtime_us is already set to %d", expected);
      }
    }
  }

  if (mismatch) {
    std::string message =
      "rt_throttling values do not match the configuration. "
      "Please create /etc/sysctl.d/99-rt-throttling.conf with the following content and reboot "
      "(or run 'sudo sysctl --system'):\n";

    if (rt_bw["period_us"]) {
      message +=
        "  kernel.sched_rt_period_us = " + std::to_string(rt_bw["period_us"].as<int>()) + "\n";
    }
    if (rt_bw["runtime_us"]) {
      message += "  kernel.sched_rt_runtime_us = " + std::to_string(rt_bw["runtime_us"].as<int>());
    }

    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }
}

ThreadConfiguratorNode::~ThreadConfiguratorNode()
{
  if (cgroup_num_ > 0) {
    for (int i = 0; i < cgroup_num_; i++) {
      rmdir(("/sys/fs/cgroup/cpuset/" + std::to_string(i)).c_str());
    }
  }
}

bool ThreadConfiguratorNode::all_applied()
{
  return unapplied_num_ == 0;
}

void ThreadConfiguratorNode::print_all_unapplied()
{
  RCLCPP_WARN(this->get_logger(), "Following callback groups are not yet configured");

  for (auto & config : callback_group_configs_) {
    if (!config.applied) {
      RCLCPP_WARN(this->get_logger(), "  - %s", config.thread_str.c_str());
    }
  }

  RCLCPP_WARN(this->get_logger(), "Following non-ROS threads are not yet configured");

  for (auto & config : non_ros_thread_configs_) {
    if (!config.applied) {
      RCLCPP_WARN(this->get_logger(), "  - %s", config.thread_str.c_str());
    }
  }
}

bool ThreadConfiguratorNode::set_affinity_by_cgroup(
  int64_t thread_id, const std::vector<int> & cpus)
{
  std::string cgroup_path = "/sys/fs/cgroup/cpuset/" + std::to_string(cgroup_num_++);
  if (!std::filesystem::create_directory(cgroup_path)) {
    return false;
  }

  std::string cpus_path = cgroup_path + "/cpuset.cpus";
  if (std::ofstream cpus_file{cpus_path}) {
    for (int cpu : cpus) {
      cpus_file << cpu << ",";
    }
  } else {
    return false;
  }

  std::string mems_path = cgroup_path + "/cpuset.mems";
  if (std::ofstream mems_file{mems_path}) {
    mems_file << 0;
  } else {
    return false;
  }

  std::string tasks_path = cgroup_path + "/tasks";
  if (std::ofstream tasks_file{tasks_path}) {
    tasks_file << thread_id;
  } else {
    return false;
  }

  return true;
}

bool ThreadConfiguratorNode::issue_syscalls(const ThreadConfig & config)
{
  if (
    config.policy == "SCHED_OTHER" || config.policy == "SCHED_BATCH" ||
    config.policy == "SCHED_IDLE") {
    struct sched_param param;
    param.sched_priority = 0;

    static std::unordered_map<std::string, int> m = {
      {"SCHED_OTHER", SCHED_OTHER},
      {"SCHED_BATCH", SCHED_BATCH},
      {"SCHED_IDLE", SCHED_IDLE},
    };

    if (sched_setscheduler(config.thread_id, m[config.policy], &param) == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
        config.thread_str.c_str(), config.thread_id, strerror(errno));
      return false;
    }

    // Specify nice value
    if (setpriority(PRIO_PROCESS, config.thread_id, config.priority) == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure nice value (id=%s, tid=%ld): %s",
        config.thread_str.c_str(), config.thread_id, strerror(errno));
      return false;
    }

  } else if (config.policy == "SCHED_FIFO" || config.policy == "SCHED_RR") {
    struct sched_param param;
    param.sched_priority = config.priority;

    static std::unordered_map<std::string, int> m = {
      {"SCHED_FIFO", SCHED_FIFO},
      {"SCHED_RR", SCHED_RR},
    };

    if (sched_setscheduler(config.thread_id, m[config.policy], &param) == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
        config.thread_str.c_str(), config.thread_id, strerror(errno));
      return false;
    }

  } else if (config.policy == "SCHED_DEADLINE") {
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = config.runtime;
    attr.sched_period = config.period;
    attr.sched_deadline = config.deadline;

    if (sched_setattr(config.thread_id, &attr, 0) == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure policy (id=%s, tid=%ld): %s",
        config.thread_str.c_str(), config.thread_id, strerror(errno));
      return false;
    }
  }

  if (config.affinity.size() > 0) {
    if (config.policy == "SCHED_DEADLINE") {
      if (!set_affinity_by_cgroup(config.thread_id, config.affinity)) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure affinity (id=%s, tid=%ld): %s",
          config.thread_str.c_str(), config.thread_id,
          "Please disable cgroup v2 if used: "
          "`systemd.unified_cgroup_hierarchy=0`");
        return false;
      }
    } else {
      cpu_set_t set;
      CPU_ZERO(&set);
      for (int cpu : config.affinity) {
        CPU_SET(cpu, &set);
      }
      if (sched_setaffinity(config.thread_id, sizeof(set), &set) == -1) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure affinity (id=%s, tid=%ld): %s",
          config.thread_str.c_str(), config.thread_id, strerror(errno));
        return false;
      }
    }
  }

  return true;
}

const std::vector<rclcpp::Node::SharedPtr> & ThreadConfiguratorNode::get_domain_nodes() const
{
  return nodes_for_each_domain_;
}

void ThreadConfiguratorNode::callback_group_callback(
  size_t domain_id, const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg)
{
  auto key = std::make_pair(domain_id, msg->callback_group_id);
  auto it = id_to_callback_group_config_.find(key);
  if (it == id_to_callback_group_config_.end()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received CallbackGroupInfo: but the yaml file does not "
      "contain configuration for domain=%zu, id=%s (tid=%ld)",
      domain_id, msg->callback_group_id.c_str(), msg->thread_id);
    return;
  }

  ThreadConfig * config = it->second;
  if (config->applied) {
    RCLCPP_INFO(
      this->get_logger(),
      "This callback group is already configured. skip (domain=%zu, id=%s, tid=%ld)", domain_id,
      msg->callback_group_id.c_str(), msg->thread_id);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received CallbackGroupInfo: domain=%zu | tid=%ld | %s", domain_id,
    msg->thread_id, msg->callback_group_id.c_str());
  config->thread_id = msg->thread_id;

  if (config->policy == "SCHED_DEADLINE") {
    // delayed applying
    deadline_configs_.push_back(config);
  } else {
    if (!issue_syscalls(*config)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping configuration for callback group (domain=%zu, id=%s, tid=%ld) due to syscall "
        "failure.",
        domain_id, msg->callback_group_id.c_str(), msg->thread_id);
      return;
    }
  }

  config->applied = true;
  unapplied_num_--;
}

void ThreadConfiguratorNode::non_ros_thread_callback(
  const cie_config_msgs::msg::NonRosThreadInfo::SharedPtr msg)
{
  auto it = id_to_non_ros_thread_config_.find(msg->thread_name);
  if (it == id_to_non_ros_thread_config_.end()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received NonRosThreadInfo: but the yaml file does not "
      "contain configuration for name=%s (tid=%ld)",
      msg->thread_name.c_str(), msg->thread_id);
    return;
  }

  ThreadConfig * config = it->second;
  if (config->applied) {
    RCLCPP_INFO(
      this->get_logger(), "This non-ROS thread is already configured. skip (name=%s, tid=%ld)",
      msg->thread_name.c_str(), msg->thread_id);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Received NonRosThreadInfo: tid=%ld | %s", msg->thread_id,
    msg->thread_name.c_str());
  config->thread_id = msg->thread_id;

  if (config->policy == "SCHED_DEADLINE") {
    // delayed applying
    deadline_configs_.push_back(config);
  } else {
    if (!issue_syscalls(*config)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping configuration for non-ROS thread (name=%s, tid=%ld) due to syscall failure.",
        msg->thread_name.c_str(), msg->thread_id);
      return;
    }
  }

  config->applied = true;
  unapplied_num_--;
}

bool ThreadConfiguratorNode::apply_deadline_configs()
{
  for (auto config : deadline_configs_) {
    if (!issue_syscalls(*config)) {
      return false;
    }
  }

  return true;
}

bool ThreadConfiguratorNode::exist_deadline_config()
{
  return !deadline_configs_.empty();
}
