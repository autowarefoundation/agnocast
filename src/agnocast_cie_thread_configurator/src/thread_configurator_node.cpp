#include "agnocast_cie_thread_configurator/thread_configurator_node.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "agnocast_cie_thread_configurator/sched_deadline.hpp"
#include "agnocast_cie_thread_configurator/sched_policy.hpp"
#include "agnocast_cie_thread_configurator/startup_checks.hpp"
#include "agnocast_cie_thread_configurator/system_scan.hpp"
#include "agnocast_cie_thread_configurator/thread_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"
#include "agnocast_cie_config_msgs/srv/reapply_config.hpp"

#include <error.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <utility>

using agnocast_cie_thread_configurator::is_cfs;
using agnocast_cie_thread_configurator::parse_sched_policy;
using agnocast_cie_thread_configurator::SchedPolicy;
using agnocast_cie_thread_configurator::to_kernel_policy;

namespace
{

bool same_cpu_set(const std::vector<int> & desired, const std::optional<std::vector<int>> & current)
{
  if (!current) {
    return false;
  }
  // Both sides are canonical: parse_affinity and parse_manageable_cpu_list
  // each return sorted, deduplicated lists bounded by the same
  // manageable-CPU limit.
  return desired == *current;
}

// Compare-before-set: unedited templates (observed values as the desired
// ones) stay no-ops, and threads the kernel refuses to modify even with
// identical values (stop-class migration/N) are left alone. DEADLINE params
// cannot be read back from stat, so a DEADLINE request always takes the
// syscall path (never counted in-sync; it can still end up in failed).
bool kernel_thread_in_sync(
  const agnocast_cie_thread_configurator::KernelThreadConfig & config,
  const agnocast_cie_thread_configurator::KernelThreadInfo & info)
{
  if (config.policy.has_value()) {
    const auto policy = parse_sched_policy(*config.policy);
    if (!policy || *policy == SchedPolicy::Deadline) {
      return false;
    }
    if (*config.policy != info.policy) {
      return false;
    }
    // The policy classes tune different knobs: nice for CFS, rt_priority for
    // FIFO/RR (the emitter and parser agree on this split).
    const bool tunable_in_sync =
      is_cfs(*policy) ? config.nice == info.nice : config.priority == info.rt_priority;
    if (!tunable_in_sync) {
      return false;
    }
  }
  if (!config.affinity.empty()) {
    if (!same_cpu_set(
          config.affinity,
          agnocast_cie_thread_configurator::parse_manageable_cpu_list(info.affinity))) {
      return false;
    }
  }
  return true;
}

// The multi-sentence guidance is shared by the open- and write-failure
// EACCES/EPERM branches so the two cannot drift apart.
constexpr const char * k_cap_guidance =
  "Writing /proc/irq/<N>/smp_affinity_list requires CAP_DAC_OVERRIDE in addition to "
  "CAP_SYS_NICE. Re-run scripts/setup_thread_configurator.bash to grant "
  "'cap_sys_nice,cap_dac_override=eip' to thread_configurator_node.";

// Shared by the pre-write actions check and the ENOENT open failure. name is
// optional (empty skips the identity check), so an unset one is labeled
// instead of being printed as ''.
void log_irq_vanished(
  const rclcpp::Logger & logger, const agnocast_cie_thread_configurator::IrqConfig & config)
{
  RCLCPP_ERROR(
    logger,
    "Failed to configure IRQ %d: it no longer exists (expected actions '%s'). IRQ numbers can "
    "change across boots or device changes; re-run prerun_node and update the config.",
    config.irq, config.name.empty() ? "<not recorded>" : config.name.c_str());
}

}  // namespace

ThreadConfiguratorNode::ThreadConfiguratorNode(const rclcpp::NodeOptions & options)
: Node("thread_configurator_node", options),
  config_file_([this]() {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    return this->declare_parameter<std::string>("config_file", "", desc);
  }()),
  default_domain_id_(agnocast_cie_thread_configurator::get_default_domain_id())
{
  if (config_file_.empty()) {
    throw std::runtime_error(
      "'config_file' parameter must be provided with a valid YAML file path.");
  }

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_file_);
  } catch (const std::exception & e) {
    throw std::runtime_error("Error reading the YAML file '" + config_file_ + "': " + e.what());
  }

  validate_hardware_info(yaml);
  validate_rt_throttling(yaml);

  RCLCPP_INFO(this->get_logger(), "Loaded config from: %s", config_file_.c_str());

  agnocast_cie_thread_configurator::parse_yaml(
    yaml, default_domain_id_, callback_group_configs_, non_ros_thread_configs_);
  kernel_thread_configs_ = agnocast_cie_thread_configurator::parse_kernel_threads(yaml);
  irq_configs_ = agnocast_cie_thread_configurator::parse_irqs(yaml);

  // Kernel threads and IRQs never announce themselves: configure them here
  // from a /proc scan, outside the announcement-driven accounting below.
  {
    const SectionApplyOutcome kernel_thread_outcome = apply_kernel_thread_configs();
    const SectionApplyOutcome irq_outcome = apply_irq_configs();
    RCLCPP_INFO(
      this->get_logger(),
      "Kernel threads and IRQs configured: applied=%zu, failed=%zu, skipped=%zu",
      kernel_thread_outcome.applied.size() + irq_outcome.applied.size(),
      kernel_thread_outcome.failed.size() + irq_outcome.failed.size(),
      kernel_thread_outcome.skipped.size() + irq_outcome.skipped.size());
  }

  unapplied_num_.store(
    static_cast<int>(callback_group_configs_.size() + non_ros_thread_configs_.size()));

  std::set<size_t> domain_ids;
  for (auto & cfg : callback_group_configs_) {
    domain_ids.insert(cfg.domain_id);
    if (cfg.is_wildcard()) {
      node_to_wildcard_config_[std::make_pair(cfg.domain_id, cfg.wildcard_prefix())] = &cfg;
    } else {
      id_to_callback_group_config_[std::make_pair(cfg.domain_id, cfg.thread_str)] = &cfg;
    }
  }
  for (auto & cfg : non_ros_thread_configs_) {
    id_to_non_ros_thread_config_[cfg.thread_str] = &cfg;
  }

  auto cbg_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

  non_ros_thread_listener_ =
    std::make_unique<agnocast_cie_thread_configurator::NonRosThreadInfoListener>(
      [this](agnocast_cie_thread_configurator::NonRosThreadInfo info) {
        this->non_ros_thread_callback(std::move(info));
      },
      this->get_logger());

  subs_for_each_domain_.push_back(
    this->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this, default_domain_id = default_domain_id_](
        const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->callback_group_callback(default_domain_id, msg);
      }));

  // Create nodes and subscriptions for other domain IDs
  for (size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id_) {
      continue;
    }

    auto node = agnocast_cie_thread_configurator::create_node_for_domain(domain_id);
    nodes_for_each_domain_.push_back(node);

    auto sub = node->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", cbg_qos,
      [this, domain_id](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        this->callback_group_callback(domain_id, msg);
      });
    subs_for_each_domain_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }

  reapply_service_ = this->create_service<agnocast_cie_config_msgs::srv::ReapplyConfig>(
    "~/reapply_config",
    [this](
      const std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Request> request,
      std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Response> response) {
      this->on_reapply_config_request(request, response);
    });
}

void ThreadConfiguratorNode::validate_rt_throttling(const YAML::Node & yaml)
{
  // The reader reports its own failures; check_rt_throttling records such a
  // key with an unset actual, which is skipped below.
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

  const auto report = agnocast_cie_thread_configurator::check_rt_throttling(yaml, read_sysctl);

  for (const auto & check : report.checks) {
    if (!check.actual.has_value()) {
      continue;
    }
    if (*check.actual != check.expected) {
      RCLCPP_ERROR(
        this->get_logger(), "%s mismatch: expected %d, actual %d", check.key.c_str(),
        check.expected, *check.actual);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s is already set to %d", check.key.c_str(), check.expected);
    }
  }

  if (report.mismatch) {
    RCLCPP_ERROR(this->get_logger(), "%s", report.sysctl_guidance.c_str());
  }
}

void ThreadConfiguratorNode::validate_hardware_info(const YAML::Node & yaml)
{
  const auto current_hw_info = agnocast_cie_thread_configurator::get_hardware_info();
  if (current_hw_info.empty()) {
    RCLCPP_WARN(this->get_logger(), "No hardware info from lscpu. Skipping hardware validation.");
    return;
  }

  const auto mismatches =
    agnocast_cie_thread_configurator::check_hardware_info(yaml, current_hw_info);
  if (!mismatches.has_value()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No hardware_info section found in configuration file. Skipping hardware validation.");
    return;
  }

  if (!mismatches->empty()) {
    std::string error_msg = "Hardware validation failed with the following mismatches:\n";
    for (const auto & mismatch : *mismatches) {
      error_msg += "  - " + mismatch.key + ": expected '" + mismatch.expected + "', got '" +
                   mismatch.actual + "'\n";
    }
    throw std::runtime_error(error_msg);
  }

  RCLCPP_INFO(
    this->get_logger(), "Hardware validation successful. Configuration matches this system.");
}

ThreadConfiguratorNode::~ThreadConfiguratorNode()
{
  stop();
  const int cgroup_count = cgroup_num_.load();
  for (int i = 0; i < cgroup_count; i++) {
    rmdir(("/sys/fs/cgroup/cpuset/" + std::to_string(i)).c_str());
  }
}

void ThreadConfiguratorNode::stop() noexcept
{
  if (non_ros_thread_listener_) {
    non_ros_thread_listener_->stop();
  }
}

void ThreadConfiguratorNode::print_all_unapplied()
{
  if (unapplied_num_.load() == 0) {
    return;
  }

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
  const int my_id = cgroup_num_.fetch_add(1, std::memory_order_relaxed);
  std::string cgroup_path = "/sys/fs/cgroup/cpuset/" + std::to_string(my_id);
  // Non-throwing overload: on a cgroup v2 host /sys/fs/cgroup/cpuset does not
  // exist and mkdir fails with ENOENT; the caller's instructive message
  // depends on a false return, not an exception.
  std::error_code ec;
  const bool created = std::filesystem::create_directory(cgroup_path, ec);
  if (ec || !created) {
    return false;
  }

  std::string cpus_path = cgroup_path + "/cpuset.cpus";
  if (std::ofstream cpus_file{cpus_path}) {
    for (size_t i = 0; i < cpus.size(); i++) {
      if (i > 0) {
        cpus_file << ",";
      }
      cpus_file << cpus[i];
    }
    // The kernel rejects invalid content (e.g. a nonexistent CPU) at write(2)
    // time, not at open, so the stream state must be checked after flushing.
    cpus_file.flush();
    if (!cpus_file) {
      return false;
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
    // Attaching a task to an empty or invalid cpuset fails at write(2) time.
    tasks_file.flush();
    if (!tasks_file) {
      return false;
    }
  } else {
    return false;
  }

  return true;
}

bool ThreadConfiguratorNode::issue_syscalls(const ThreadConfig & config, int64_t thread_id)
{
  const auto policy = parse_sched_policy(config.policy);
  if (!policy) {
    RCLCPP_ERROR(
      this->get_logger(), "Unknown scheduling policy '%s' (thread=%s, tid=%" PRId64 ")",
      config.policy.c_str(), config.thread_str.c_str(), thread_id);
    return false;
  }

  // No default: -Werror=switch must reject an unhandled SchedPolicy, since an
  // unhandled case would fall through to the affinity syscalls.
  switch (*policy) {
    case SchedPolicy::Other:
    case SchedPolicy::Batch:
    case SchedPolicy::Idle:
    case SchedPolicy::Fifo:
    case SchedPolicy::Rr: {
      struct sched_param param;
      param.sched_priority = is_cfs(*policy) ? 0 : config.priority;

      if (sched_setscheduler(thread_id, to_kernel_policy(*policy), &param) == -1) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure policy (thread=%s, tid=%" PRId64 "): %s",
          config.thread_str.c_str(), thread_id, strerror(errno));
        return false;
      }

      if (is_cfs(*policy) && setpriority(PRIO_PROCESS, thread_id, config.nice) == -1) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure nice value (thread=%s, tid=%" PRId64 "): %s",
          config.thread_str.c_str(), thread_id, strerror(errno));
        return false;
      }
      break;
    }
    case SchedPolicy::Deadline: {
      struct sched_attr attr;
      memset(&attr, 0, sizeof(attr));
      attr.size = sizeof(attr);
      // SCHED_FLAG_RESET_ON_FORK lets the target thread still call fork(2)/clone(2)
      // after being placed under SCHED_DEADLINE; without it, clone(2) returns EAGAIN.
      // Children reset to SCHED_OTHER; each callback-group thread that needs its own
      // SCHED_DEADLINE gets it via its own CallbackGroupInfo message.
      attr.sched_flags = SCHED_FLAG_RESET_ON_FORK;
      attr.sched_nice = 0;
      attr.sched_priority = 0;

      attr.sched_policy = SCHED_DEADLINE;
      attr.sched_runtime = config.runtime;
      attr.sched_period = config.period;
      attr.sched_deadline = config.deadline;

      if (sched_setattr(thread_id, &attr, 0) == -1) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure policy (thread=%s, tid=%" PRId64 "): %s",
          config.thread_str.c_str(), thread_id, strerror(errno));
        return false;
      }
      break;
    }
  }

  return issue_affinity_syscalls(config.thread_str, policy, config.affinity, thread_id);
}

bool ThreadConfiguratorNode::issue_affinity_syscalls(
  const std::string & thread_str, std::optional<SchedPolicy> policy,
  const std::vector<int> & affinity, int64_t thread_id)
{
  if (affinity.size() > 0) {
    if (policy == SchedPolicy::Deadline) {
      if (!set_affinity_by_cgroup(thread_id, affinity)) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure affinity (thread=%s, tid=%" PRId64 "): %s",
          thread_str.c_str(), thread_id,
          "Please disable cgroup v2 if used: "
          "`systemd.unified_cgroup_hierarchy=0`");
        return false;
      }
    } else {
      cpu_set_t set;
      CPU_ZERO(&set);
      for (int cpu : affinity) {
        CPU_SET(cpu, &set);
      }
      if (sched_setaffinity(thread_id, sizeof(set), &set) == -1) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to configure affinity (thread=%s, tid=%" PRId64 "): %s",
          thread_str.c_str(), thread_id, strerror(errno));
        return false;
      }
    }
  }

  return true;
}

ThreadConfiguratorNode::SectionApplyOutcome ThreadConfiguratorNode::apply_kernel_thread_configs()
{
  SectionApplyOutcome outcome;
  const bool any_managed = std::any_of(
    kernel_thread_configs_.begin(), kernel_thread_configs_.end(),
    [](const KernelThreadConfig & config) { return config.is_managed(); });
  if (!any_managed) {
    return outcome;
  }

  const auto scanned = agnocast_cie_thread_configurator::scan_kernel_threads();
  for (const auto & config : kernel_thread_configs_) {
    if (!config.is_managed()) {
      continue;
    }
    const auto matches =
      agnocast_cie_thread_configurator::find_kernel_threads_by_comm(scanned, config.comm);
    if (matches.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "No running kernel thread matches comm=%s; skipping. It may appear later; call "
        "~/reapply_config to retry.",
        config.comm.c_str());
      outcome.skipped.push_back(config.comm);
      continue;
    }

    // Reuse the announcement-path syscall code (and its error wording)
    // verbatim by shaping the entry as a ThreadConfig: one desired state,
    // applied to every matched tid.
    std::optional<ThreadConfig> shaped;
    if (config.policy.has_value()) {
      ThreadConfig tmp;
      tmp.thread_str = config.comm;
      tmp.policy = *config.policy;
      tmp.nice = config.nice;
      tmp.priority = config.priority;
      tmp.affinity = config.affinity;
      tmp.runtime = config.runtime;
      tmp.period = config.period;
      tmp.deadline = config.deadline;
      shaped = std::move(tmp);
    }

    for (const auto * info : matches) {
      std::string key = config.comm + ":" + std::to_string(info->tid);
      if (kernel_thread_in_sync(config, *info)) {
        outcome.applied.push_back(std::move(key));
        continue;
      }
      // A per-CPU kthread's affinity is kernel-fixed, but a request equal to
      // that fixed value needs no change; only a different one is impossible.
      const bool affinity_on_fixed = !config.affinity.empty() && info->no_setaffinity;
      if (
        affinity_on_fixed &&
        !same_cpu_set(
          config.affinity,
          agnocast_cie_thread_configurator::parse_manageable_cpu_list(info->affinity))) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to configure affinity (thread=%s, tid=%" PRId64
          "): per-CPU kernel thread "
          "(PF_NO_SETAFFINITY); the kernel fixes its affinity. Set 'affinity' to %s for this "
          "entry.",
          config.comm.c_str(), info->tid,
          std::string(agnocast_cie_thread_configurator::k_unmanageable).c_str());
        outcome.failed.push_back(std::move(key));
        continue;
      }

      bool ok = false;
      if (shaped.has_value()) {
        if (affinity_on_fixed) {
          // sched_setaffinity returns EINVAL for such a thread even with the
          // identical set, so issue only the policy part.
          ThreadConfig policy_only = *shaped;
          policy_only.affinity.clear();
          ok = issue_syscalls(policy_only, info->tid);
        } else {
          ok = issue_syscalls(*shaped, info->tid);
        }
      } else {
        // Branch on the observed policy: an affinity-only entry matching a
        // thread currently under SCHED_DEADLINE needs the cgroup path.
        ok = issue_affinity_syscalls(
          config.comm, parse_sched_policy(info->policy), config.affinity, info->tid);
      }

      if (ok) {
        RCLCPP_INFO(
          this->get_logger(), "Configured kernel thread (comm=%s, tid=%" PRId64 ")",
          config.comm.c_str(), info->tid);
        outcome.applied.push_back(std::move(key));
      } else {
        outcome.failed.push_back(std::move(key));
      }
    }
  }
  return outcome;
}

ThreadConfiguratorNode::SectionApplyOutcome ThreadConfiguratorNode::apply_irq_configs() const
{
  SectionApplyOutcome outcome;
  const bool any_managed = std::any_of(
    irq_configs_.begin(), irq_configs_.end(),
    [](const IrqConfig & config) { return config.is_managed(); });
  if (!any_managed) {
    return outcome;
  }

  // A running irqbalance would periodically rewrite smp_affinity, silently
  // undoing whatever is written here, so entries needing a write are failed
  // instead of written. The gate sits after the per-entry compare-before-set:
  // in-sync entries need no write and still count as applied, or every entry
  // of an unedited template would be flagged on distros that enable
  // irqbalance by default.
  const bool irqbalance_detected =
    agnocast_cie_thread_configurator::process_with_comm_exists("irqbalance");
  bool irqbalance_reported = false;

  for (const auto & config : irq_configs_) {
    if (!config.is_managed()) {
      continue;
    }
    std::string key = std::to_string(config.irq);

    const auto actions = agnocast_cie_thread_configurator::read_irq_actions(config.irq);
    if (!actions) {
      log_irq_vanished(this->get_logger(), config);
      outcome.failed.push_back(std::move(key));
      continue;
    }
    if (!config.name.empty() && *actions != config.name) {
      RCLCPP_ERROR(
        this->get_logger(),
        "IRQ %d identity mismatch: expected actions '%s', current '%s'. IRQ numbers can change "
        "across boots; re-run prerun_node and update the config.",
        config.irq, config.name.c_str(), actions->c_str());
      outcome.failed.push_back(std::move(key));
      continue;
    }

    std::string current;
    {
      std::ifstream file("/proc/irq/" + key + "/smp_affinity_list");
      if (!std::getline(file, current)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not read the current affinity of IRQ %d; treating it as out "
          "of sync.",
          config.irq);
      }
    }
    if (same_cpu_set(
          config.affinity, agnocast_cie_thread_configurator::parse_manageable_cpu_list(current))) {
      // Also spares kernel-managed IRQs (which reject every write with EIO)
      // from a spurious error when the recorded value is still current.
      outcome.applied.push_back(std::move(key));
      continue;
    }

    if (irqbalance_detected) {
      if (!irqbalance_reported) {
        RCLCPP_ERROR(
          this->get_logger(),
          "IRQ affinity changes are needed but irqbalance is running; it would periodically "
          "overwrite them, so the out-of-sync entries are failed instead of written. Disable it "
          "(sudo systemctl disable --now irqbalance) and call ~/reapply_config to retry.");
        irqbalance_reported = true;
      }
      outcome.failed.push_back(std::move(key));
      continue;
    }

    if (write_irq_affinity_file(config)) {
      outcome.applied.push_back(std::move(key));
    } else {
      outcome.failed.push_back(std::move(key));
    }
  }
  return outcome;
}

bool ThreadConfiguratorNode::write_irq_affinity_file(const IrqConfig & config) const
{
  const std::string path = "/proc/irq/" + std::to_string(config.irq) + "/smp_affinity_list";
  const std::string value = agnocast_cie_thread_configurator::format_cpu_list(config.affinity);

  // Raw open/write instead of std::ofstream: the instructive messages below
  // depend on which errno occurred, and EIO only appears at write(2) time.
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd == -1) {
    // The logging macro may clobber errno before its arguments are evaluated.
    const int open_errno = errno;
    if (open_errno == EACCES || open_errno == EPERM) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure IRQ %d affinity: %s. %s", config.irq,
        strerror(open_errno), k_cap_guidance);
    } else if (open_errno == ENOENT) {
      log_irq_vanished(this->get_logger(), config);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to configure IRQ %d affinity: %s", config.irq,
        strerror(open_errno));
    }
    return false;
  }

  const ssize_t written = ::write(fd, value.c_str(), value.size());
  const int write_errno = errno;
  ::close(fd);
  if (written == static_cast<ssize_t>(value.size())) {
    RCLCPP_INFO(
      this->get_logger(), "Configured IRQ %d affinity to [%s]", config.irq, value.c_str());
    return true;
  }

  if (written == -1 && write_errno == EIO) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to configure IRQ %d affinity: %s. The kernel does not allow user-space affinity "
      "changes for this IRQ (e.g. IRQ 0 timer or kernel-managed MSI-X vectors). Set 'affinity' "
      "to %s for this entry.",
      config.irq, strerror(write_errno),
      std::string(agnocast_cie_thread_configurator::k_unmanageable).c_str());
  } else if (written == -1 && (write_errno == EACCES || write_errno == EPERM)) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to configure IRQ %d affinity: %s. %s", config.irq,
      strerror(write_errno), k_cap_guidance);
  } else if (written == -1) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to configure IRQ %d affinity: %s. The kernel rejected CPU list '%s' (offline CPUs "
      "or no valid CPU in the mask?).",
      config.irq, strerror(write_errno), value.c_str());
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to configure IRQ %d affinity: short write to %s", config.irq,
      path.c_str());
  }
  return false;
}

const std::vector<rclcpp::Node::SharedPtr> & ThreadConfiguratorNode::get_domain_nodes() const
{
  return nodes_for_each_domain_;
}

void ThreadConfiguratorNode::callback_group_callback(
  size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg)
{
  ThreadConfig * config = nullptr;
  bool already_seen = false;

  // Exact entries take precedence over wildcard ("<node>/*") entries.
  auto it = id_to_callback_group_config_.find(std::make_pair(domain_id, msg->callback_group_id));
  if (it != id_to_callback_group_config_.end()) {
    config = it->second;
    already_seen = config->applied;
    // Drop stale tracking left in a wildcard entry from before this exact entry
    // existed; otherwise reapply would apply both entries to the same thread.
    auto wit = node_to_wildcard_config_.find(std::make_pair(
      domain_id, agnocast_cie_thread_configurator::extract_node_part(msg->callback_group_id)));
    if (wit != node_to_wildcard_config_.end()) {
      wit->second->matched_tids.erase(msg->callback_group_id);
    }
  } else {
    auto wit = node_to_wildcard_config_.find(std::make_pair(
      domain_id, agnocast_cie_thread_configurator::extract_node_part(msg->callback_group_id)));
    if (wit == node_to_wildcard_config_.end()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received CallbackGroupInfo: but the yaml file does not "
        "contain configuration for domain=%zu, id=%s (tid=%" PRId64 ")",
        domain_id, msg->callback_group_id.c_str(), msg->thread_id);
      return;
    }
    config = wit->second;
    already_seen = config->matched_tids.count(msg->callback_group_id) > 0;
    RCLCPP_INFO(
      this->get_logger(), "Callback group (domain=%zu, id=%s) matched wildcard entry '%s'",
      domain_id, msg->callback_group_id.c_str(), config->thread_str.c_str());
  }

  if (already_seen) {
    // Always re-apply: the OS may reuse the same thread IDs after an application
    // restarts, so we cannot use thread_id equality to skip reconfiguration.
    // "tracked", not "configured": a recorded wildcard tid's syscall may have failed.
    RCLCPP_INFO(
      this->get_logger(),
      "Re-applying configuration for already tracked callback group "
      "(domain=%zu, id=%s, tid=%" PRId64 ")",
      domain_id, msg->callback_group_id.c_str(), msg->thread_id);
  }

  RCLCPP_INFO(
    this->get_logger(), "Received CallbackGroupInfo: domain=%zu | tid=%" PRId64 " | %s", domain_id,
    msg->thread_id, msg->callback_group_id.c_str());
  // Record the tid before the syscall so a failed attempt can be retried via reapply.
  if (config->is_wildcard()) {
    config->matched_tids[msg->callback_group_id] = msg->thread_id;
  } else {
    config->thread_id = msg->thread_id;
  }

  if (!issue_syscalls(*config, msg->thread_id)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Skipping configuration for callback group (domain=%zu, id=%s, tid=%" PRId64
      ") due to syscall "
      "failure.",
      domain_id, msg->callback_group_id.c_str(), msg->thread_id);
    return;
  }

  if (!config->applied) {
    config->applied = true;
    if (unapplied_num_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      bool expected = false;
      if (configured_at_least_once_.compare_exchange_strong(expected, true)) {
        RCLCPP_INFO(this->get_logger(), "Success: All of the configurations are applied.");
      }
    }
  }
}

void ThreadConfiguratorNode::non_ros_thread_callback(
  agnocast_cie_thread_configurator::NonRosThreadInfo info)
{
  std::lock_guard<std::mutex> lk(non_ros_state_mutex_);

  auto it = id_to_non_ros_thread_config_.find(info.name);
  if (it == id_to_non_ros_thread_config_.end()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received NonRosThreadInfo: but the yaml file does not "
      "contain configuration for name=%s (tid=%" PRId64 ")",
      info.name.c_str(), info.tid);
    return;
  }

  ThreadConfig * config = it->second;
  if (config->applied) {
    // Always re-apply: the OS may reuse the same thread IDs after an application
    // restarts, so we cannot use thread_id equality to skip reconfiguration.
    RCLCPP_INFO(
      this->get_logger(),
      "Re-applying configuration for already configured non-ROS thread (name=%s, tid=%" PRId64 ")",
      info.name.c_str(), info.tid);
  }

  RCLCPP_INFO(
    this->get_logger(), "Received NonRosThreadInfo: tid=%" PRId64 " | %s", info.tid,
    info.name.c_str());
  config->thread_id = info.tid;

  if (!issue_syscalls(*config, info.tid)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Skipping configuration for non-ROS thread (name=%s, tid=%" PRId64
      ") due to syscall "
      "failure.",
      info.name.c_str(), info.tid);
    return;
  }

  if (!config->applied) {
    config->applied = true;
    if (unapplied_num_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      bool expected = false;
      if (configured_at_least_once_.compare_exchange_strong(expected, true)) {
        RCLCPP_INFO(this->get_logger(), "Success: All of the configurations are applied.");
      }
    }
  }
}

void ThreadConfiguratorNode::on_reapply_config_request(
  const std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Request> /*request*/,
  std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Reapplying configuration from: %s", config_file_.c_str());

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_file_);
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = "YAML parse error for '" + config_file_ + "': " + e.what();
    RCLCPP_ERROR(this->get_logger(), "Reapply rejected: %s", response->error_message.c_str());
    return;
  }

  std::vector<ThreadConfig> new_cb;
  std::vector<ThreadConfig> new_nrt;
  std::vector<KernelThreadConfig> new_kernel_threads;
  std::vector<IrqConfig> new_irqs;
  try {
    agnocast_cie_thread_configurator::parse_yaml(yaml, default_domain_id_, new_cb, new_nrt);
    new_kernel_threads = agnocast_cie_thread_configurator::parse_kernel_threads(yaml);
    new_irqs = agnocast_cie_thread_configurator::parse_irqs(yaml);
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = "YAML validation error for '" + config_file_ + "': " + e.what();
    RCLCPP_ERROR(this->get_logger(), "Reapply rejected: %s", response->error_message.c_str());
    return;
  }

  // Carry known tids over from the existing state, same-form entries only:
  // exact entries carry thread_id, wildcard entries carry matched_tids. An
  // entry that changed form (exact <-> wildcard) starts fresh and is only
  // picked up at the next announcement (see ReapplyConfig.srv). 'applied' is
  // intentionally NOT carried: a syscall failure below must leave
  // applied=false, not the stale 'true' that a verbatim carry-over would imply.
  for (auto & cfg : new_cb) {
    if (cfg.is_wildcard()) {
      auto it = node_to_wildcard_config_.find(std::make_pair(cfg.domain_id, cfg.wildcard_prefix()));
      if (it != node_to_wildcard_config_.end()) {
        cfg.matched_tids = it->second->matched_tids;
      }
    } else {
      auto it = id_to_callback_group_config_.find(std::make_pair(cfg.domain_id, cfg.thread_str));
      if (it != id_to_callback_group_config_.end()) {
        cfg.thread_id = it->second->thread_id;
      }
    }
  }

  callback_group_configs_ = std::move(new_cb);
  id_to_callback_group_config_.clear();
  node_to_wildcard_config_.clear();
  for (auto & cfg : callback_group_configs_) {
    if (cfg.is_wildcard()) {
      node_to_wildcard_config_[std::make_pair(cfg.domain_id, cfg.wildcard_prefix())] = &cfg;
    } else {
      id_to_callback_group_config_[std::make_pair(cfg.domain_id, cfg.thread_str)] = &cfg;
    }
  }

  // One lock spans non-ROS carry-over + swap + unapplied_num_ recompute so the
  // listener cannot write thread_id into the old vector between phases (the
  // update would be dropped by move-assign) nor race the counter store.
  {
    std::lock_guard<std::mutex> lk(non_ros_state_mutex_);
    for (auto & cfg : new_nrt) {
      auto it = id_to_non_ros_thread_config_.find(cfg.thread_str);
      if (it != id_to_non_ros_thread_config_.end()) {
        cfg.thread_id = it->second->thread_id;
      }
    }
    non_ros_thread_configs_ = std::move(new_nrt);
    id_to_non_ros_thread_config_.clear();
    for (auto & cfg : non_ros_thread_configs_) {
      id_to_non_ros_thread_config_[cfg.thread_str] = &cfg;
    }

    unapplied_num_.store(
      static_cast<int>(callback_group_configs_.size() + non_ros_thread_configs_.size()),
      std::memory_order_release);
  }

  // No carry-over: kernel threads and IRQs are matched against a fresh /proc
  // scan on every apply pass.
  kernel_thread_configs_ = std::move(new_kernel_threads);
  irq_configs_ = std::move(new_irqs);

  for (auto & cfg : callback_group_configs_) {
    if (cfg.is_wildcard()) {
      // One applied/failed key per known instance; the pattern itself is
      // reported as skipped only while no instance has been announced yet.
      if (cfg.matched_tids.empty()) {
        response->skipped_callback_groups.push_back(
          std::to_string(cfg.domain_id) + ":" + cfg.thread_str);
        continue;
      }
      bool any_applied = false;
      for (const auto & [full_id, tid] : cfg.matched_tids) {
        std::string key = std::to_string(cfg.domain_id) + ":" + full_id;
        if (issue_syscalls(cfg, tid)) {
          response->applied_callback_groups.push_back(std::move(key));
          any_applied = true;
        } else {
          response->failed_callback_groups.push_back(std::move(key));
        }
      }
      if (any_applied) {
        cfg.applied = true;
        unapplied_num_.fetch_sub(1, std::memory_order_acq_rel);
      }
      continue;
    }

    std::string key = std::to_string(cfg.domain_id) + ":" + cfg.thread_str;
    if (cfg.thread_id == -1) {
      response->skipped_callback_groups.push_back(std::move(key));
      continue;
    }
    if (issue_syscalls(cfg, cfg.thread_id)) {
      response->applied_callback_groups.push_back(std::move(key));
      cfg.applied = true;
      unapplied_num_.fetch_sub(1, std::memory_order_acq_rel);
    } else {
      response->failed_callback_groups.push_back(std::move(key));
    }
  }

  {
    std::lock_guard<std::mutex> lk(non_ros_state_mutex_);
    for (auto & cfg : non_ros_thread_configs_) {
      if (cfg.thread_id == -1) {
        response->skipped_non_ros_threads.push_back(cfg.thread_str);
        continue;
      }
      if (issue_syscalls(cfg, cfg.thread_id)) {
        response->applied_non_ros_threads.push_back(cfg.thread_str);
        if (!cfg.applied) {
          cfg.applied = true;
          unapplied_num_.fetch_sub(1, std::memory_order_acq_rel);
        }
      } else {
        response->failed_non_ros_threads.push_back(cfg.thread_str);
      }
    }
  }

  {
    SectionApplyOutcome kernel_thread_outcome = apply_kernel_thread_configs();
    SectionApplyOutcome irq_outcome = apply_irq_configs();
    response->applied_kernel_threads = std::move(kernel_thread_outcome.applied);
    response->failed_kernel_threads = std::move(kernel_thread_outcome.failed);
    response->skipped_kernel_threads = std::move(kernel_thread_outcome.skipped);
    response->applied_irqs = std::move(irq_outcome.applied);
    response->failed_irqs = std::move(irq_outcome.failed);
  }

  response->success = true;
  RCLCPP_INFO(
    this->get_logger(), "Reapply done: applied=%zu, failed=%zu, skipped=%zu",
    response->applied_callback_groups.size() + response->applied_non_ros_threads.size() +
      response->applied_kernel_threads.size() + response->applied_irqs.size(),
    response->failed_callback_groups.size() + response->failed_non_ros_threads.size() +
      response->failed_kernel_threads.size() + response->failed_irqs.size(),
    response->skipped_callback_groups.size() + response->skipped_non_ros_threads.size() +
      response->skipped_kernel_threads.size());

  // configured_at_least_once_ is one-shot, so the "all applied" log is not
  // re-emitted on reapply; operators need a dedicated reapply-complete signal.
  if (
    response->failed_callback_groups.empty() && response->failed_non_ros_threads.empty() &&
    response->skipped_callback_groups.empty() && response->skipped_non_ros_threads.empty() &&
    response->failed_kernel_threads.empty() && response->skipped_kernel_threads.empty() &&
    response->failed_irqs.empty()) {
    RCLCPP_INFO(this->get_logger(), "Reapply: all entries successfully (re-)applied.");
  }
}
