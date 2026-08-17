#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <linux/sched.h>
#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace agnocast_cie_thread_configurator
{

namespace
{

constexpr int k_nice_min = -20;
constexpr int k_nice_max = 19;
constexpr int k_rt_priority_min = 1;
constexpr int k_rt_priority_max = 99;

bool is_cfs_policy(const std::string & policy)
{
  return policy == "SCHED_OTHER" || policy == "SCHED_BATCH" || policy == "SCHED_IDLE";
}

// 'nice' is required for the CFS policies (SCHED_OTHER/BATCH/IDLE);
// parse_rt_priority is the mirror image for SCHED_FIFO/SCHED_RR. `entry_desc`
// is the "id=..."/"name=..." fragment used in messages.
int parse_nice(const YAML::Node & entry, const std::string & policy, const std::string & entry_desc)
{
  const YAML::Node nice = entry["nice"];
  // A key with an empty value ("nice:") is a defined null node, so `!nice`
  // alone would pass it on to as<int>()'s context-free BadConversion.
  if (!nice || nice.IsNull()) {
    throw std::runtime_error("Policy '" + policy + "' requires 'nice' for " + entry_desc);
  }
  int value = 0;
  try {
    value = nice.as<int>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error("'nice' must be an integer for " + entry_desc);
  }
  if (value < k_nice_min || value > k_nice_max) {
    // setpriority(2) would silently clamp an out-of-range value to
    // [-20, 19]; reject it here so a misunderstanding of the scale
    // (e.g. an rt_priority-style 50) fails loudly instead.
    throw std::runtime_error(
      "'nice' must be in [-20, 19] for " + entry_desc + ", got " + std::to_string(value));
  }
  return value;
}

int parse_rt_priority(
  const YAML::Node & entry, const std::string & policy, const std::string & entry_desc)
{
  const YAML::Node priority = entry["priority"];
  if (!priority || priority.IsNull()) {
    throw std::runtime_error("Policy '" + policy + "' requires 'priority' for " + entry_desc);
  }
  int value = 0;
  try {
    value = priority.as<int>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error("'priority' must be an integer for " + entry_desc);
  }
  if (value < k_rt_priority_min || value > k_rt_priority_max) {
    throw std::runtime_error(
      "'priority' must be in [1, 99] for " + entry_desc + ", got " + std::to_string(value));
  }
  return value;
}

// CPU_SET(3) silently ignores CPUs outside [0, CPU_SETSIZE) and
// sched_setaffinity(2) silently intersects away CPUs that do not exist on
// this machine, so a typo'd "affinity: [2, 2000]" would pin to {2} yet be
// reported as configured (and the SCHED_DEADLINE cgroup path would write the
// raw value into cpuset.cpus); reject such values here instead. Checking
// against the actual CPU count is valid because the config is always parsed
// on the machine that applies it. The result is sorted and deduplicated so
// downstream consumers see a canonical list.
std::vector<int> parse_affinity(const YAML::Node & entry, const std::string & entry_desc)
{
  const YAML::Node affinity = entry["affinity"];
  std::vector<int> cpus;
  // Absent or null means "do not manage affinity".
  if (!affinity || affinity.IsNull()) {
    return cpus;
  }
  if (!affinity.IsSequence()) {
    throw std::runtime_error(
      "'affinity' must be a list of CPU numbers (e.g. [2, 3]) for " + entry_desc);
  }
  const long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
  const int max_cpu = static_cast<int>(std::min<long>(CPU_SETSIZE, num_cpus)) - 1;
  for (const auto & cpu_node : affinity) {
    int cpu = 0;
    try {
      cpu = cpu_node.as<int>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error(
        "'affinity' must contain only integers for " + entry_desc + ", got '" +
        (cpu_node.IsScalar() ? cpu_node.Scalar() : std::string("<non-scalar>")) + "'");
    }
    if (cpu < 0 || cpu > max_cpu) {
      throw std::runtime_error(
        "'affinity' CPU " + std::to_string(cpu) + " must be in [0, " + std::to_string(max_cpu) +
        "] (this machine has " + std::to_string(num_cpus) + " CPUs) for " + entry_desc);
    }
    cpus.push_back(cpu);
  }
  std::sort(cpus.begin(), cpus.end());
  cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
  return cpus;
}

}  // namespace

const std::unordered_map<std::string, int> policy_to_sched_const = {
  {"SCHED_OTHER", SCHED_OTHER}, {"SCHED_BATCH", SCHED_BATCH}, {"SCHED_IDLE", SCHED_IDLE},
  {"SCHED_FIFO", SCHED_FIFO},   {"SCHED_RR", SCHED_RR},       {"SCHED_DEADLINE", SCHED_DEADLINE},
};

bool ThreadConfig::is_wildcard() const noexcept
{
  static constexpr std::string_view suffix = "/*";
  return thread_str.size() >= suffix.size() &&
         thread_str.compare(thread_str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string ThreadConfig::wildcard_prefix() const
{
  return thread_str.substr(0, thread_str.size() - 2);
}

std::string extract_node_part(const std::string & callback_group_id)
{
  return callback_group_id.substr(0, callback_group_id.find('@'));
}

void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out)
{
  YAML::Node callback_groups = yaml["callback_groups"];
  YAML::Node non_ros_threads = yaml["non_ros_threads"];

  callback_groups_out.clear();
  non_ros_threads_out.clear();
  callback_groups_out.resize(callback_groups.size());
  non_ros_threads_out.resize(non_ros_threads.size());

  for (size_t i = 0; i < callback_groups.size(); ++i) {
    const auto & cg = callback_groups[i];
    auto & cfg = callback_groups_out[i];

    cfg.thread_str = cg["id"].as<std::string>();
    if (cfg.thread_str.find('*') != std::string::npos) {
      // A typo'd pattern silently treated as an exact id would never match,
      // so any id containing '*' must be a well-formed "<node name>/*".
      if (!cfg.is_wildcard()) {
        throw std::runtime_error(
          "Invalid id '" + cfg.thread_str +
          "': '*' is only allowed as a trailing \"/*\" wildcard (e.g. /my_node/*)");
      }
      const std::string prefix = cfg.wildcard_prefix();
      if (prefix.empty() || prefix.find('*') != std::string::npos) {
        throw std::runtime_error(
          "Invalid wildcard id '" + cfg.thread_str +
          "': the part before \"/*\" must be a non-empty node name without '*'");
      }
      if (prefix.find('@') != std::string::npos) {
        throw std::runtime_error(
          "Invalid wildcard id '" + cfg.thread_str +
          "': the part before \"/*\" must be a plain node name, not a full callback-group id "
          "containing '@'");
      }
    }
    cfg.domain_id = cg["domain_id"] ? cg["domain_id"].as<size_t>() : default_domain_id;
    cfg.affinity = parse_affinity(cg, "id=" + cfg.thread_str);
    cfg.policy = cg["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for id=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = cg["runtime"].as<unsigned int>();
      cfg.period = cg["period"].as<unsigned int>();
      cfg.deadline = cg["deadline"].as<unsigned int>();
    } else if (is_cfs_policy(cfg.policy)) {
      cfg.nice = parse_nice(cg, cfg.policy, "id=" + cfg.thread_str);
    } else {
      cfg.priority = parse_rt_priority(cg, cfg.policy, "id=" + cfg.thread_str);
    }
  }

  for (size_t i = 0; i < non_ros_threads.size(); ++i) {
    const auto & nrt = non_ros_threads[i];
    auto & cfg = non_ros_threads_out[i];

    cfg.thread_str = nrt["name"].as<std::string>();
    cfg.affinity = parse_affinity(nrt, "name=" + cfg.thread_str);
    cfg.policy = nrt["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for name=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = nrt["runtime"].as<unsigned int>();
      cfg.period = nrt["period"].as<unsigned int>();
      cfg.deadline = nrt["deadline"].as<unsigned int>();
    } else if (is_cfs_policy(cfg.policy)) {
      cfg.nice = parse_nice(nrt, cfg.policy, "name=" + cfg.thread_str);
    } else {
      cfg.priority = parse_rt_priority(nrt, cfg.policy, "name=" + cfg.thread_str);
    }
  }

  // Reject duplicates: id_to_*_config_ would silently collapse them to the
  // last-inserted entry, dropping earlier YAML lines without warning.
  // The std::find_if rewrite cppcheck suggests would hide a side-effecting
  // predicate inside the algorithm; a plain loop is clearer here.
  std::unordered_set<std::string> seen_cb;
  for (const auto & c : callback_groups_out) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen_cb.insert(std::to_string(c.domain_id) + ":" + c.thread_str).second) {
      throw std::runtime_error(
        "Duplicate callback_group entry: domain_id=" + std::to_string(c.domain_id) +
        ", id=" + c.thread_str);
    }
  }
  std::unordered_set<std::string> seen_nrt;
  for (const auto & c : non_ros_threads_out) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen_nrt.insert(c.thread_str).second) {
      throw std::runtime_error("Duplicate non_ros_thread entry: name=" + c.thread_str);
    }
  }
}

}  // namespace agnocast_cie_thread_configurator
