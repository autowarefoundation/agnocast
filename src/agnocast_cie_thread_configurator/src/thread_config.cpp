#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <linux/sched.h>
#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace agnocast_cie_thread_configurator
{

namespace
{

// Unset = the attribute must not be applied: YAML null / absent key (the
// user's opt-out) or, when `allow_unmanageable` is set, the UNMANAGEABLE
// sentinel (a kernel/tool constraint). The sentinel is scoped to the
// kernel_threads/irqs sections; elsewhere it is an ordinary invalid value.
bool is_unset(const YAML::Node & node, bool allow_unmanageable)
{
  if (!node || node.IsNull()) {
    return true;
  }
  return allow_unmanageable && node.IsScalar() && node.Scalar() == k_unmanageable;
}

bool is_unmanageable_sentinel(const YAML::Node & node)
{
  return node && node.IsScalar() && node.Scalar() == k_unmanageable;
}

constexpr int k_nice_min = -20;
constexpr int k_nice_max = 19;
constexpr int k_rt_priority_min = 1;
constexpr int k_rt_priority_max = 99;

// 'nice' is required for the CFS policies (SCHED_OTHER/BATCH/IDLE);
// parse_rt_priority is the mirror image for SCHED_FIFO/SCHED_RR. `entry_desc`
// is the "id=..."/"name=..." fragment used in messages.
int parse_nice(
  const YAML::Node & entry, const std::string & policy, const std::string & entry_desc,
  bool allow_unmanageable)
{
  const YAML::Node nice = entry["nice"];
  if (is_unset(nice, allow_unmanageable)) {
    throw std::runtime_error(
      "Policy '" + policy + "' requires 'nice' for " + entry_desc +
      (is_unmanageable_sentinel(nice) ? " (UNMANAGEABLE counts as unset)" : ""));
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
  const YAML::Node & entry, const std::string & policy, const std::string & entry_desc,
  bool allow_unmanageable)
{
  const YAML::Node priority = entry["priority"];
  if (is_unset(priority, allow_unmanageable)) {
    throw std::runtime_error(
      "Policy '" + policy + "' requires 'priority' for " + entry_desc +
      (is_unmanageable_sentinel(priority) ? " (UNMANAGEABLE counts as unset)" : ""));
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

// yaml-cpp's as<T>() auto-detects the numeric base (YAML 1.1), so a
// zero-padded "010" would parse as octal 8 and "0x10" as hex 16. IRQ numbers
// and SCHED_DEADLINE parameters are hand-aligned columns where zero-padding
// is plausible, so accept digits only and parse base-10, matching the
// is_all_digits() + from_chars() path the system scanners use.
template <typename T>
std::optional<T> as_base10(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const std::string & s = node.Scalar();
  if (s.empty() || !std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
    return std::nullopt;
  }
  T value{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc() || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

unsigned int parse_deadline_field(
  const YAML::Node & entry, const char * key, const std::string & entry_desc)
{
  const auto value = as_base10<unsigned int>(entry[key]);
  if (!value) {
    throw std::runtime_error(
      "'" + std::string(key) + "' must be a non-negative decimal integer for " + entry_desc);
  }
  return *value;
}

// CPU_SET(3) and sched_setaffinity(2) silently drop out-of-range or
// nonexistent CPUs instead of failing, so reject them at parse time. The
// machine CPU count is a valid bound because the config is always parsed on
// the machine that applies it. The result is sorted and deduplicated so
// downstream consumers see a canonical form.
std::vector<int> parse_affinity(
  const YAML::Node & entry, const std::string & entry_desc, bool allow_unmanageable)
{
  const YAML::Node affinity = entry["affinity"];
  std::vector<int> cpus;
  // Unset (absent, null, or the allowed UNMANAGEABLE sentinel) means
  // "do not manage affinity".
  if (is_unset(affinity, allow_unmanageable)) {
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

bool is_cfs_policy(const std::string & policy)
{
  return policy == "SCHED_OTHER" || policy == "SCHED_BATCH" || policy == "SCHED_IDLE";
}

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

bool is_kworker_comm(const std::string & comm)
{
  constexpr std::string_view kworker_prefix = "kworker/";
  return comm.compare(0, kworker_prefix.size(), kworker_prefix) == 0;
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
    cfg.affinity = parse_affinity(cg, "id=" + cfg.thread_str, /*allow_unmanageable=*/false);
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
      cfg.nice = parse_nice(cg, cfg.policy, "id=" + cfg.thread_str, /*allow_unmanageable=*/false);
    } else {
      cfg.priority =
        parse_rt_priority(cg, cfg.policy, "id=" + cfg.thread_str, /*allow_unmanageable=*/false);
    }
  }

  for (size_t i = 0; i < non_ros_threads.size(); ++i) {
    const auto & nrt = non_ros_threads[i];
    auto & cfg = non_ros_threads_out[i];

    cfg.thread_str = nrt["name"].as<std::string>();
    cfg.affinity = parse_affinity(nrt, "name=" + cfg.thread_str, /*allow_unmanageable=*/false);
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
      cfg.nice =
        parse_nice(nrt, cfg.policy, "name=" + cfg.thread_str, /*allow_unmanageable=*/false);
    } else {
      cfg.priority =
        parse_rt_priority(nrt, cfg.policy, "name=" + cfg.thread_str, /*allow_unmanageable=*/false);
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

bool KernelThreadConfig::is_managed() const noexcept
{
  return policy.has_value() || !affinity.empty();
}

bool IrqConfig::is_managed() const noexcept
{
  return !affinity.empty();
}

std::vector<KernelThreadConfig> parse_kernel_threads(const YAML::Node & yaml)
{
  YAML::Node section = yaml["kernel_threads"];
  std::vector<KernelThreadConfig> result;
  if (!section || section.IsNull()) {
    return result;
  }
  if (!section.IsSequence()) {
    throw std::runtime_error("'kernel_threads' must be a list");
  }
  result.resize(section.size());

  for (size_t i = 0; i < section.size(); ++i) {
    const auto & kt = section[i];
    auto & cfg = result[i];
    const std::string entry_pos = "kernel_threads entry #" + std::to_string(i);

    if (!kt.IsMap()) {
      throw std::runtime_error(entry_pos + " must be a mapping (e.g. '- comm: ...')");
    }
    if (!kt["comm"] || kt["comm"].IsNull()) {
      throw std::runtime_error(entry_pos + " is missing a non-empty 'comm'");
    }
    try {
      cfg.comm = kt["comm"].as<std::string>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error(entry_pos + ": 'comm' must be a string");
    }
    if (cfg.comm.empty()) {
      throw std::runtime_error(entry_pos + " is missing a non-empty 'comm'");
    }
    if (is_kworker_comm(cfg.comm)) {
      throw std::runtime_error(
        "kernel_threads entry '" + cfg.comm +
        "' is not manageable: kworker comms are ephemeral and mutate at runtime, so they cannot "
        "be matched reliably");
    }
    cfg.affinity = parse_affinity(kt, "comm=" + cfg.comm, /*allow_unmanageable=*/true);

    if (is_unset(kt["policy"], /*allow_unmanageable=*/true)) {
      // Any policy-dependent field without 'policy' would otherwise be
      // silently dead configuration (is_managed() == false).
      for (const char * key : {"nice", "priority", "runtime", "period", "deadline"}) {
        if (!is_unset(kt[key], /*allow_unmanageable=*/true)) {
          throw std::runtime_error(
            "'" + std::string(key) + "' requires 'policy' for comm=" + cfg.comm +
            ": set both or leave both unset");
        }
      }
      continue;
    }

    try {
      cfg.policy = kt["policy"].as<std::string>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error("'policy' must be a string for comm=" + cfg.comm);
    }
    if (policy_to_sched_const.count(*cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + *cfg.policy + "' for comm=" + cfg.comm +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (*cfg.policy == "SCHED_DEADLINE") {
      // Explicit check for a clear message: these fields are always
      // hand-written (prerun never emits DEADLINE) and easy to forget.
      if (
        is_unset(kt["runtime"], /*allow_unmanageable=*/true) ||
        is_unset(kt["period"], /*allow_unmanageable=*/true) ||
        is_unset(kt["deadline"], /*allow_unmanageable=*/true)) {
        throw std::runtime_error(
          "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for comm=" + cfg.comm);
      }
      cfg.runtime = parse_deadline_field(kt, "runtime", "comm=" + cfg.comm);
      cfg.period = parse_deadline_field(kt, "period", "comm=" + cfg.comm);
      cfg.deadline = parse_deadline_field(kt, "deadline", "comm=" + cfg.comm);
    } else if (is_cfs_policy(*cfg.policy)) {
      cfg.nice = parse_nice(kt, *cfg.policy, "comm=" + cfg.comm, /*allow_unmanageable=*/true);
    } else {
      cfg.priority =
        parse_rt_priority(kt, *cfg.policy, "comm=" + cfg.comm, /*allow_unmanageable=*/true);
    }
  }

  std::unordered_set<std::string> seen;
  for (const auto & c : result) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen.insert(c.comm).second) {
      throw std::runtime_error("Duplicate kernel_thread entry: comm=" + c.comm);
    }
  }
  return result;
}

std::vector<IrqConfig> parse_irqs(const YAML::Node & yaml)
{
  YAML::Node section = yaml["irqs"];
  std::vector<IrqConfig> result;
  if (!section || section.IsNull()) {
    return result;
  }
  if (!section.IsSequence()) {
    throw std::runtime_error("'irqs' must be a list");
  }
  result.resize(section.size());

  for (size_t i = 0; i < section.size(); ++i) {
    const auto & iq = section[i];
    auto & cfg = result[i];
    const std::string entry_pos = "irqs entry #" + std::to_string(i);

    if (!iq.IsMap()) {
      throw std::runtime_error(entry_pos + " must be a mapping (e.g. '- irq: ...')");
    }
    if (!iq["irq"] || iq["irq"].IsNull()) {
      throw std::runtime_error(entry_pos + " is missing a non-negative integer 'irq'");
    }
    const auto irq = as_base10<int>(iq["irq"]);
    if (!irq) {
      throw std::runtime_error(
        entry_pos + ": 'irq' must be a non-negative decimal integer, got '" +
        (iq["irq"].IsScalar() ? iq["irq"].Scalar() : std::string("<non-scalar>")) + "'");
    }
    cfg.irq = *irq;

    if (iq["name"] && !iq["name"].IsNull()) {
      try {
        cfg.name = iq["name"].as<std::string>();
      } catch (const YAML::Exception &) {
        throw std::runtime_error("'name' must be a string for irq=" + std::to_string(cfg.irq));
      }
    }
    cfg.affinity =
      parse_affinity(iq, "irq=" + std::to_string(cfg.irq), /*allow_unmanageable=*/true);
  }

  std::unordered_set<int> seen;
  for (const auto & c : result) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen.insert(c.irq).second) {
      throw std::runtime_error("Duplicate irq entry: irq=" + std::to_string(c.irq));
    }
  }
  return result;
}

}  // namespace agnocast_cie_thread_configurator
