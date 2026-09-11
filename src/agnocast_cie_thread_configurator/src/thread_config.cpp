#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include "agnocast_cie_thread_configurator/sched_policy.hpp"
#include "agnocast_cie_thread_configurator/system_scan.hpp"

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
  const YAML::Node & entry, SchedPolicy policy, const std::string & entry_desc,
  bool allow_unmanageable)
{
  const YAML::Node nice = entry["nice"];
  if (is_unset(nice, allow_unmanageable)) {
    throw std::runtime_error(
      "Policy '" + std::string(to_string(policy)) + "' requires 'nice' for " + entry_desc +
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
  const YAML::Node & entry, SchedPolicy policy, const std::string & entry_desc,
  bool allow_unmanageable)
{
  const YAML::Node priority = entry["priority"];
  if (is_unset(priority, allow_unmanageable)) {
    throw std::runtime_error(
      "Policy '" + std::string(to_string(policy)) + "' requires 'priority' for " + entry_desc +
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

uint64_t parse_deadline_field(
  const YAML::Node & entry, const char * key, const std::string & entry_desc)
{
  const auto value = as_base10<uint64_t>(entry[key]);
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
  const int max_cpu = manageable_cpu_bound();
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
        "] (this machine has " + std::to_string(sysconf(_SC_NPROCESSORS_CONF)) + " CPUs) for " +
        entry_desc);
    }
    cpus.push_back(cpu);
  }
  std::sort(cpus.begin(), cpus.end());
  cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
  return cpus;
}

SchedPolicy parse_policy_or_throw(const std::string & policy, const std::string & entry_desc)
{
  const auto parsed = parse_sched_policy(policy);
  if (!parsed) {
    throw std::runtime_error(
      "Unknown scheduling policy '" + policy + "' for " + entry_desc +
      ". Valid policies: " + sched_policy_names());
  }
  return *parsed;
}

// A typo'd pattern silently treated as an exact id would never match, so any
// id containing '*' must be a well-formed "<node name>/*".
void validate_callback_group_id(const CallbackGroupEntry & entry)
{
  if (entry.id.find('*') == std::string::npos) {
    return;
  }
  if (!entry.is_wildcard()) {
    throw std::runtime_error(
      "Invalid id '" + entry.id +
      "': '*' is only allowed as a trailing \"/*\" wildcard (e.g. /my_node/*)");
  }
  const std::string prefix = entry.wildcard_prefix();
  if (prefix.empty() || prefix.find('*') != std::string::npos) {
    throw std::runtime_error(
      "Invalid wildcard id '" + entry.id +
      "': the part before \"/*\" must be a non-empty node name without '*'");
  }
  if (prefix.find('@') != std::string::npos) {
    throw std::runtime_error(
      "Invalid wildcard id '" + entry.id +
      "': the part before \"/*\" must be a plain node name, not a full callback-group id "
      "containing '@'");
  }
}

// Duplicates would otherwise collapse to the last-inserted entry in the
// owner's index, dropping earlier YAML lines without warning. `describe`
// doubles as the identity, so entries with the same description are
// duplicates.
template <typename Entries, typename Describe>
void reject_duplicates(const Entries & entries, const char * what, Describe describe)
{
  std::unordered_set<std::string> seen;
  for (const auto & entry : entries) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen.insert(describe(entry)).second) {
      throw std::runtime_error(std::string("Duplicate ") + what + " entry: " + describe(entry));
    }
  }
}

}  // namespace

bool CallbackGroupEntry::is_wildcard() const noexcept
{
  static constexpr std::string_view suffix = "/*";
  return id.size() >= suffix.size() &&
         id.compare(id.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string CallbackGroupEntry::wildcard_prefix() const
{
  return id.substr(0, id.size() - 2);
}

bool KernelThreadEntry::is_managed() const noexcept
{
  return attrs.policy.has_value() || !attrs.affinity.empty();
}

bool IrqEntry::is_managed() const noexcept
{
  return !affinity.empty();
}

std::string extract_node_part(const std::string & callback_group_id)
{
  return callback_group_id.substr(0, callback_group_id.find('@'));
}

ParsedConfig parse_config(const YAML::Node & yaml, size_t default_domain_id)
{
  ParsedConfig config;

  // A missing or null section has no entries, like kernel_threads / irqs.
  const YAML::Node callback_groups = yaml["callback_groups"];
  const size_t callback_group_count =
    (callback_groups && !callback_groups.IsNull()) ? callback_groups.size() : 0;
  for (size_t i = 0; i < callback_group_count; ++i) {
    const YAML::Node cg = callback_groups[i];
    CallbackGroupEntry entry;
    entry.id = cg["id"].as<std::string>();
    validate_callback_group_id(entry);
    entry.domain_id = cg["domain_id"] ? cg["domain_id"].as<size_t>() : default_domain_id;
    entry.attrs.affinity = parse_affinity(cg, "id=" + entry.id, /*allow_unmanageable=*/false);
    const SchedPolicy policy =
      parse_policy_or_throw(cg["policy"].as<std::string>(), "id=" + entry.id);
    entry.attrs.policy = policy;

    if (policy == SchedPolicy::Deadline) {
      entry.attrs.deadline = DeadlineParams{
        cg["runtime"].as<uint64_t>(), cg["period"].as<uint64_t>(), cg["deadline"].as<uint64_t>()};
    } else if (is_cfs(policy)) {
      entry.attrs.nice = parse_nice(cg, policy, "id=" + entry.id, /*allow_unmanageable=*/false);
    } else {
      entry.attrs.rt_priority =
        parse_rt_priority(cg, policy, "id=" + entry.id, /*allow_unmanageable=*/false);
    }
    config.callback_groups.push_back(std::move(entry));
  }
  reject_duplicates(config.callback_groups, "callback_group", [](const CallbackGroupEntry & e) {
    return "domain_id=" + std::to_string(e.domain_id) + ", id=" + e.id;
  });

  const YAML::Node non_ros_threads = yaml["non_ros_threads"];
  const size_t non_ros_thread_count =
    (non_ros_threads && !non_ros_threads.IsNull()) ? non_ros_threads.size() : 0;
  for (size_t i = 0; i < non_ros_thread_count; ++i) {
    const YAML::Node nrt = non_ros_threads[i];
    NonRosThreadEntry entry;
    entry.name = nrt["name"].as<std::string>();
    entry.attrs.affinity = parse_affinity(nrt, "name=" + entry.name, /*allow_unmanageable=*/false);
    const SchedPolicy policy =
      parse_policy_or_throw(nrt["policy"].as<std::string>(), "name=" + entry.name);
    entry.attrs.policy = policy;

    if (policy == SchedPolicy::Deadline) {
      entry.attrs.deadline = DeadlineParams{
        nrt["runtime"].as<uint64_t>(), nrt["period"].as<uint64_t>(),
        nrt["deadline"].as<uint64_t>()};
    } else if (is_cfs(policy)) {
      entry.attrs.nice =
        parse_nice(nrt, policy, "name=" + entry.name, /*allow_unmanageable=*/false);
    } else {
      entry.attrs.rt_priority =
        parse_rt_priority(nrt, policy, "name=" + entry.name, /*allow_unmanageable=*/false);
    }
    config.non_ros_threads.push_back(std::move(entry));
  }
  reject_duplicates(config.non_ros_threads, "non_ros_thread", [](const NonRosThreadEntry & e) {
    return "name=" + e.name;
  });

  const YAML::Node kernel_threads = yaml["kernel_threads"];
  if (kernel_threads && !kernel_threads.IsNull()) {
    if (!kernel_threads.IsSequence()) {
      throw std::runtime_error("'kernel_threads' must be a list");
    }
    for (size_t i = 0; i < kernel_threads.size(); ++i) {
      const YAML::Node kt = kernel_threads[i];
      KernelThreadEntry entry;
      const std::string entry_pos = "kernel_threads entry #" + std::to_string(i);

      if (!kt.IsMap()) {
        throw std::runtime_error(entry_pos + " must be a mapping (e.g. '- comm: ...')");
      }
      if (!kt["comm"] || kt["comm"].IsNull()) {
        throw std::runtime_error(entry_pos + " is missing a non-empty 'comm'");
      }
      try {
        entry.comm = kt["comm"].as<std::string>();
      } catch (const YAML::Exception &) {
        throw std::runtime_error(entry_pos + ": 'comm' must be a string");
      }
      if (entry.comm.empty()) {
        throw std::runtime_error(entry_pos + " is missing a non-empty 'comm'");
      }
      if (is_kworker_comm(entry.comm)) {
        throw std::runtime_error(
          "kernel_threads entry '" + entry.comm +
          "' is not manageable: kworker comms are ephemeral and mutate at runtime, so they cannot "
          "be matched reliably");
      }
      entry.attrs.affinity = parse_affinity(kt, "comm=" + entry.comm, /*allow_unmanageable=*/true);

      if (is_unset(kt["policy"], /*allow_unmanageable=*/true)) {
        // Any policy-dependent field without 'policy' would otherwise be
        // silently dead configuration (is_managed() == false).
        for (const char * key : {"nice", "priority", "runtime", "period", "deadline"}) {
          if (!is_unset(kt[key], /*allow_unmanageable=*/true)) {
            throw std::runtime_error(
              "'" + std::string(key) + "' requires 'policy' for comm=" + entry.comm +
              ": set both or leave both unset");
          }
        }
        config.kernel_threads.push_back(std::move(entry));
        continue;
      }

      std::string policy_str;
      try {
        policy_str = kt["policy"].as<std::string>();
      } catch (const YAML::Exception &) {
        throw std::runtime_error("'policy' must be a string for comm=" + entry.comm);
      }
      const SchedPolicy policy = parse_policy_or_throw(policy_str, "comm=" + entry.comm);
      entry.attrs.policy = policy;

      if (policy == SchedPolicy::Deadline) {
        // Explicit check for a clear message: these fields are always
        // hand-written (prerun never emits DEADLINE) and easy to forget.
        if (
          is_unset(kt["runtime"], /*allow_unmanageable=*/true) ||
          is_unset(kt["period"], /*allow_unmanageable=*/true) ||
          is_unset(kt["deadline"], /*allow_unmanageable=*/true)) {
          throw std::runtime_error(
            "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for comm=" + entry.comm);
        }
        entry.attrs.deadline = DeadlineParams{
          parse_deadline_field(kt, "runtime", "comm=" + entry.comm),
          parse_deadline_field(kt, "period", "comm=" + entry.comm),
          parse_deadline_field(kt, "deadline", "comm=" + entry.comm)};
      } else if (is_cfs(policy)) {
        entry.attrs.nice =
          parse_nice(kt, policy, "comm=" + entry.comm, /*allow_unmanageable=*/true);
      } else {
        entry.attrs.rt_priority =
          parse_rt_priority(kt, policy, "comm=" + entry.comm, /*allow_unmanageable=*/true);
      }
      config.kernel_threads.push_back(std::move(entry));
    }
  }
  reject_duplicates(config.kernel_threads, "kernel_thread", [](const KernelThreadEntry & e) {
    return "comm=" + e.comm;
  });

  const YAML::Node irqs = yaml["irqs"];
  if (irqs && !irqs.IsNull()) {
    if (!irqs.IsSequence()) {
      throw std::runtime_error("'irqs' must be a list");
    }
    for (size_t i = 0; i < irqs.size(); ++i) {
      const YAML::Node iq = irqs[i];
      IrqEntry entry;
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
      entry.irq = *irq;

      if (iq["name"] && !iq["name"].IsNull()) {
        try {
          entry.name = iq["name"].as<std::string>();
        } catch (const YAML::Exception &) {
          throw std::runtime_error("'name' must be a string for irq=" + std::to_string(entry.irq));
        }
      }
      entry.affinity =
        parse_affinity(iq, "irq=" + std::to_string(entry.irq), /*allow_unmanageable=*/true);
      config.irqs.push_back(std::move(entry));
    }
  }
  reject_duplicates(
    config.irqs, "irq", [](const IrqEntry & e) { return "irq=" + std::to_string(e.irq); });

  return config;
}

}  // namespace agnocast_cie_thread_configurator
