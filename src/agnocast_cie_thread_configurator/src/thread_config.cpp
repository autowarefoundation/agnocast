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
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace agnocast_cie_thread_configurator
{

namespace
{

struct EntryContext
{
  std::string desc;  // "id=..." / "name=..." / "comm=..." fragment for messages
  // kernel_threads/irqs are emitted from a scan, so UNMANAGEABLE counts as
  // unset and 'policy' may be absent. The hand-written callback_groups /
  // non_ros_threads require 'policy' and treat the sentinel as an ordinary
  // invalid value.
  bool scanned = false;
};

// Unset = the attribute must not be applied: YAML null / absent key (the
// user's opt-out) or, when `allow_unmanageable` is set, the UNMANAGEABLE
// sentinel (a kernel/tool constraint).
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

std::string scalar_or_placeholder(const YAML::Node & node)
{
  return node.IsScalar() ? node.Scalar() : std::string("<non-scalar>");
}

std::string entry_pos(const char * section, size_t index)
{
  return std::string(section) + " entry #" + std::to_string(index);
}

constexpr int k_nice_min = -20;
constexpr int k_nice_max = 19;
constexpr int k_rt_priority_min = 1;
constexpr int k_rt_priority_max = 99;

// Decimal digits only, with a leading '-' for signed T. yaml-cpp's as<T>()
// auto-detects the base (YAML 1.1), so a zero-padded "010" would parse as
// octal 8 and "0x10" as hex 16; IRQ numbers and SCHED_DEADLINE parameters are
// hand-aligned columns where zero-padding is plausible. This matches the
// is_all_digits() + from_chars() path the system scanners use.
template <typename T>
std::optional<T> as_decimal(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const std::string & s = node.Scalar();
  const size_t digits_begin = (std::is_signed_v<T> && !s.empty() && s[0] == '-') ? 1 : 0;
  if (
    s.size() == digits_begin || !std::all_of(
                                  s.begin() + static_cast<std::ptrdiff_t>(digits_begin), s.end(),
                                  [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return std::nullopt;
  }
  T value{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc() || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

// Missing or null = no entries (pre-existing YAMLs keep working). Anything
// but a list of mappings is rejected, since a scalar would iterate zero times
// and silently drop the section, and a list of bare keys would fail with a
// raw yaml-cpp BadSubscript instead of the entry diagnostic.
YAML::Node section_entries(const YAML::Node & yaml, const char * name, const char * example_key)
{
  const YAML::Node section = yaml[name];
  if (!section || section.IsNull()) {
    return YAML::Node(YAML::NodeType::Sequence);
  }
  if (!section.IsSequence()) {
    throw std::runtime_error("'" + std::string(name) + "' must be a list");
  }
  for (size_t i = 0; i < section.size(); ++i) {
    if (!section[i].IsMap()) {
      throw std::runtime_error(
        entry_pos(name, i) + " must be a mapping (e.g. '- " + example_key + ": ...')");
    }
  }
  return section;
}

std::string required_string(
  const YAML::Node & entry, const char * key, const std::string & entry_position)
{
  const YAML::Node node = entry[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(entry_position + " is missing a non-empty '" + key + "'");
  }
  if (!node.IsScalar()) {
    throw std::runtime_error(entry_position + ": '" + key + "' must be a string");
  }
  if (node.Scalar().empty()) {
    throw std::runtime_error(entry_position + " is missing a non-empty '" + key + "'");
  }
  return node.Scalar();
}

// The policy's tunable ('nice' for CFS, 'priority' for FIFO/RR) is mandatory
// once 'policy' is set. setpriority(2) would silently clamp an out-of-range
// nice to [-20, 19], so both ranges are enforced here and a misunderstanding
// of the scale (e.g. an rt_priority-style 50 as nice) fails loudly.
int parse_tunable(
  const YAML::Node & entry, const char * key, int min, int max, SchedPolicy policy,
  const EntryContext & ctx)
{
  const YAML::Node node = entry[key];
  if (is_unset(node, ctx.scanned)) {
    throw std::runtime_error(
      "Policy '" + std::string(to_string(policy)) + "' requires '" + key + "' for " + ctx.desc +
      (is_unmanageable_sentinel(node) ? " (UNMANAGEABLE counts as unset)" : ""));
  }
  const auto value = as_decimal<int>(node);
  if (!value) {
    throw std::runtime_error(
      "'" + std::string(key) + "' must be a decimal integer for " + ctx.desc + ", got '" +
      scalar_or_placeholder(node) + "'");
  }
  if (*value < min || *value > max) {
    throw std::runtime_error(
      "'" + std::string(key) + "' must be in [" + std::to_string(min) + ", " + std::to_string(max) +
      "] for " + ctx.desc + ", got " + std::to_string(*value));
  }
  return *value;
}

DeadlineParams parse_deadline_params(const YAML::Node & entry, const EntryContext & ctx)
{
  // Explicit check for a clear message: these fields are always hand-written
  // (prerun never emits DEADLINE) and easy to forget.
  for (const char * key : {"runtime", "period", "deadline"}) {
    // cppcheck-suppress useStlAlgorithm
    if (is_unset(entry[key], ctx.scanned)) {
      throw std::runtime_error(
        "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for " + ctx.desc);
    }
  }
  const auto field = [&](const char * key) {
    const auto value = as_decimal<uint64_t>(entry[key]);
    if (!value) {
      throw std::runtime_error(
        "'" + std::string(key) + "' must be a non-negative decimal integer for " + ctx.desc);
    }
    return *value;
  };
  return DeadlineParams{field("runtime"), field("period"), field("deadline")};
}

// CPU_SET(3) and sched_setaffinity(2) silently drop out-of-range or
// nonexistent CPUs instead of failing, so reject them at parse time. The
// machine CPU count is a valid bound because the config is always parsed on
// the machine that applies it. The result is sorted and deduplicated so
// downstream consumers see a canonical form.
std::vector<int> parse_affinity(const YAML::Node & entry, const EntryContext & ctx)
{
  const YAML::Node affinity = entry["affinity"];
  std::vector<int> cpus;
  if (is_unset(affinity, ctx.scanned)) {
    return cpus;
  }
  if (!affinity.IsSequence()) {
    throw std::runtime_error(
      "'affinity' must be a list of CPU numbers (e.g. [2, 3]) for " + ctx.desc);
  }
  const int max_cpu = manageable_cpu_bound();
  for (const auto & cpu_node : affinity) {
    const auto cpu = as_decimal<int>(cpu_node);
    if (!cpu) {
      throw std::runtime_error(
        "'affinity' must contain only decimal integers for " + ctx.desc + ", got '" +
        scalar_or_placeholder(cpu_node) + "'");
    }
    if (*cpu < 0 || *cpu > max_cpu) {
      throw std::runtime_error(
        "'affinity' CPU " + std::to_string(*cpu) + " must be in [0, " + std::to_string(max_cpu) +
        "] (this machine has " + std::to_string(sysconf(_SC_NPROCESSORS_CONF)) + " CPUs) for " +
        ctx.desc);
    }
    cpus.push_back(*cpu);
  }
  std::sort(cpus.begin(), cpus.end());
  cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
  return cpus;
}

// The one attribute parser behind every section, so an entry body is
// validated the same way wherever it appears.
SchedAttrs parse_sched_attrs(const YAML::Node & entry, const EntryContext & ctx)
{
  SchedAttrs attrs;
  attrs.affinity = parse_affinity(entry, ctx);

  const YAML::Node policy_node = entry["policy"];
  if (is_unset(policy_node, ctx.scanned)) {
    if (!ctx.scanned) {
      throw std::runtime_error("'policy' is required for " + ctx.desc);
    }
    // Any policy-dependent field without 'policy' would otherwise be
    // silently dead configuration (is_managed() == false).
    for (const char * key : {"nice", "priority", "runtime", "period", "deadline"}) {
      // cppcheck-suppress useStlAlgorithm
      if (!is_unset(entry[key], ctx.scanned)) {
        throw std::runtime_error(
          "'" + std::string(key) + "' requires 'policy' for " + ctx.desc +
          ": set both or leave both unset");
      }
    }
    return attrs;
  }
  if (!policy_node.IsScalar()) {
    throw std::runtime_error("'policy' must be a string for " + ctx.desc);
  }
  const auto policy = parse_sched_policy(policy_node.Scalar());
  if (!policy) {
    throw std::runtime_error(
      "Unknown scheduling policy '" + policy_node.Scalar() + "' for " + ctx.desc +
      ". Valid policies: " + sched_policy_names());
  }
  attrs.policy = policy;

  if (*policy == SchedPolicy::Deadline) {
    attrs.deadline = parse_deadline_params(entry, ctx);
  } else if (is_cfs(*policy)) {
    attrs.nice = parse_tunable(entry, "nice", k_nice_min, k_nice_max, *policy, ctx);
  } else {
    attrs.rt_priority =
      parse_tunable(entry, "priority", k_rt_priority_min, k_rt_priority_max, *policy, ctx);
  }
  return attrs;
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

  const YAML::Node callback_groups = section_entries(yaml, "callback_groups", "id");
  for (size_t i = 0; i < callback_groups.size(); ++i) {
    const YAML::Node cg = callback_groups[i];
    CallbackGroupEntry entry;
    entry.id = required_string(cg, "id", entry_pos("callback_groups", i));
    validate_callback_group_id(entry);
    entry.domain_id = default_domain_id;
    if (!is_unset(cg["domain_id"], /*allow_unmanageable=*/false)) {
      const auto domain_id = as_decimal<size_t>(cg["domain_id"]);
      if (!domain_id) {
        throw std::runtime_error(
          "'domain_id' must be a non-negative decimal integer for id=" + entry.id + ", got '" +
          scalar_or_placeholder(cg["domain_id"]) + "'");
      }
      entry.domain_id = *domain_id;
    }
    entry.attrs = parse_sched_attrs(cg, EntryContext{"id=" + entry.id, /*scanned=*/false});
    config.callback_groups.push_back(std::move(entry));
  }
  reject_duplicates(config.callback_groups, "callback_group", [](const CallbackGroupEntry & e) {
    return "domain_id=" + std::to_string(e.domain_id) + ", id=" + e.id;
  });

  const YAML::Node non_ros_threads = section_entries(yaml, "non_ros_threads", "name");
  for (size_t i = 0; i < non_ros_threads.size(); ++i) {
    const YAML::Node nrt = non_ros_threads[i];
    NonRosThreadEntry entry;
    entry.name = required_string(nrt, "name", entry_pos("non_ros_threads", i));
    entry.attrs = parse_sched_attrs(nrt, EntryContext{"name=" + entry.name, /*scanned=*/false});
    config.non_ros_threads.push_back(std::move(entry));
  }
  reject_duplicates(config.non_ros_threads, "non_ros_thread", [](const NonRosThreadEntry & e) {
    return "name=" + e.name;
  });

  const YAML::Node kernel_threads = section_entries(yaml, "kernel_threads", "comm");
  for (size_t i = 0; i < kernel_threads.size(); ++i) {
    const YAML::Node kt = kernel_threads[i];
    KernelThreadEntry entry;
    entry.comm = required_string(kt, "comm", entry_pos("kernel_threads", i));
    if (is_kworker_comm(entry.comm)) {
      throw std::runtime_error(
        "kernel_threads entry '" + entry.comm +
        "' is not manageable: kworker comms are ephemeral and mutate at runtime, so they cannot "
        "be matched reliably");
    }
    entry.attrs = parse_sched_attrs(kt, EntryContext{"comm=" + entry.comm, /*scanned=*/true});
    config.kernel_threads.push_back(std::move(entry));
  }
  reject_duplicates(config.kernel_threads, "kernel_thread", [](const KernelThreadEntry & e) {
    return "comm=" + e.comm;
  });

  const YAML::Node irqs = section_entries(yaml, "irqs", "irq");
  for (size_t i = 0; i < irqs.size(); ++i) {
    const YAML::Node iq = irqs[i];
    IrqEntry entry;
    const YAML::Node irq_node = iq["irq"];
    if (!irq_node || irq_node.IsNull()) {
      throw std::runtime_error(entry_pos("irqs", i) + " is missing a non-negative integer 'irq'");
    }
    const auto irq = as_decimal<int>(irq_node);
    if (!irq || *irq < 0) {
      throw std::runtime_error(
        entry_pos("irqs", i) + ": 'irq' must be a non-negative decimal integer, got '" +
        scalar_or_placeholder(irq_node) + "'");
    }
    entry.irq = *irq;
    const std::string desc = "irq=" + std::to_string(entry.irq);
    if (!is_unset(iq["name"], /*allow_unmanageable=*/false)) {
      if (!iq["name"].IsScalar()) {
        throw std::runtime_error("'name' must be a string for " + desc);
      }
      entry.name = iq["name"].Scalar();
    }
    entry.affinity = parse_affinity(iq, EntryContext{desc, /*scanned=*/true});
    config.irqs.push_back(std::move(entry));
  }
  reject_duplicates(
    config.irqs, "irq", [](const IrqEntry & e) { return "irq=" + std::to_string(e.irq); });

  return config;
}

}  // namespace agnocast_cie_thread_configurator
