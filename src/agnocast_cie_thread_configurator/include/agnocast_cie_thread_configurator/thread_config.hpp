#pragma once

#include "agnocast_cie_thread_configurator/sched_policy.hpp"
#include "yaml-cpp/yaml.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// The desired state of one schedulable entity as written in the YAML. What
// is observed at runtime (announced tids, applied flags) lives with the
// owner, not here, so re-parsing yields a plain value to swap in.

struct DeadlineParams
{
  uint64_t runtime = 0;  // nsec
  uint64_t period = 0;
  uint64_t deadline = 0;
};

// Only the knobs of the policy's class are meaningful, namely nice for
// OTHER/BATCH/IDLE, rt_priority for FIFO/RR and deadline for DEADLINE; the
// parser leaves the others at their defaults. affinity is independent of the
// policy.
struct SchedAttrs
{
  std::optional<SchedPolicy> policy;  // nullopt only for an affinity-only kernel_threads entry
  int nice = 0;                       // -20..19
  int rt_priority = 0;                // 1..99
  DeadlineParams deadline;
  std::vector<int> affinity;  // sorted, deduplicated; empty = do not manage
};

// A callback-group id ending in "/*" is a wildcard matching every callback
// group whose node part (before the first '@') equals the prefix, within the
// same domain; exact entries take precedence over wildcards.
struct CallbackGroupEntry
{
  size_t domain_id = 0;
  std::string id;
  SchedAttrs attrs;

  bool is_wildcard() const noexcept;
  // id minus the trailing "/*"; only meaningful when is_wildcard().
  std::string wildcard_prefix() const;
};

// Names are opaque and matched exactly (no wildcards).
struct NonRosThreadEntry
{
  std::string name;
  SchedAttrs attrs;
};

// Applies to every kernel thread whose comm matches at apply time (comms are
// not unique, e.g. an nfsd pool). Unset attributes are never applied.
struct KernelThreadEntry
{
  std::string comm;
  SchedAttrs attrs;

  bool is_managed() const noexcept;
};

// Affinity is a hard IRQ's only schedulable attribute (threaded-IRQ handler
// threads are plain kernel threads).
struct IrqEntry
{
  int irq = -1;
  // Expected /sys/kernel/irq/<irq>/actions content, verified before applying
  // (guards against IRQ renumbering across boots). Empty = skip the check.
  std::string name;
  std::vector<int> affinity;  // empty = leave alone

  bool is_managed() const noexcept;
};

struct ParsedConfig
{
  std::vector<CallbackGroupEntry> callback_groups;
  std::vector<NonRosThreadEntry> non_ros_threads;
  std::vector<KernelThreadEntry> kernel_threads;
  std::vector<IrqEntry> irqs;
};

// Node-name part of an incoming callback_group_id: the substring before the
// first '@' (the whole string when no '@' is present).
std::string extract_node_part(const std::string & callback_group_id);

// Sentinel for kernel_threads/irqs attribute values: the kernel or this tool
// cannot manage the attribute (fixed per-CPU affinity, a policy with no YAML
// representation), whereas YAML null means the USER chose not to. Both parse
// to "not applied". Case-sensitive, exact match.
inline constexpr std::string_view k_unmanageable = "UNMANAGEABLE";

// Parse the four entry sections. A missing or null section yields an empty
// vector. Every section shares one attribute parser, so a given entry body
// is validated identically wherever it appears; the differences are the
// entry key (id / name / comm / irq), that callback_groups and
// non_ros_threads require 'policy' and reject UNMANAGEABLE, and that
// callback_groups take an optional 'domain_id' (default_domain_id otherwise).
// Integer fields are decimal only ("010" is ten, "0x10" is rejected).
// Throws std::runtime_error naming the entry and key on validation error.
// hardware_info / rt_throttling are validated only at startup, not here.
ParsedConfig parse_config(const YAML::Node & yaml, size_t default_domain_id);

}  // namespace agnocast_cie_thread_configurator
