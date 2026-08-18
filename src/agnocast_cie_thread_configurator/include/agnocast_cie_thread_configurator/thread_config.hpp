#pragma once

#include "yaml-cpp/yaml.h"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// A single thread's scheduling configuration as parsed from the YAML and
// observed at runtime. Owned by ThreadConfiguratorNode in two vectors.
struct ThreadConfig
{
  std::string thread_str;  // callback_group_id or thread_name
  size_t domain_id = 0;
  int64_t thread_id = -1;  // -1 until announced by the target application
  std::vector<int> affinity;
  std::string policy;
  int nice = 0;      // SCHED_OTHER/BATCH/IDLE only (-20..19)
  int priority = 0;  // rt_priority; SCHED_FIFO/RR only (1..99)

  // SCHED_DEADLINE only
  unsigned int runtime = 0;
  unsigned int period = 0;
  unsigned int deadline = 0;

  // Full incoming callback_group_id -> last announced tid; wildcard
  // ("<node name>/*") entries only. For such entries `applied` means "at least
  // one matched instance has been configured" and thread_id stays -1. std::map
  // for deterministic iteration order in the reapply response arrays.
  std::map<std::string, int64_t> matched_tids;

  bool applied = false;  // true once issue_syscalls() has succeeded

  bool is_wildcard() const noexcept;
  // thread_str minus the trailing "/*"; only meaningful when is_wildcard().
  std::string wildcard_prefix() const;
};

// Node-name part of an incoming callback_group_id: the substring before the
// first '@' (the whole string when no '@' is present).
std::string extract_node_part(const std::string & callback_group_id);

// Sentinel for kernel_threads/irqs attribute values: the kernel or this tool
// cannot manage the attribute (fixed per-CPU affinity, a policy with no YAML
// representation), whereas YAML null means the USER chose not to. Both parse
// to "not applied". Case-sensitive, exact match.
inline constexpr std::string_view k_unmanageable = "UNMANAGEABLE";

// Desired attributes for every kernel thread whose comm matches at apply
// time (comms are not unique, e.g. an nfsd pool). Unset fields (YAML null,
// absent key, or UNMANAGEABLE) are never applied.
struct KernelThreadConfig
{
  std::string comm;
  std::optional<std::string> policy;
  int priority = 0;           // meaningful only when policy is set: nice or rt_priority
  std::vector<int> affinity;  // empty = leave alone

  // SCHED_DEADLINE only
  unsigned int runtime = 0;
  unsigned int period = 0;
  unsigned int deadline = 0;

  bool is_managed() const noexcept;
};

// Desired affinity for one IRQ, a hard IRQ's only schedulable attribute
// (threaded-IRQ handler threads are plain kernel threads).
struct IrqConfig
{
  int irq = -1;
  // Expected /sys/kernel/irq/<irq>/actions content, verified before applying
  // (guards against IRQ renumbering across boots). Empty = skip the check.
  std::string name;
  std::vector<int> affinity;  // empty = leave alone

  bool is_managed() const noexcept;
};

// Parse the optional kernel_threads / irqs sections. Missing/null sections
// yield empty vectors (pre-existing YAMLs keep working). Throws
// std::runtime_error on validation error.
std::vector<KernelThreadConfig> parse_kernel_threads(const YAML::Node & yaml);
std::vector<IrqConfig> parse_irqs(const YAML::Node & yaml);

// Mapping from the policy string in the YAML to the kernel SCHED_* constant.
// Defined in thread_config.cpp; both the parser and issue_syscalls() use it.
extern const std::unordered_map<std::string, int> policy_to_sched_const;

// Parse the given YAML document and populate the two output vectors.
// Throws std::runtime_error on per-entry validation error. Output ThreadConfigs
// have thread_id=-1 and applied=false; callers that re-parse must carry
// thread_id over from their existing index manually.
// A callback-group id ending in "/*" is a wildcard entry matching every
// callback group whose node part (before the first '@') equals the prefix,
// within the same domain; exact entries take precedence over wildcards.
// non_ros_threads names are always matched exactly.
// hardware_info / rt_throttling are validated only at startup, not here, and
// the kernel_threads / irqs sections have their own parsers (see above).
void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out);

}  // namespace agnocast_cie_thread_configurator
