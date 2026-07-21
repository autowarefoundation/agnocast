#pragma once

#include "yaml-cpp/yaml.h"

#include <cstdint>
#include <map>
#include <string>
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
  int priority = 0;

  // SCHED_DEADLINE only
  unsigned int runtime = 0;
  unsigned int period = 0;
  unsigned int deadline = 0;

  // Full incoming callback_group_id -> last announced tid. Wildcard
  // ("<node name>/*") callback-group entries only; unused by non_ros_threads.
  // For a wildcard entry, `applied` means "at least one matched instance has
  // been configured" and thread_id stays -1. std::map for deterministic
  // iteration order in the reapply response arrays.
  std::map<std::string, int64_t> matched_tids;

  bool applied = false;  // true once issue_syscalls() has succeeded

  // Whether this is a wildcard callback-group entry.
  bool is_wildcard() const;
  // The "<node name>" part of a wildcard id: thread_str minus the trailing
  // "/*". Only meaningful when is_wildcard().
  std::string wildcard_prefix() const;
};

// Node-name part of an incoming callback_group_id: the substring before the
// first '@' (the whole string when no '@' is present).
std::string extract_node_part(const std::string & callback_group_id);

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
// hardware_info / rt_throttling are validated only at startup, not here.
void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out);

}  // namespace agnocast_cie_thread_configurator
