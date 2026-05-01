#pragma once

#include "yaml-cpp/yaml.h"

#include <cstdint>
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

  bool applied = false;  // true once issue_syscalls() has succeeded
};

// Mapping from the policy string in the YAML to the kernel SCHED_* constant.
// Defined in thread_config.cpp; both the parser and issue_syscalls() use it.
extern const std::unordered_map<std::string, int> policy_to_sched_const;

// Parse the given YAML document and populate the two output vectors.
// Throws std::runtime_error on per-entry validation error (unknown policy,
// missing SCHED_DEADLINE fields, etc.).
//
// Output ThreadConfigs have thread_id=-1 and applied=false.
//
// hardware_info / rt_throttling sections are NOT processed here; they are validated
// at startup only by ThreadConfiguratorNode's constructor.
void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out);

}  // namespace agnocast_cie_thread_configurator
