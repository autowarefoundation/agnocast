#include "agnocast_cie_thread_configurator/sched_policy.hpp"

#include <linux/sched.h>
#include <sched.h>

#include <algorithm>
#include <array>
#include <cstddef>

namespace agnocast_cie_thread_configurator
{

namespace
{

struct PolicyEntry
{
  SchedPolicy policy;
  std::string_view name;
  int kernel_policy;
};

// Indexed by the enumerator's underlying value (checked below).
constexpr std::array<PolicyEntry, 6> k_policies{{
  {SchedPolicy::Other, "SCHED_OTHER", SCHED_OTHER},
  {SchedPolicy::Batch, "SCHED_BATCH", SCHED_BATCH},
  {SchedPolicy::Idle, "SCHED_IDLE", SCHED_IDLE},
  {SchedPolicy::Fifo, "SCHED_FIFO", SCHED_FIFO},
  {SchedPolicy::Rr, "SCHED_RR", SCHED_RR},
  {SchedPolicy::Deadline, "SCHED_DEADLINE", SCHED_DEADLINE},
}};

constexpr bool indexed_by_enumerator()
{
  for (size_t i = 0; i < k_policies.size(); ++i) {
    if (static_cast<size_t>(k_policies[i].policy) != i) {
      return false;
    }
  }
  return true;
}
static_assert(indexed_by_enumerator(), "k_policies must be ordered by SchedPolicy value");

const PolicyEntry & entry(SchedPolicy policy)
{
  return k_policies[static_cast<size_t>(policy)];
}

}  // namespace

std::optional<SchedPolicy> parse_sched_policy(std::string_view name)
{
  const auto * const it = std::find_if(
    k_policies.begin(), k_policies.end(), [name](const PolicyEntry & e) { return e.name == name; });
  if (it == k_policies.end()) {
    return std::nullopt;
  }
  return it->policy;
}

std::string_view to_string(SchedPolicy policy)
{
  return entry(policy).name;
}

int to_kernel_policy(SchedPolicy policy)
{
  return entry(policy).kernel_policy;
}

std::optional<SchedPolicy> from_kernel_policy(int kernel_policy)
{
  const auto * const it = std::find_if(
    k_policies.begin(), k_policies.end(),
    [kernel_policy](const PolicyEntry & e) { return e.kernel_policy == kernel_policy; });
  if (it == k_policies.end()) {
    return std::nullopt;
  }
  return it->policy;
}

bool is_cfs(SchedPolicy policy)
{
  return policy == SchedPolicy::Other || policy == SchedPolicy::Batch ||
         policy == SchedPolicy::Idle;
}

std::string sched_policy_names()
{
  std::string names;
  for (const auto & e : k_policies) {
    if (!names.empty()) {
      names += ", ";
    }
    names += e.name;
  }
  return names;
}

}  // namespace agnocast_cie_thread_configurator
