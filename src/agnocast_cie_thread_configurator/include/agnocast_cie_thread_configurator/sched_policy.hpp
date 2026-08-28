#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace agnocast_cie_thread_configurator
{

// Scheduling policies with a YAML representation. Enumerator order is the
// order sched_policy_names() lists them in.
enum class SchedPolicy { Other, Batch, Idle, Fifo, Rr, Deadline };

// YAML policy name ("SCHED_FIFO") <-> policy. Exact, case-sensitive match;
// nullopt for any other string.
std::optional<SchedPolicy> parse_sched_policy(std::string_view name);
std::string_view to_string(SchedPolicy policy);

// Kernel SCHED_* value (sched_setscheduler(2), /proc/<pid>/stat field 41)
// <-> policy. nullopt for values that have no YAML representation.
int to_kernel_policy(SchedPolicy policy);
std::optional<SchedPolicy> from_kernel_policy(int kernel_policy);

// The CFS policies (OTHER/BATCH/IDLE) are tuned by nice, FIFO/RR by
// rt_priority, DEADLINE by runtime/period/deadline. The template emitter and
// the parser rely on this split.
bool is_cfs(SchedPolicy policy);

// "SCHED_OTHER, SCHED_BATCH, ..." for error messages.
std::string sched_policy_names();

}  // namespace agnocast_cie_thread_configurator
