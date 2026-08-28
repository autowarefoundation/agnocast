#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// One kernel thread observed in a /proc scan (kthreads are single-threaded,
// so top-level /proc pid dirs suffice). All fields are file-derived, so unit
// tests can inject a fake tree.
struct KernelThreadInfo
{
  int64_t tid = -1;
  std::string comm;             // /proc/<pid>/comm, trailing newline stripped
  std::string policy;           // "SCHED_OTHER" etc., or "UNKNOWN(<n>)" for unmapped values
  int nice = 0;                 // stat nice; meaningful for OTHER/BATCH/IDLE
  int rt_priority = 0;          // stat rt_priority; meaningful for FIFO/RR
  std::string affinity;         // raw Cpus_allowed_list string, e.g. "0-15"
  bool no_setaffinity = false;  // PF_NO_SETAFFINITY: per-CPU kthread, affinity fixed by kernel
};

// One device-backed IRQ (non-empty /sys/kernel/irq/<N>/actions).
struct IrqInfo
{
  int irq = -1;
  std::string name;      // actions file content (comma-joined for shared IRQs)
  std::string affinity;  // /proc/irq/<N>/smp_affinity_list, "" when unreadable
};

// kworker comms embed worker-pool/CPU state that mutates at runtime, so they
// can never be matched reliably; both the scanner and the kernel_threads
// parser exclude them.
bool is_kworker_comm(const std::string & comm);

// Sorted by (comm, tid); entries that vanish mid-scan are skipped silently.
// kworker/* are excluded: their comms mutate at runtime, so they cannot be
// managed per-thread.
std::vector<KernelThreadInfo> scan_kernel_threads(const std::string & proc_root = "/proc");

// Sorted by irq. Iterates numeric directories under sys_irq_root
// (authoritative for actions); the /proc/irq affinity read is best-effort.
std::vector<IrqInfo> scan_irqs(
  const std::string & proc_irq_root = "/proc/irq",
  const std::string & sys_irq_root = "/sys/kernel/irq");

// nullopt when the IRQ directory or its actions file is missing, unreadable,
// or empty (the IRQ is no longer device-backed, matching scan_irqs).
std::optional<std::string> read_irq_actions(
  int irq, const std::string & sys_irq_root = "/sys/kernel/irq");

// Pointers into `scanned`, in scan order; comms are not unique (e.g. an nfsd
// pool), so every match is returned.
std::vector<const KernelThreadInfo *> find_kernel_threads_by_comm(
  const std::vector<KernelThreadInfo> & scanned, const std::string & comm);

// True when any /proc/<pid>/comm equals `comm` (e.g. irqbalance detection).
bool process_with_comm_exists(const std::string & comm, const std::string & proc_root = "/proc");

// {2, 3} -> "2,3": the comma list accepted by /proc/irq/<N>/smp_affinity_list.
std::string format_cpu_list(const std::vector<int> & cpus);

// Kernel cpu-list format ("0-5,8") -> sorted, deduplicated {0..5,8};
// nullopt on empty or malformed input.
std::optional<std::vector<int>> parse_cpu_list(const std::string & s);

// Highest CPU number this tool manages on this machine:
// min(CPU_SETSIZE, _SC_NPROCESSORS_CONF) - 1. The YAML affinity parser and
// parse_manageable_cpu_list both bound by it, so an untouched template entry
// compares equal to the observed value it was emitted from.
int manageable_cpu_bound();

// parse_cpu_list, additionally dropping CPUs above manageable_cpu_bound():
// the kernel prints affinity over the possible-CPU mask, which can exceed the
// present CPUs. nullopt when nothing manageable remains.
std::optional<std::vector<int>> parse_manageable_cpu_list(const std::string & s);

}  // namespace agnocast_cie_thread_configurator
