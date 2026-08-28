#include "agnocast_cie_thread_configurator/system_scan.hpp"

#include <gtest/gtest.h>
#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace acie = agnocast_cie_thread_configurator;
namespace fs = std::filesystem;

namespace
{

// From include/linux/sched.h (not exposed by uapi headers).
constexpr unsigned long kPfKthread = 0x00200000;
constexpr unsigned long kPfNoSetaffinity = 0x04000000;

// Kernel SCHED_* constants as they appear in /proc/<pid>/stat field 41.
constexpr int kPolicyOther = 0;
constexpr int kPolicyFifo = 1;
constexpr int kPolicyBatch = 3;

// Each test builds its own fake /proc//sys tree and removes it afterwards.
struct TempTree
{
  fs::path root;

  TempTree()
  {
    static std::atomic<int> counter{0};
    root = fs::temp_directory_path() / ("agnocast_system_scan_test_" + std::to_string(::getpid()) +
                                        "_" + std::to_string(counter.fetch_add(1)));
    fs::create_directories(root);
  }

  ~TempTree()
  {
    std::error_code ec;
    fs::remove_all(root, ec);
  }
};

void write_file(const fs::path & path, const std::string & content)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path);
  file << content;
}

// Layout per `man 5 proc`: pid (comm) state ppid ... ; the token after the
// closing paren is field 3 (state), flags is field 9, nice 19, rt_priority 40,
// policy 41.
std::string make_stat_line(
  int pid, const std::string & comm, unsigned long flags, int nice, int rt_priority, int policy)
{
  std::string line = std::to_string(pid) + " (" + comm + ") S 2 0 0 0 -1 " + std::to_string(flags);
  line += " 0 0 0 0 0 0 0 0 20";  // fields 10-18
  line += " " + std::to_string(nice);
  for (int i = 0; i < 20; ++i) {  // fields 20-39
    line += " 0";
  }
  line += " " + std::to_string(rt_priority) + " " + std::to_string(policy);
  line += " 0 0 0 0 0 0 0 0 0 0";
  return line;
}

void add_proc_entry(
  const fs::path & proc_root, int pid, const std::string & comm, unsigned long flags, int nice,
  int rt_priority, int policy, const std::string & cpus_allowed_list)
{
  const fs::path dir = proc_root / std::to_string(pid);
  write_file(dir / "stat", make_stat_line(pid, comm, flags, nice, rt_priority, policy) + "\n");
  write_file(dir / "comm", comm + "\n");
  write_file(
    dir / "status", "Name:\t" + comm + "\nCpus_allowed_list:\t" + cpus_allowed_list + "\n");
}

void add_irq_entry(
  const fs::path & sys_root, const fs::path & proc_irq_root, int irq, const std::string & actions,
  const std::string & affinity_list)
{
  const fs::path sys_dir = sys_root / std::to_string(irq);
  write_file(sys_dir / "actions", actions + "\n");
  // Real /sys/kernel/irq/<N> dirs carry further attribute files; one is
  // enough to prove the scan ignores files it does not consume.
  write_file(sys_dir / "chip_name", "IO-APIC\n");
  write_file(proc_irq_root / std::to_string(irq) / "smp_affinity_list", affinity_list + "\n");
}

}  // namespace

// ---------- scan_kernel_threads ----------

TEST(ScanKernelThreads, FindsKthreadAndReadsFields)
{
  TempTree tree;
  add_proc_entry(tree.root, 15, "my_kthread", kPfKthread | 0x40, 0, 50, kPolicyFifo, "0-3");
  add_proc_entry(tree.root, 100, "user_process", 0x00400040, 0, 0, kPolicyOther, "0-3");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].tid, 15);
  EXPECT_EQ(result[0].comm, "my_kthread");
  EXPECT_EQ(result[0].policy, "SCHED_FIFO");
  EXPECT_EQ(result[0].nice, 0);
  EXPECT_EQ(result[0].rt_priority, 50);
  EXPECT_EQ(result[0].affinity, "0-3");
  EXPECT_FALSE(result[0].no_setaffinity);
}

TEST(ScanKernelThreads, CapturesNiceAndRtPriority)
{
  TempTree tree;
  add_proc_entry(tree.root, 10, "nice_thread", kPfKthread, 5, 0, kPolicyOther, "0");
  add_proc_entry(tree.root, 11, "rt_thread", kPfKthread, 0, 42, kPolicyFifo, "0");
  add_proc_entry(tree.root, 12, "batch_thread", kPfKthread, -7, 0, kPolicyBatch, "0");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0].comm, "batch_thread");
  EXPECT_EQ(result[0].nice, -7);
  EXPECT_EQ(result[0].rt_priority, 0);
  EXPECT_EQ(result[1].comm, "nice_thread");
  EXPECT_EQ(result[1].nice, 5);
  EXPECT_EQ(result[1].rt_priority, 0);
  EXPECT_EQ(result[2].comm, "rt_thread");
  EXPECT_EQ(result[2].nice, 0);
  EXPECT_EQ(result[2].rt_priority, 42);
}

TEST(ScanKernelThreads, ExcludesKworkers)
{
  TempTree tree;
  add_proc_entry(
    tree.root, 8, "kworker/0:0H-events_highpri", kPfKthread | kPfNoSetaffinity, 0, 0, kPolicyOther,
    "0");
  add_proc_entry(tree.root, 9, "kept_thread", kPfKthread, 0, 0, kPolicyOther, "0");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].comm, "kept_thread");
}

TEST(ScanKernelThreads, ParsesCommWithParensAndSpaces)
{
  TempTree tree;
  add_proc_entry(tree.root, 753, "irq/24-PCIe PME", kPfKthread, 0, 50, kPolicyFifo, "0-15");
  add_proc_entry(tree.root, 754, "a) b (c", kPfKthread, 3, 0, kPolicyOther, "1");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].comm, "a) b (c");
  EXPECT_EQ(result[0].nice, 3);
  EXPECT_EQ(result[1].comm, "irq/24-PCIe PME");
  EXPECT_EQ(result[1].policy, "SCHED_FIFO");
  EXPECT_EQ(result[1].rt_priority, 50);
}

TEST(ScanKernelThreads, SetsNoSetaffinityFlag)
{
  TempTree tree;
  add_proc_entry(
    tree.root, 15, "ksoftirqd/0", kPfKthread | kPfNoSetaffinity, 0, 0, kPolicyOther, "0");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result[0].no_setaffinity);
}

TEST(ScanKernelThreads, SortedByCommThenTid)
{
  TempTree tree;
  add_proc_entry(tree.root, 30, "nfsd", kPfKthread, 0, 0, kPolicyOther, "0");
  add_proc_entry(tree.root, 10, "zeta", kPfKthread, 0, 0, kPolicyOther, "0");
  add_proc_entry(tree.root, 20, "nfsd", kPfKthread, 0, 0, kPolicyOther, "0");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0].comm, "nfsd");
  EXPECT_EQ(result[0].tid, 20);
  EXPECT_EQ(result[1].comm, "nfsd");
  EXPECT_EQ(result[1].tid, 30);
  EXPECT_EQ(result[2].comm, "zeta");
}

TEST(ScanKernelThreads, SkipsMalformedEntries)
{
  TempTree tree;
  add_proc_entry(tree.root, 5, "good_thread", kPfKthread, 0, 0, kPolicyOther, "0");
  fs::create_directories(tree.root / "not_a_pid");
  fs::create_directories(tree.root / "6");  // no stat/comm/status at all
  write_file(tree.root / "7" / "stat", "7 (truncated) S 2 0\n");
  write_file(
    tree.root / "8" / "stat", make_stat_line(8, "no_status_file", kPfKthread, 0, 0, 0) + "\n");
  write_file(tree.root / "8" / "comm", "no_status_file\n");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].comm, "good_thread");

  EXPECT_TRUE(acie::scan_kernel_threads((tree.root / "missing_dir").string()).empty());
}

TEST(ScanKernelThreads, RendersUnknownPolicy)
{
  TempTree tree;
  add_proc_entry(tree.root, 15, "odd_thread", kPfKthread, -3, 7, 4, "0");

  const auto result = acie::scan_kernel_threads(tree.root.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].policy, "UNKNOWN(4)");
  EXPECT_EQ(result[0].nice, -3);
  EXPECT_EQ(result[0].rt_priority, 7);
}

// ---------- scan_irqs / read_irq_actions ----------

TEST(ScanIrqs, OnlyDeviceBackedIrqs)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  const fs::path proc = tree.root / "proc";
  add_irq_entry(sys, proc, 5, "eth0", "0-3");
  write_file(sys / "3" / "actions", "");             // present but empty
  fs::create_directories(sys / "4");                 // no actions file
  write_file(sys / "4" / "chip_name", "IO-APIC\n");  // still no actions

  const auto result = acie::scan_irqs(proc.string(), sys.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].irq, 5);
  EXPECT_EQ(result[0].name, "eth0");
}

TEST(ScanIrqs, ReadsAffinity)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  const fs::path proc = tree.root / "proc";
  add_irq_entry(sys, proc, 103, "nvidia", "0-15");

  const auto result = acie::scan_irqs(proc.string(), sys.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].name, "nvidia");
  EXPECT_EQ(result[0].affinity, "0-15");
}

TEST(ScanIrqs, MissingProcAffinityYieldsEmpty)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  const fs::path proc = tree.root / "proc";
  write_file(sys / "7" / "actions", "timer\n");

  const auto result = acie::scan_irqs(proc.string(), sys.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].affinity, "");
}

TEST(ScanIrqs, IgnoresNonNumericEntries)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  const fs::path proc = tree.root / "proc";
  add_irq_entry(sys, proc, 1, "i8042", "0");
  write_file(sys / "default_smp_affinity", "ffff\n");
  fs::create_directories(sys / "power");

  const auto result = acie::scan_irqs(proc.string(), sys.string());
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].irq, 1);
}

TEST(ScanIrqs, SortedByIrqNumber)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  const fs::path proc = tree.root / "proc";
  add_irq_entry(sys, proc, 100, "snd", "0");
  add_irq_entry(sys, proc, 24, "PCIe PME", "0");
  add_irq_entry(sys, proc, 3, "serial", "0");

  const auto result = acie::scan_irqs(proc.string(), sys.string());
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0].irq, 3);
  EXPECT_EQ(result[1].irq, 24);
  EXPECT_EQ(result[2].irq, 100);
}

TEST(ReadIrqActions, TrimsNewlineAndReportsMissing)
{
  TempTree tree;
  const fs::path sys = tree.root / "sys";
  write_file(sys / "103" / "actions", "nvidia\n");

  const auto present = acie::read_irq_actions(103, sys.string());
  ASSERT_TRUE(present.has_value());
  EXPECT_EQ(*present, "nvidia");

  EXPECT_FALSE(acie::read_irq_actions(999, sys.string()).has_value());
}

// ---------- find_kernel_threads_by_comm / process_with_comm_exists ----------

TEST(FindKernelThreadsByComm, MatchesAllEqualComms)
{
  std::vector<acie::KernelThreadInfo> scanned(3);
  scanned[0].comm = "nfsd";
  scanned[0].tid = 20;
  scanned[1].comm = "nfsd";
  scanned[1].tid = 30;
  scanned[2].comm = "other";
  scanned[2].tid = 40;

  const auto matches = acie::find_kernel_threads_by_comm(scanned, "nfsd");
  ASSERT_EQ(matches.size(), 2u);
  EXPECT_EQ(matches[0]->tid, 20);
  EXPECT_EQ(matches[1]->tid, 30);

  EXPECT_TRUE(acie::find_kernel_threads_by_comm(scanned, "absent").empty());
}

TEST(ProcessWithCommExists, FindsByExactComm)
{
  TempTree tree;
  write_file(tree.root / "1234" / "comm", "irqbalance\n");
  write_file(tree.root / "1235" / "comm", "bash\n");

  EXPECT_TRUE(acie::process_with_comm_exists("irqbalance", tree.root.string()));
  EXPECT_FALSE(acie::process_with_comm_exists("irqbalanc", tree.root.string()));
  EXPECT_FALSE(acie::process_with_comm_exists("nonexistent", tree.root.string()));
}

// ---------- is_kworker_comm ----------

TEST(IsKworkerComm, MatchesTheKworkerPrefixOnly)
{
  EXPECT_TRUE(acie::is_kworker_comm("kworker/0:0H-events_highpri"));
  EXPECT_TRUE(acie::is_kworker_comm("kworker/u16:3"));
  EXPECT_FALSE(acie::is_kworker_comm("kworker"));
  EXPECT_FALSE(acie::is_kworker_comm("ksoftirqd/0"));
  EXPECT_FALSE(acie::is_kworker_comm(""));
}

// ---------- format_cpu_list / parse_cpu_list ----------

TEST(FormatCpuList, JoinsWithCommas)
{
  EXPECT_EQ(acie::format_cpu_list({2, 3}), "2,3");
  EXPECT_EQ(acie::format_cpu_list({5}), "5");
  EXPECT_EQ(acie::format_cpu_list({}), "");
}

TEST(ParseCpuList, ParsesRangesAndSingles)
{
  auto result = acie::parse_cpu_list("0-5,8");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, (std::vector<int>{0, 1, 2, 3, 4, 5, 8}));

  result = acie::parse_cpu_list("3");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, (std::vector<int>{3}));

  result = acie::parse_cpu_list("0-15");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 16u);
  EXPECT_EQ(result->front(), 0);
  EXPECT_EQ(result->back(), 15);
}

TEST(ParseCpuList, ToleratesTrailingNewlineAndNormalizes)
{
  auto result = acie::parse_cpu_list("0-2\n");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, (std::vector<int>{0, 1, 2}));

  result = acie::parse_cpu_list("2,1,1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, (std::vector<int>{1, 2}));
}

TEST(ParseCpuList, RejectsMalformedInput)
{
  EXPECT_FALSE(acie::parse_cpu_list("").has_value());
  EXPECT_FALSE(acie::parse_cpu_list("a-b").has_value());
  EXPECT_FALSE(acie::parse_cpu_list("5-3").has_value());
  EXPECT_FALSE(acie::parse_cpu_list("-1").has_value());
  EXPECT_FALSE(acie::parse_cpu_list("1,,2").has_value());
  EXPECT_FALSE(acie::parse_cpu_list("1,x").has_value());
}

TEST(ParseCpuList, RoundTripsWithFormat)
{
  const std::vector<int> cpus{0, 1, 2, 3, 4, 5, 8};
  const auto parsed = acie::parse_cpu_list(acie::format_cpu_list(cpus));
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, cpus);
}

TEST(ParseManageableCpuList, DropsCpusAboveTheManageableBound)
{
  // "0-8191" spans every CPU number parse_cpu_list accepts, so the result
  // must be exactly the manageable range of the machine running the test.
  const long bound = std::min<long>(CPU_SETSIZE, sysconf(_SC_NPROCESSORS_CONF));
  const auto trimmed = acie::parse_manageable_cpu_list("0-8191");
  ASSERT_TRUE(trimmed.has_value());
  EXPECT_EQ(static_cast<long>(trimmed->size()), bound);
  EXPECT_EQ(trimmed->front(), 0);
  EXPECT_EQ(trimmed->back(), static_cast<int>(bound) - 1);

  // Nothing manageable left, and malformed input propagates as nullopt.
  EXPECT_EQ(acie::parse_manageable_cpu_list("8191"), std::nullopt);
  EXPECT_EQ(acie::parse_manageable_cpu_list(""), std::nullopt);
}
