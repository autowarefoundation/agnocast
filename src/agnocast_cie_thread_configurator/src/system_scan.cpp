#include "agnocast_cie_thread_configurator/system_scan.hpp"

#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <system_error>
#include <tuple>

namespace agnocast_cie_thread_configurator
{

namespace
{

// Not exposed by uapi headers; values from include/linux/sched.h.
constexpr unsigned long k_pf_kthread = 0x00200000;
constexpr unsigned long k_pf_no_setaffinity = 0x04000000;

// Mainstream kernels cap NR_CPUS at 8192, so any larger CPU number is
// malformed input; bounding it also keeps range expansion small.
constexpr int k_max_cpu_number = 8191;

// 1-based /proc/<pid>/stat field numbers (man 5 proc); see split_stat_after_comm.
constexpr size_t k_stat_field_flags = 9;
constexpr size_t k_stat_field_nice = 19;
constexpr size_t k_stat_field_rt_priority = 40;
constexpr size_t k_stat_field_policy = 41;

std::optional<std::string> read_first_line(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file) {
    return std::nullopt;
  }
  std::string line;
  std::getline(file, line);
  if (file.bad()) {
    return std::nullopt;
  }
  return line;
}

bool is_all_digits(std::string_view s)
{
  return !s.empty() &&
         std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

template <typename T>
std::optional<T> to_number(std::string_view s)
{
  T value{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc() || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

// The stat comm may contain spaces and parentheses (e.g. "irq/24-PCIe PME"),
// so split at the LAST ')'. tokens[0] is field 3 (state), so field N maps to
// tokens[N - 3]. The returned views point into `stat`.
std::optional<std::vector<std::string_view>> split_stat_after_comm(std::string_view stat)
{
  const size_t close = stat.rfind(')');
  if (close == std::string_view::npos) {
    return std::nullopt;
  }
  std::vector<std::string_view> tokens;
  size_t pos = close + 1;
  while (pos < stat.size()) {
    pos = stat.find_first_not_of(' ', pos);
    if (pos == std::string_view::npos) {
      break;
    }
    size_t end = stat.find(' ', pos);
    if (end == std::string_view::npos) {
      end = stat.size();
    }
    tokens.push_back(stat.substr(pos, end - pos));
    pos = end;
  }
  return tokens;
}

std::optional<std::string_view> stat_token(
  const std::vector<std::string_view> & tokens, size_t field)
{
  const size_t index = field - 3;
  if (index >= tokens.size()) {
    return std::nullopt;
  }
  return tokens[index];
}

std::string policy_const_to_string(int policy)
{
  for (const auto & [name, value] : policy_to_sched_const) {
    if (value == policy) {
      return name;
    }
  }
  return "UNKNOWN(" + std::to_string(policy) + ")";
}

std::optional<std::string> read_cpus_allowed_list(const std::filesystem::path & status_path)
{
  std::ifstream file(status_path);
  if (!file) {
    return std::nullopt;
  }
  constexpr std::string_view key = "Cpus_allowed_list:";
  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, key.size(), key) != 0) {
      continue;
    }
    size_t begin = key.size();
    while (begin < line.size() && (line[begin] == ' ' || line[begin] == '\t')) {
      ++begin;
    }
    return line.substr(begin);
  }
  return std::nullopt;
}

std::optional<KernelThreadInfo> read_kernel_thread(
  const std::filesystem::path & pid_dir, int64_t tid)
{
  const auto stat = read_first_line(pid_dir / "stat");
  if (!stat) {
    return std::nullopt;
  }
  const auto tokens = split_stat_after_comm(*stat);
  if (!tokens) {
    return std::nullopt;
  }

  const auto flags_str = stat_token(*tokens, k_stat_field_flags);
  if (!flags_str) {
    return std::nullopt;
  }
  const auto flags = to_number<unsigned long>(*flags_str);
  if (!flags) {
    return std::nullopt;
  }
  // Most /proc entries are user processes; gate on PF_KTHREAD before parsing
  // the remaining fields.
  if ((*flags & k_pf_kthread) == 0) {
    return std::nullopt;
  }

  const auto nice_str = stat_token(*tokens, k_stat_field_nice);
  const auto rt_priority_str = stat_token(*tokens, k_stat_field_rt_priority);
  const auto policy_str = stat_token(*tokens, k_stat_field_policy);
  if (!nice_str || !rt_priority_str || !policy_str) {
    return std::nullopt;
  }
  const auto nice = to_number<int>(*nice_str);
  const auto rt_priority = to_number<int>(*rt_priority_str);
  const auto policy = to_number<int>(*policy_str);
  if (!nice || !rt_priority || !policy) {
    return std::nullopt;
  }

  // The comm between the first '(' and the last ')' of stat is the same
  // task->comm that /proc/<pid>/comm reports; taking it from stat keeps all
  // fields from a single read.
  const size_t open = stat->find('(');
  const size_t close = stat->rfind(')');
  if (open == std::string::npos || close == std::string::npos || close <= open + 1) {
    return std::nullopt;
  }
  const std::string comm = stat->substr(open + 1, close - open - 1);
  if (is_kworker_comm(comm)) {
    return std::nullopt;
  }

  const auto affinity = read_cpus_allowed_list(pid_dir / "status");
  if (!affinity) {
    return std::nullopt;
  }

  KernelThreadInfo info;
  info.tid = tid;
  info.comm = comm;
  info.policy = policy_const_to_string(*policy);
  info.nice = *nice;
  info.rt_priority = *rt_priority;
  info.affinity = *affinity;
  info.no_setaffinity = (*flags & k_pf_no_setaffinity) != 0;
  return info;
}

}  // namespace

std::vector<KernelThreadInfo> scan_kernel_threads(const std::string & proc_root)
{
  std::vector<KernelThreadInfo> result;
  std::error_code ec;
  std::filesystem::directory_iterator it(proc_root, ec);
  for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
    const auto & entry = *it;
    const std::string filename = entry.path().filename().string();
    if (!is_all_digits(filename)) {
      continue;
    }
    const auto tid = to_number<int64_t>(filename);
    if (!tid) {
      continue;
    }
    if (auto info = read_kernel_thread(entry.path(), *tid)) {
      result.push_back(std::move(*info));
    }
  }
  std::sort(
    result.begin(), result.end(), [](const KernelThreadInfo & a, const KernelThreadInfo & b) {
      return std::tie(a.comm, a.tid) < std::tie(b.comm, b.tid);
    });
  return result;
}

std::vector<IrqInfo> scan_irqs(const std::string & proc_irq_root, const std::string & sys_irq_root)
{
  std::vector<IrqInfo> result;
  const std::filesystem::path proc_root(proc_irq_root);
  std::error_code ec;
  std::filesystem::directory_iterator it(sys_irq_root, ec);
  for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
    const auto & entry = *it;
    const std::string filename = entry.path().filename().string();
    if (!is_all_digits(filename)) {
      continue;
    }
    const auto irq = to_number<int>(filename);
    if (!irq) {
      continue;
    }
    const auto actions = read_first_line(entry.path() / "actions");
    if (!actions || actions->empty()) {
      continue;
    }

    IrqInfo info;
    info.irq = *irq;
    info.name = *actions;
    info.affinity =
      read_first_line(proc_root / std::to_string(*irq) / "smp_affinity_list").value_or("");
    result.push_back(std::move(info));
  }
  std::sort(result.begin(), result.end(), [](const IrqInfo & a, const IrqInfo & b) {
    return a.irq < b.irq;
  });
  return result;
}

std::optional<std::string> read_irq_actions(int irq, const std::string & sys_irq_root)
{
  auto actions =
    read_first_line(std::filesystem::path(sys_irq_root) / std::to_string(irq) / "actions");
  if (actions && actions->empty()) {
    return std::nullopt;
  }
  return actions;
}

std::vector<const KernelThreadInfo *> find_kernel_threads_by_comm(
  const std::vector<KernelThreadInfo> & scanned, const std::string & comm)
{
  std::vector<const KernelThreadInfo *> result;
  for (const auto & info : scanned) {
    if (info.comm == comm) {
      result.push_back(&info);
    }
  }
  return result;
}

bool process_with_comm_exists(const std::string & comm, const std::string & proc_root)
{
  std::error_code ec;
  std::filesystem::directory_iterator it(proc_root, ec);
  for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
    const auto & entry = *it;
    if (!is_all_digits(entry.path().filename().string())) {
      continue;
    }
    const auto entry_comm = read_first_line(entry.path() / "comm");
    if (entry_comm && *entry_comm == comm) {
      return true;
    }
  }
  return false;
}

std::string format_cpu_list(const std::vector<int> & cpus)
{
  std::string result;
  for (size_t i = 0; i < cpus.size(); i++) {
    if (i > 0) {
      result += ",";
    }
    result += std::to_string(cpus[i]);
  }
  return result;
}

std::optional<std::vector<int>> parse_cpu_list(const std::string & s)
{
  std::vector<int> result;
  std::string_view rest(s);
  // Tolerate the trailing newline that raw kernel file contents carry.
  while (!rest.empty() && (rest.back() == '\n' || rest.back() == '\r' || rest.back() == ' ')) {
    rest.remove_suffix(1);
  }
  if (rest.empty()) {
    return std::nullopt;
  }
  while (!rest.empty()) {
    const size_t comma = rest.find(',');
    const std::string_view token = rest.substr(0, comma);
    rest = (comma == std::string_view::npos) ? std::string_view() : rest.substr(comma + 1);
    if (comma != std::string_view::npos && rest.empty()) {
      return std::nullopt;
    }

    const size_t dash = token.find('-');
    if (dash == std::string_view::npos) {
      const auto cpu = to_number<int>(token);
      if (!cpu || *cpu < 0 || *cpu > k_max_cpu_number) {
        return std::nullopt;
      }
      result.push_back(*cpu);
    } else {
      const auto first = to_number<int>(token.substr(0, dash));
      const auto last = to_number<int>(token.substr(dash + 1));
      if (!first || !last || *first < 0 || *last < *first || *last > k_max_cpu_number) {
        return std::nullopt;
      }
      for (int cpu = *first; cpu <= *last; cpu++) {
        result.push_back(cpu);
      }
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

}  // namespace agnocast_cie_thread_configurator
