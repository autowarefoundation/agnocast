#pragma once

#include "yaml-cpp/yaml.h"

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// The hardware_info entries the template records and startup validates, keyed
// by the YAML key (model_name, cpu_family, ...). Empty when lscpu cannot be
// run or reports none of them: callers have nothing to record or compare.
std::map<std::string, std::string> get_hardware_info();

// The parsing half of get_hardware_info, split out so it can be tested
// without running lscpu. Keeps only the keys hardware_info records; values
// are whitespace-trimmed.
std::map<std::string, std::string> parse_lscpu_output(const std::string & output);

// One hardware_info key whose configured value does not match this machine.
struct HardwareMismatch
{
  std::string key;
  std::string expected;  // value in the YAML
  std::string actual;    // value on this machine
};

// Compare the YAML hardware_info section against `current` (typically
// get_hardware_info()). Keys missing from either side are not compared: the
// YAML records only what the user wants pinned, and `current` only what lscpu
// reported on this machine. nullopt when the YAML has no hardware_info
// section, so the caller can tell "validation skipped" from "no mismatch".
std::optional<std::vector<HardwareMismatch>> check_hardware_info(
  const YAML::Node & yaml, const std::map<std::string, std::string> & current);

// One rt_throttling key present in the YAML, compared against the value
// read_sysctl returned for the corresponding /proc/sys/kernel file.
struct RtThrottlingCheck
{
  std::string key;  // "sched_rt_period_us" or "sched_rt_runtime_us"
  int expected = 0;
  std::optional<int> actual;  // nullopt: the sysctl could not be read
};

struct RtThrottlingReport
{
  std::vector<RtThrottlingCheck> checks;  // in YAML key order: period_us, runtime_us
  bool mismatch = false;                  // some check has a readable actual != expected
  // Non-empty iff mismatch: operator guidance listing every configured key as
  // an /etc/sysctl.d entry.
  std::string sysctl_guidance;
};

// Writing /proc/sys/kernel/sched_rt_{period,runtime}_us requires root
// (uid 0); Linux capabilities (CAP_SYS_ADMIN etc.) cannot bypass the proc
// sysctl DAC check. So the configured values are only compared against the
// running kernel's, and the caller reports sysctl_guidance on a mismatch.
// read_sysctl receives the /proc/sys/kernel path and returns nullopt when the
// value cannot be read (such a key is recorded but never counts as a
// mismatch); reporting that failure is the reader's job.
RtThrottlingReport check_rt_throttling(
  const YAML::Node & yaml,
  const std::function<std::optional<int>(const std::string & path)> & read_sysctl);

}  // namespace agnocast_cie_thread_configurator
