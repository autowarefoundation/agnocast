#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace acie = agnocast_cie_thread_configurator;

// ---------- parse_lscpu_output ----------

TEST(ParseLscpuOutput, ExtractsExactlyTheRecordedKeys)
{
  const char * output =
    "Architecture:                    x86_64\n"
    "CPU(s):                          16\n"
    "Model name:                      AMD Ryzen 7 5800X 8-Core Processor\n"
    "CPU family:                      25\n"
    "Model:                           33\n"
    "Thread(s) per core:              2\n"
    "Frequency boost:                 enabled\n"
    "CPU max MHz:                     4850.1948\n"
    "CPU min MHz:                     2200.0000\n"
    "Vendor ID:                       AuthenticAMD\n";
  const auto info = acie::parse_lscpu_output(output);
  EXPECT_EQ(info.size(), 7u);
  EXPECT_EQ(info.at("model_name"), "AMD Ryzen 7 5800X 8-Core Processor");
  EXPECT_EQ(info.at("cpu_family"), "25");
  EXPECT_EQ(info.at("model"), "33");
  EXPECT_EQ(info.at("threads_per_core"), "2");
  EXPECT_EQ(info.at("frequency_boost"), "enabled");
  EXPECT_EQ(info.at("cpu_max_mhz"), "4850.1948");
  EXPECT_EQ(info.at("cpu_min_mhz"), "2200.0000");
}

TEST(ParseLscpuOutput, TrimsWhitespaceAroundValues)
{
  const auto info = acie::parse_lscpu_output("Model name:\t  Cortex-A76  \r\n");
  ASSERT_EQ(info.size(), 1u);
  EXPECT_EQ(info.at("model_name"), "Cortex-A76");
}

TEST(ParseLscpuOutput, IgnoresMalformedAndUnknownLines)
{
  EXPECT_TRUE(acie::parse_lscpu_output("").empty());
  EXPECT_TRUE(acie::parse_lscpu_output("no colon here\n").empty());
  EXPECT_TRUE(acie::parse_lscpu_output("Vendor ID: GenuineIntel\n").empty());
  // "BIOS Model name" must not be picked up as "Model name".
  EXPECT_TRUE(acie::parse_lscpu_output("BIOS Model name: To Be Filled By O.E.M.\n").empty());
}

// ---------- check_hardware_info ----------

TEST(CheckHardwareInfo, MissingSectionMeansSkippedNotPassed)
{
  const auto result =
    acie::check_hardware_info(YAML::Load("callback_groups: []"), {{"model", "33"}});
  EXPECT_FALSE(result.has_value());
}

TEST(CheckHardwareInfo, MatchingValuesYieldNoMismatch)
{
  const auto yaml = YAML::Load("hardware_info:\n  model: '33'\n  cpu_family: '25'\n");
  const auto result = acie::check_hardware_info(yaml, {{"model", "33"}, {"cpu_family", "25"}});
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}

TEST(CheckHardwareInfo, ReportsEachMismatchWithExpectedAndActual)
{
  const auto yaml = YAML::Load("hardware_info:\n  model: '33'\n  threads_per_core: '2'\n");
  const auto result = acie::check_hardware_info(yaml, {{"model", "44"}, {"threads_per_core", "1"}});
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
  // `current` is a std::map, so mismatches come back in key order.
  EXPECT_EQ((*result)[0].key, "model");
  EXPECT_EQ((*result)[0].expected, "33");
  EXPECT_EQ((*result)[0].actual, "44");
  EXPECT_EQ((*result)[1].key, "threads_per_core");
  EXPECT_EQ((*result)[1].expected, "2");
  EXPECT_EQ((*result)[1].actual, "1");
}

TEST(CheckHardwareInfo, ComparesOnlyKeysPresentOnBothSides)
{
  // The YAML pins keys this machine did not report, and the machine reports
  // keys the YAML does not pin; neither may produce a mismatch.
  const auto yaml = YAML::Load("hardware_info:\n  model: '33'\n  cpu_max_mhz: '9999'\n");
  const auto result = acie::check_hardware_info(yaml, {{"model", "33"}, {"cpu_min_mhz", "2200"}});
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}

// ---------- check_rt_throttling ----------

namespace
{

// read_sysctl double: records the requested paths and serves fixed values.
struct FakeSysctl
{
  std::map<std::string, std::optional<int>> values;
  mutable std::vector<std::string> requested;

  std::function<std::optional<int>(const std::string &)> reader() const
  {
    return [this](const std::string & path) {
      requested.push_back(path);
      const auto it = values.find(path);
      return it == values.end() ? std::nullopt : it->second;
    };
  }
};

constexpr const char * k_period_path = "/proc/sys/kernel/sched_rt_period_us";
constexpr const char * k_runtime_path = "/proc/sys/kernel/sched_rt_runtime_us";

}  // namespace

TEST(CheckRtThrottling, MissingSectionYieldsEmptyReport)
{
  FakeSysctl sysctl;
  const auto report = acie::check_rt_throttling(YAML::Load("callback_groups: []"), sysctl.reader());
  EXPECT_TRUE(report.checks.empty());
  EXPECT_FALSE(report.mismatch);
  EXPECT_TRUE(report.sysctl_guidance.empty());
  EXPECT_TRUE(sysctl.requested.empty());
}

TEST(CheckRtThrottling, ReadsTheKernelSysctlPathsInYamlKeyOrder)
{
  FakeSysctl sysctl;
  sysctl.values[k_period_path] = 1000000;
  sysctl.values[k_runtime_path] = 950000;
  const auto yaml = YAML::Load("rt_throttling:\n  period_us: 1000000\n  runtime_us: 950000\n");
  const auto report = acie::check_rt_throttling(yaml, sysctl.reader());

  EXPECT_EQ(sysctl.requested, (std::vector<std::string>{k_period_path, k_runtime_path}));
  ASSERT_EQ(report.checks.size(), 2u);
  EXPECT_EQ(report.checks[0].key, "sched_rt_period_us");
  EXPECT_EQ(report.checks[0].expected, 1000000);
  EXPECT_EQ(report.checks[0].actual, 1000000);
  EXPECT_EQ(report.checks[1].key, "sched_rt_runtime_us");
  EXPECT_EQ(report.checks[1].expected, 950000);
  EXPECT_EQ(report.checks[1].actual, 950000);
  EXPECT_FALSE(report.mismatch);
  EXPECT_TRUE(report.sysctl_guidance.empty());
}

TEST(CheckRtThrottling, MismatchGuidanceListsEveryConfiguredKey)
{
  FakeSysctl sysctl;
  sysctl.values[k_period_path] = 1000000;  // matches
  sysctl.values[k_runtime_path] = -1;      // differs
  const auto yaml = YAML::Load("rt_throttling:\n  period_us: 1000000\n  runtime_us: 950000\n");
  const auto report = acie::check_rt_throttling(yaml, sysctl.reader());

  EXPECT_TRUE(report.mismatch);
  EXPECT_EQ(
    report.sysctl_guidance,
    "rt_throttling values do not match the configuration. "
    "Please create /etc/sysctl.d/99-rt-throttling.conf with the following content and reboot "
    "(or run 'sudo sysctl --system'):\n"
    "  kernel.sched_rt_period_us = 1000000\n"
    "  kernel.sched_rt_runtime_us = 950000");
}

TEST(CheckRtThrottling, ChecksOnlyTheConfiguredKeys)
{
  FakeSysctl sysctl;
  sysctl.values[k_runtime_path] = -1;
  const auto yaml = YAML::Load("rt_throttling:\n  runtime_us: 950000\n");
  const auto report = acie::check_rt_throttling(yaml, sysctl.reader());

  EXPECT_EQ(sysctl.requested, (std::vector<std::string>{k_runtime_path}));
  ASSERT_EQ(report.checks.size(), 1u);
  EXPECT_EQ(report.checks[0].key, "sched_rt_runtime_us");
  EXPECT_TRUE(report.mismatch);
  EXPECT_EQ(
    report.sysctl_guidance,
    "rt_throttling values do not match the configuration. "
    "Please create /etc/sysctl.d/99-rt-throttling.conf with the following content and reboot "
    "(or run 'sudo sysctl --system'):\n"
    "  kernel.sched_rt_runtime_us = 950000");
}

TEST(CheckRtThrottling, UnreadableSysctlIsRecordedButNeverAMismatch)
{
  FakeSysctl sysctl;  // serves nothing: every read returns nullopt
  const auto yaml = YAML::Load("rt_throttling:\n  period_us: 1000000\n  runtime_us: 950000\n");
  const auto report = acie::check_rt_throttling(yaml, sysctl.reader());

  ASSERT_EQ(report.checks.size(), 2u);
  EXPECT_FALSE(report.checks[0].actual.has_value());
  EXPECT_FALSE(report.checks[1].actual.has_value());
  EXPECT_FALSE(report.mismatch);
  EXPECT_TRUE(report.sysctl_guidance.empty());
}
