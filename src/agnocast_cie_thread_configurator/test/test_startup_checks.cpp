#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace acie = agnocast_cie_thread_configurator;

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
  ASSERT_EQ(info.size(), 7u);
  EXPECT_EQ(info.at("model_name"), "AMD Ryzen 7 5800X 8-Core Processor");
  EXPECT_EQ(info.at("cpu_family"), "25");
  EXPECT_EQ(info.at("model"), "33");
  EXPECT_EQ(info.at("threads_per_core"), "2");
  EXPECT_EQ(info.at("frequency_boost"), "enabled");
  EXPECT_EQ(info.at("cpu_max_mhz"), "4850.1948");
  EXPECT_EQ(info.at("cpu_min_mhz"), "2200.0000");
}

TEST(ParseLscpuOutput, TrimsValuesAndEmptiesWhitespaceOnlyOnes)
{
  const auto info =
    acie::parse_lscpu_output("Model name:\t  Cortex-A76  \r\nFrequency boost:   \r\nModel:\n");
  ASSERT_EQ(info.size(), 3u);
  EXPECT_EQ(info.at("model_name"), "Cortex-A76");
  EXPECT_EQ(info.at("frequency_boost"), "");
  EXPECT_EQ(info.at("model"), "");
}

TEST(ParseLscpuOutput, MatchesIndentedHierarchicKeys)
{
  // The indented layout util-linux >= 2.38 emits for hybrid-CPU machines.
  const char * output =
    "CPU(s):                 20\n"
    "  Model name:           12th Gen Intel(R) Core(TM) i7-12700H\n"
    "    CPU family:         6\n"
    "    Thread(s) per core: 2\n";
  const auto info = acie::parse_lscpu_output(output);
  ASSERT_EQ(info.size(), 3u);
  EXPECT_EQ(info.at("model_name"), "12th Gen Intel(R) Core(TM) i7-12700H");
  EXPECT_EQ(info.at("cpu_family"), "6");
  EXPECT_EQ(info.at("threads_per_core"), "2");
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
