#include "agnocast_cie_thread_configurator/startup_checks.hpp"

#include <gtest/gtest.h>

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
