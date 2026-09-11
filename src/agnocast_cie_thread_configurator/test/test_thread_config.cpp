#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <gtest/gtest.h>
#include <sched.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace acie = agnocast_cie_thread_configurator;

namespace
{
constexpr size_t kTestDefaultDomain = 7;

YAML::Node yaml_from_str(const char * s)
{
  return YAML::Load(s);
}

acie::ParsedConfig parse(const YAML::Node & yaml)
{
  return acie::parse_config(yaml, kTestDefaultDomain);
}

// EXPECT_THROW alone cannot tell WHICH validation fired (every failure path
// derives from std::runtime_error), so the negative tests also match a
// distinguishing fragment of the message.
void expect_error(const char * yaml, const std::string & fragment)
{
  try {
    parse(yaml_from_str(yaml));
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
  }
}
}  // namespace

// ---------- callback_groups / non_ros_threads ----------

TEST(ParseConfig, ParsesEmptyConfig)
{
  auto y = yaml_from_str("callback_groups: []\nnon_ros_threads: []\n");
  const auto config = parse(y);
  EXPECT_TRUE(config.callback_groups.empty());
  EXPECT_TRUE(config.non_ros_threads.empty());
}

TEST(ParseConfig, ParsesCallbackGroupSchedFifo)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 3
    policy: SCHED_FIFO
    priority: 50
    affinity: [0, 1]
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].id, "my_cbg");
  EXPECT_EQ(config.callback_groups[0].domain_id, 3u);
  EXPECT_EQ(config.callback_groups[0].attrs.policy, acie::SchedPolicy::Fifo);
  EXPECT_EQ(config.callback_groups[0].attrs.rt_priority, 50);
  EXPECT_EQ(config.callback_groups[0].attrs.affinity, (std::vector<int>{0, 1}));
  EXPECT_FALSE(config.callback_groups[0].is_wildcard());
}

TEST(ParseConfig, ParsesNiceForCfsPolicies)
{
  for (const char * policy : {"SCHED_OTHER", "SCHED_BATCH", "SCHED_IDLE"}) {
    auto y = yaml_from_str(("callback_groups:\n"
                            "  - id: my_cbg\n"
                            "    domain_id: 0\n"
                            "    policy: " +
                            std::string(policy) +
                            "\n"
                            "    nice: -10\n"
                            "    affinity: []\n"
                            "non_ros_threads: []\n")
                             .c_str());
    const auto config = parse(y);
    ASSERT_EQ(config.callback_groups.size(), 1u);
    EXPECT_EQ(config.callback_groups[0].attrs.nice, -10) << policy;
    EXPECT_EQ(config.callback_groups[0].attrs.rt_priority, 0) << policy;
  }
}

TEST(ParseConfig, IgnoresStrayKeyOfTheOtherPolicyClass)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: cfs_cbg
    domain_id: 0
    policy: SCHED_OTHER
    nice: -5
    priority: 50
    affinity: []
  - id: rt_cbg
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    nice: 10
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 2u);
  EXPECT_EQ(config.callback_groups[0].attrs.nice, -5);
  EXPECT_EQ(config.callback_groups[0].attrs.rt_priority, 0);
  EXPECT_EQ(config.callback_groups[1].attrs.rt_priority, 50);
  EXPECT_EQ(config.callback_groups[1].attrs.nice, 0);
}

TEST(ParseConfig, RejectsMissingNiceOnSchedOther)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_OTHER
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsNiceOutOfRange)
{
  for (const char * bad_nice : {"-21", "20", "50"}) {
    auto y = yaml_from_str(("callback_groups:\n"
                            "  - id: my_cbg\n"
                            "    domain_id: 0\n"
                            "    policy: SCHED_OTHER\n"
                            "    nice: " +
                            std::string(bad_nice) +
                            "\n"
                            "    affinity: []\n"
                            "non_ros_threads: []\n")
                             .c_str());
    EXPECT_THROW(parse(y), std::runtime_error) << "nice=" << bad_nice;
  }
}

TEST(ParseConfig, TreatsNullNiceAsMissing)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_OTHER
    nice:
    affinity: []
non_ros_threads: []
)YAML");
  try {
    parse(y);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("requires 'nice'"), std::string::npos) << e.what();
  }
}

TEST(ParseConfig, ReportsEntryOnNonIntegerNice)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_OTHER
    nice: low
    affinity: []
non_ros_threads: []
)YAML");
  try {
    parse(y);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'nice' must be an integer"), std::string::npos) << what;
    EXPECT_NE(what.find("id=my_cbg"), std::string::npos) << what;
  }
}

TEST(ParseConfig, ReportsEntryOnNonIntegerRtPriority)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_FIFO
    priority: high
    affinity: []
non_ros_threads: []
)YAML");
  try {
    parse(y);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'priority' must be an integer"), std::string::npos) << what;
    EXPECT_NE(what.find("id=my_cbg"), std::string::npos) << what;
  }
}

TEST(ParseConfig, RejectsMissingPriorityOnRtPolicy)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_FIFO
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsRtPriorityOutOfRange)
{
  for (const char * bad_priority : {"0", "100", "-1"}) {
    auto y = yaml_from_str(("callback_groups:\n"
                            "  - id: my_cbg\n"
                            "    domain_id: 0\n"
                            "    policy: SCHED_FIFO\n"
                            "    priority: " +
                            std::string(bad_priority) +
                            "\n"
                            "    affinity: []\n"
                            "non_ros_threads: []\n")
                             .c_str());
    EXPECT_THROW(parse(y), std::runtime_error) << "priority=" << bad_priority;
  }
}

TEST(ParseConfig, FallsBackToDefaultDomainId)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].domain_id, kTestDefaultDomain);
}

TEST(ParseConfig, ParsesSchedDeadline)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: dl_cbg
    domain_id: 0
    policy: SCHED_DEADLINE
    runtime: 1000000
    period: 5000000
    deadline: 5000000
    affinity: [0]
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].attrs.policy, acie::SchedPolicy::Deadline);
  EXPECT_EQ(config.callback_groups[0].attrs.deadline.runtime, 1000000u);
  EXPECT_EQ(config.callback_groups[0].attrs.deadline.period, 5000000u);
  EXPECT_EQ(config.callback_groups[0].attrs.deadline.deadline, 5000000u);
}

TEST(ParseConfig, AcceptsDeadlineParamsBeyond32Bits)
{
  // sched_attr carries nanoseconds in 64-bit fields; a 5 s period exceeds
  // 2^32 ns.
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: dl_cbg
    domain_id: 0
    policy: SCHED_DEADLINE
    runtime: 1000000000
    period: 5000000000
    deadline: 5000000000
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].attrs.deadline.period, 5000000000u);
}

TEST(ParseConfig, RejectsUnknownPolicyOnCallbackGroup)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: bad
    domain_id: 0
    policy: SCHED_BOGUS
    priority: 0
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsUnknownPolicyOnNonRosThread)
{
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: bad_worker
    policy: NOT_A_POLICY
    priority: 0
    affinity: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsSchedDeadlineMissingFields)
{
  expect_error(
    R"YAML(
callback_groups:
  - id: dl_cbg
    domain_id: 0
    policy: SCHED_DEADLINE
    affinity: []
)YAML",
    "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for id=dl_cbg");
  expect_error(
    R"YAML(
non_ros_threads:
  - name: dl_worker
    policy: SCHED_DEADLINE
    runtime: 1000000
)YAML",
    "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for name=dl_worker");
}

TEST(ParseConfig, RejectsMalformedSchedDeadlineFields)
{
  // Same rules as kernel_threads: non-negative decimal digits only.
  expect_error(
    R"YAML(
callback_groups:
  - id: dl_cbg
    policy: SCHED_DEADLINE
    runtime: -5
    period: 5000000
    deadline: 5000000
)YAML",
    "'runtime' must be a non-negative decimal integer for id=dl_cbg");
  // yaml-cpp would read "0x10" as 16; only plain decimal digits are valid.
  expect_error(
    R"YAML(
non_ros_threads:
  - name: dl_worker
    policy: SCHED_DEADLINE
    runtime: 1000000
    period: 5000000
    deadline: 0x10
)YAML",
    "'deadline' must be a non-negative decimal integer for name=dl_worker");
}

TEST(ParseConfig, ParsesZeroPaddedDeadlineFieldAsBase10)
{
  // yaml-cpp's base auto-detection would read "0500000" as octal; a
  // zero-padded column must mean decimal.
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: dl_cbg
    policy: SCHED_DEADLINE
    runtime: 0500000
    period: 5000000
    deadline: 5000000
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].attrs.deadline.runtime, 500000u);
}

TEST(ParseConfig, RequiresPolicy)
{
  // Unlike kernel_threads, an announced thread's entry has no "leave the
  // policy alone" meaning, so a missing or null policy is an error.
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    nice: 0\n", "'policy' is required for id=my_cbg");
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    policy: ~\n    nice: 0\n",
    "'policy' is required for id=my_cbg");
  expect_error(
    "non_ros_threads:\n  - name: worker\n    nice: 0\n", "'policy' is required for name=worker");
}

TEST(ParseConfig, RejectsNonStringPolicy)
{
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    policy: [a, b]\n",
    "'policy' must be a string for id=my_cbg");
}

TEST(ParseConfig, ParsesNonRosThread)
{
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: worker
    policy: SCHED_RR
    priority: 30
    affinity: [0]
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.non_ros_threads.size(), 1u);
  EXPECT_EQ(config.non_ros_threads[0].name, "worker");
  EXPECT_EQ(config.non_ros_threads[0].attrs.policy, acie::SchedPolicy::Rr);
  EXPECT_EQ(config.non_ros_threads[0].attrs.rt_priority, 30);
}

TEST(ParseConfig, RejectsDuplicateCallbackGroupKey)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, AllowsSameIdInDifferentDomains)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - id: cg
    domain_id: 1
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 2u);
}

TEST(ParseConfig, RejectsDuplicateNonRosThreadName)
{
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: t
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - name: t
    policy: SCHED_OTHER
    nice: 0
    affinity: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

// ---------- wildcard ("<node name>/*") callback-group ids ----------

TEST(ParseConfig, ParsesWildcardCallbackGroupId)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /perception/lidar_node/*
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    affinity: [0]
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_TRUE(config.callback_groups[0].is_wildcard());
  EXPECT_EQ(config.callback_groups[0].wildcard_prefix(), "/perception/lidar_node");
  EXPECT_EQ(config.callback_groups[0].id, "/perception/lidar_node/*");  // kept as written
}

TEST(ParseConfig, RejectsWildcardWithEmptyNodePart)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsStrayAsteriskInId)
{
  for (const char * bad_id :
       {"/node*", "/node/**", "/node/*x", "/a/*/b", "*", "/node/*@Waitable"}) {
    auto y = yaml_from_str(("callback_groups:\n"
                            "  - id: \"" +
                            std::string(bad_id) +
                            "\"\n"
                            "    domain_id: 0\n"
                            "    policy: SCHED_OTHER\n"
                            "    nice: 0\n"
                            "    affinity: []\n"
                            "non_ros_threads: []\n")
                             .c_str());
    EXPECT_THROW(parse(y), std::runtime_error) << "id=" << bad_id;
  }
}

TEST(ParseConfig, RejectsAtSignInWildcardPrefix)
{
  // A full callback-group id copied from the template with "/*" appended.
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /node@Timer(100)/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, RejectsDuplicateWildcardKey)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);
}

TEST(ParseConfig, AllowsSameWildcardInDifferentDomains)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - id: /node/*
    domain_id: 1
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 2u);
}

TEST(ParseConfig, AllowsExactAndWildcardForSameNode)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
  - id: /node@Timer(100)
    domain_id: 0
    policy: SCHED_FIFO
    priority: 80
    affinity: []
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 2u);
  EXPECT_TRUE(config.callback_groups[0].is_wildcard());
  EXPECT_FALSE(config.callback_groups[1].is_wildcard());
}

TEST(ParseConfig, NonRosThreadNameEndingInSlashStarStaysExact)
{
  // Wildcards are a callback_groups-only feature; non_ros_threads names are
  // opaque strings matched exactly, even when they happen to end in "/*".
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: worker/*
    policy: SCHED_OTHER
    nice: 0
    affinity: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.non_ros_threads.size(), 1u);
  EXPECT_EQ(config.non_ros_threads[0].name, "worker/*");
}

// ---------- affinity validation ----------

TEST(ParseConfig, NormalizesAffinityToSortedUnique)
{
  // parse_affinity bounds values by the machine CPU count, so only CPUs 0
  // and 1 are used to keep this test valid on small CI machines.
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    affinity: [1, 0, 1]
non_ros_threads:
  - name: my_thread
    policy: SCHED_OTHER
    nice: 0
    affinity: [1, 1, 0]
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].attrs.affinity, (std::vector<int>{0, 1}));
  ASSERT_EQ(config.non_ros_threads.size(), 1u);
  EXPECT_EQ(config.non_ros_threads[0].attrs.affinity, (std::vector<int>{0, 1}));
}

TEST(ParseConfig, TreatsAbsentOrNullAffinityAsUnmanaged)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: no_key
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
  - id: null_value
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    affinity: ~
non_ros_threads: []
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 2u);
  EXPECT_TRUE(config.callback_groups[0].attrs.affinity.empty());
  EXPECT_TRUE(config.callback_groups[1].attrs.affinity.empty());
}

TEST(ParseConfig, RejectsAffinityCpuOutOfRange)
{
  // CPU_SET(3) / sched_setaffinity(2) would silently ignore all of these,
  // shrinking the mask without any error report. The valid CPU 0 in front
  // checks that it does not mask the error.
  const long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
  for (const std::string & bad_affinity :
       {std::string("[-1]"), "[0, " + std::to_string(num_cpus) + "]",
        "[0, " + std::to_string(CPU_SETSIZE) + "]"}) {
    auto y = yaml_from_str(("callback_groups:\n"
                            "  - id: my_cbg\n"
                            "    domain_id: 0\n"
                            "    policy: SCHED_FIFO\n"
                            "    priority: 50\n"
                            "    affinity: " +
                            bad_affinity +
                            "\n"
                            "non_ros_threads: []\n")
                             .c_str());
    EXPECT_THROW(parse(y), std::runtime_error) << "affinity=" << bad_affinity;
  }
}

TEST(ParseConfig, AcceptsHighestValidAffinityCpu)
{
  // Pins the accept side of the [0, min(CPU_SETSIZE, num_cpus)) boundary.
  const int max_cpu =
    static_cast<int>(std::min<long>(CPU_SETSIZE, sysconf(_SC_NPROCESSORS_CONF))) - 1;
  auto y = yaml_from_str(("callback_groups:\n"
                          "  - id: my_cbg\n"
                          "    domain_id: 0\n"
                          "    policy: SCHED_FIFO\n"
                          "    priority: 50\n"
                          "    affinity: [" +
                          std::to_string(max_cpu) +
                          "]\n"
                          "non_ros_threads: []\n")
                           .c_str());
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].attrs.affinity, (std::vector<int>{max_cpu}));
}

TEST(ParseConfig, RejectsScalarAffinity)
{
  // A scalar iterates zero times and would otherwise silently mean
  // "no affinity".
  for (const char * bad_affinity : {"2", "\"0-3\""}) {
    auto y = yaml_from_str(("callback_groups: []\n"
                            "non_ros_threads:\n"
                            "  - name: my_thread\n"
                            "    policy: SCHED_OTHER\n"
                            "    nice: 0\n"
                            "    affinity: " +
                            std::string(bad_affinity) + "\n")
                             .c_str());
    EXPECT_THROW(parse(y), std::runtime_error) << "affinity=" << bad_affinity;
  }
}

TEST(ParseConfig, ReportsEntryOnNonIntegerAffinityElement)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    affinity: [0, all]
non_ros_threads: []
)YAML");
  try {
    parse(y);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'affinity' must contain only integers"), std::string::npos) << what;
    EXPECT_NE(what.find("id=my_cbg"), std::string::npos) << what;
  }
}

// ---------- extract_node_part ----------

TEST(ExtractNodePart, SplitsAtFirstAtSign)
{
  EXPECT_EQ(acie::extract_node_part("/ns/node@Timer(1000000)@Subscription(/topic)"), "/ns/node");
  EXPECT_EQ(acie::extract_node_part("/plain_node"), "/plain_node");
  EXPECT_EQ(acie::extract_node_part("/node@"), "/node");
  EXPECT_EQ(acie::extract_node_part("@Timer(1)"), "");
  EXPECT_EQ(acie::extract_node_part(""), "");
}

// ---------- kernel_threads ----------

TEST(ParseKernelThreads, MissingOrNullSectionYieldsEmpty)
{
  EXPECT_TRUE(parse(yaml_from_str("callback_groups: []\n")).kernel_threads.empty());
  EXPECT_TRUE(parse(yaml_from_str("kernel_threads: ~\n")).kernel_threads.empty());
  EXPECT_TRUE(parse(yaml_from_str("kernel_threads: []\n")).kernel_threads.empty());
}

TEST(ParseKernelThreads, ParsesPolicyPriorityAffinity)
{
  // CPUs 0/1 only, to keep this test valid on small CI machines.
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: agnocast_exit_w
    policy: SCHED_FIFO
    priority: 10
    affinity: [0, 1]
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].comm, "agnocast_exit_w");
  EXPECT_EQ(result[0].attrs.policy, acie::SchedPolicy::Fifo);
  EXPECT_EQ(result[0].attrs.rt_priority, 10);
  EXPECT_EQ(result[0].attrs.affinity, (std::vector<int>{0, 1}));
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseKernelThreads, ParsesCfsPolicyWithNice)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: nfsd
    policy: SCHED_OTHER
    nice: -10
    affinity: ~
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].attrs.policy, acie::SchedPolicy::Other);
  EXPECT_EQ(result[0].attrs.nice, -10);
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseKernelThreads, AllNullEntryIsUnmanaged)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: ~
    nice: ~
    priority: ~
    affinity: ~
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_FALSE(result[0].attrs.policy.has_value());
  EXPECT_TRUE(result[0].attrs.affinity.empty());
  EXPECT_FALSE(result[0].is_managed());
}

TEST(ParseKernelThreads, UnmanageableSentinelEqualsNull)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: ksoftirqd/0
    policy: UNMANAGEABLE
    nice: UNMANAGEABLE
    priority: UNMANAGEABLE
    affinity: UNMANAGEABLE
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_FALSE(result[0].attrs.policy.has_value());
  EXPECT_TRUE(result[0].attrs.affinity.empty());
  EXPECT_FALSE(result[0].is_managed());
}

TEST(ParseKernelThreads, AffinityOnlyEntryIsManaged)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: agnocast_exit_w
    policy: ~
    priority: ~
    affinity: [1]
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_FALSE(result[0].attrs.policy.has_value());
  EXPECT_EQ(result[0].attrs.affinity, (std::vector<int>{1}));
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseKernelThreads, NormalizesAffinityToSortedUnique)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: nfsd
    policy: ~
    priority: ~
    affinity: [1, 0, 1]
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].attrs.affinity, (std::vector<int>{0, 1}));
}

TEST(ParseKernelThreads, RejectsScalarAndOutOfRangeAffinity)
{
  // A scalar (e.g. a cpu-list string copied from a scan) would otherwise
  // silently mean "leave alone".
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: ~
    priority: ~
    affinity: 0-3
)YAML",
    "'affinity' must be a list");
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: ~
    priority: ~
    affinity: [-1]
)YAML",
    "'affinity' CPU -1 must be in");
}

TEST(ParseKernelThreads, PolicyWithUnmanageableAffinityIsManaged)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: ksoftirqd/0
    policy: SCHED_FIFO
    priority: 5
    affinity: UNMANAGEABLE
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  ASSERT_TRUE(result[0].attrs.policy.has_value());
  EXPECT_TRUE(result[0].attrs.affinity.empty());
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseKernelThreads, ParsesSchedDeadline)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 1000000
    period: 5000000
    deadline: 5000000
    affinity: ~
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].attrs.policy, acie::SchedPolicy::Deadline);
  EXPECT_EQ(result[0].attrs.deadline.runtime, 1000000u);
  EXPECT_EQ(result[0].attrs.deadline.period, 5000000u);
  EXPECT_EQ(result[0].attrs.deadline.deadline, 5000000u);
}

TEST(ParseKernelThreads, AcceptsDeadlineParamsBeyond32Bits)
{
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 1000000000
    period: 5000000000
    deadline: 5000000000
    affinity: ~
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].attrs.deadline.period, 5000000000u);
}

TEST(ParseKernelThreads, LowercaseSentinelIsNotRecognized)
{
  // Only the exact uppercase sentinel disengages an attribute; anything else
  // must fall through to the normal validation (here: unknown policy). No
  // other attribute key is set, so a case-insensitive sentinel match would
  // parse cleanly and fail the test.
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: unmanageable
    affinity: ~
)YAML",
    "Unknown scheduling policy 'unmanageable'");
}

TEST(ParseKernelThreads, RejectsMissingOrEmptyComm)
{
  expect_error(
    R"YAML(
kernel_threads:
  - policy: ~
    priority: ~
    affinity: ~
)YAML",
    "is missing a non-empty 'comm'");
  expect_error(
    R"YAML(
kernel_threads:
  - comm: ""
    policy: ~
    priority: ~
    affinity: ~
)YAML",
    "is missing a non-empty 'comm'");
}

TEST(ParseKernelThreads, RejectsNonStringComm)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: [a, b]
    policy: ~
    priority: ~
    affinity: ~
)YAML",
    "'comm' must be a string");
}

TEST(ParseKernelThreads, RejectsKworkerComm)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: kworker/0:0H-events_highpri
    policy: ~
    priority: ~
    affinity: ~
)YAML",
    "kworker comms are ephemeral");
}

TEST(ParseKernelThreads, AcceptsCommLongerThan15Chars)
{
  // Since Linux 5.17 /proc reports a kthread's full name untruncated (e.g.
  // the kmod's "agnocast_exit_worker"), so the scanner returns comms longer
  // than TASK_COMM_LEN - 1 and they must round-trip through the parser.
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: agnocast_exit_worker
    policy: ~
    priority: ~
    affinity: [0]
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].comm, "agnocast_exit_worker");
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseKernelThreads, RejectsDuplicateComm)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: ~
    priority: ~
    affinity: ~
  - comm: nfsd
    policy: ~
    priority: ~
    affinity: ~
)YAML",
    "Duplicate kernel_thread entry: comm=nfsd");
}

TEST(ParseKernelThreads, RejectsUnknownPolicy)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: SCHED_BOGUS
    priority: 0
    affinity: ~
)YAML",
    "Unknown scheduling policy 'SCHED_BOGUS'");
}

TEST(ParseKernelThreads, RejectsPriorityWithoutPolicy)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: ~
    priority: 5
    affinity: ~
)YAML",
    "'priority' requires 'policy'");
}

TEST(ParseKernelThreads, RejectsNiceWithoutPolicy)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: ~
    nice: 5
    affinity: ~
)YAML",
    "'nice' requires 'policy'");
}

TEST(ParseKernelThreads, RejectsCfsPolicyWithoutNice)
{
  // As in callback_groups, a CFS policy takes 'nice'; 'priority' does not
  // satisfy the requirement.
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: SCHED_OTHER
    priority: 5
    affinity: ~
)YAML",
    "requires 'nice'");
}

TEST(ParseKernelThreads, RejectsOutOfRangeNiceAndRtPriority)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: SCHED_OTHER
    nice: 50
    affinity: ~
)YAML",
    "'nice' must be in [-20, 19]");
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: SCHED_FIFO
    priority: 150
    affinity: ~
)YAML",
    "'priority' must be in [1, 99]");
}

TEST(ParseKernelThreads, RejectsPolicyWithoutPriority)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: SCHED_FIFO
    priority: ~
    affinity: ~
)YAML",
    "requires 'priority'");
  // The sentinel counts as unset exactly like null does, and the message
  // says so because the key is visibly present.
  expect_error(
    R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: SCHED_FIFO
    priority: UNMANAGEABLE
    affinity: ~
)YAML",
    "requires 'priority' for comm=rcu_preempt (UNMANAGEABLE counts as unset)");
}

TEST(ParseKernelThreads, RejectsDeadlineFieldsWithoutPolicy)
{
  // The full SCHED_DEADLINE triple with 'policy' forgotten would otherwise
  // parse cleanly as unmanaged, the same silently dead configuration the
  // nice/priority guard rejects.
  expect_error(
    R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: ~
    runtime: 1000000
    period: 5000000
    deadline: 5000000
    affinity: ~
)YAML",
    "'runtime' requires 'policy'");
}

TEST(ParseKernelThreads, RejectsSchedDeadlineMissingFields)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 1000000
    affinity: ~
)YAML",
    "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline'");
}

TEST(ParseKernelThreads, RejectsMalformedSchedDeadlineFields)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: -5
    period: 5000000
    deadline: 5000000
    affinity: ~
)YAML",
    "'runtime' must be a non-negative decimal integer for comm=dl_thread");
  expect_error(
    R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 1000000
    period: not_a_number
    deadline: 5000000
    affinity: ~
)YAML",
    "'period' must be a non-negative decimal integer for comm=dl_thread");
  // yaml-cpp would read "0x10" as 16; only plain decimal digits are valid.
  expect_error(
    R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 1000000
    period: 5000000
    deadline: 0x10
    affinity: ~
)YAML",
    "'deadline' must be a non-negative decimal integer for comm=dl_thread");
}

TEST(ParseKernelThreads, ParsesZeroPaddedDeadlineFieldAsBase10)
{
  // yaml-cpp's base auto-detection would read "0500000" as octal; a
  // zero-padded column must mean decimal.
  auto y = yaml_from_str(R"YAML(
kernel_threads:
  - comm: dl_thread
    policy: SCHED_DEADLINE
    runtime: 0500000
    period: 5000000
    deadline: 5000000
    affinity: ~
)YAML");
  const auto result = parse(y).kernel_threads;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].attrs.deadline.runtime, 500000u);
}

TEST(ParseKernelThreads, RejectsNonListSection)
{
  // A scalar or map section would otherwise silently parse as empty.
  expect_error("kernel_threads: oops\n", "'kernel_threads' must be a list");
  expect_error("kernel_threads:\n  comm: nfsd\n", "'kernel_threads' must be a list");
}

TEST(ParseKernelThreads, RejectsScalarListEntry)
{
  // A plausible shorthand (a list of bare comms) must fail with the entry
  // diagnostic, not a raw yaml-cpp BadSubscript.
  expect_error("kernel_threads: [nfsd]\n", "entry #0 must be a mapping");
}

// ---------- irqs ----------

TEST(ParseIrqs, MissingOrNullSectionYieldsEmpty)
{
  EXPECT_TRUE(parse(yaml_from_str("callback_groups: []\n")).irqs.empty());
  EXPECT_TRUE(parse(yaml_from_str("irqs: ~\n")).irqs.empty());
  EXPECT_TRUE(parse(yaml_from_str("irqs: []\n")).irqs.empty());
}

TEST(ParseIrqs, ParsesFilledAffinityAndName)
{
  auto y = yaml_from_str(R"YAML(
irqs:
  - irq: 103
    name: nvidia
    affinity: [1, 0]
)YAML");
  const auto result = parse(y).irqs;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].irq, 103);
  EXPECT_EQ(result[0].name, "nvidia");
  EXPECT_EQ(result[0].affinity, (std::vector<int>{0, 1}));
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseIrqs, NullOrUnmanageableAffinityIsUnmanaged)
{
  auto y = yaml_from_str(R"YAML(
irqs:
  - irq: 0
    name: timer
    affinity: UNMANAGEABLE
  - irq: 1
    name: i8042
    affinity: ~
)YAML");
  const auto result = parse(y).irqs;
  ASSERT_EQ(result.size(), 2u);
  EXPECT_FALSE(result[0].is_managed());
  EXPECT_FALSE(result[1].is_managed());
}

TEST(ParseIrqs, MissingNameMeansNoVerification)
{
  auto y = yaml_from_str(R"YAML(
irqs:
  - irq: 42
    affinity: [1]
)YAML");
  const auto result = parse(y).irqs;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result[0].name.empty());
  EXPECT_TRUE(result[0].is_managed());
}

TEST(ParseIrqs, RejectsMissingOrNegativeOrNonIntIrq)
{
  expect_error(
    R"YAML(
irqs:
  - name: orphan
    affinity: ~
)YAML",
    "is missing a non-negative integer 'irq'");
  expect_error(
    R"YAML(
irqs:
  - irq: -1
    affinity: ~
)YAML",
    "'irq' must be a non-negative decimal integer, got '-1'");
  expect_error(
    R"YAML(
irqs:
  - irq: not_a_number
    affinity: ~
)YAML",
    "'irq' must be a non-negative decimal integer, got 'not_a_number'");
}

TEST(ParseIrqs, ParsesZeroPaddedIrqAsBase10)
{
  // yaml-cpp's as<int>() would read "010" as octal 8 and silently target a
  // different interrupt; a zero-padded column must mean decimal 10.
  auto y = yaml_from_str(R"YAML(
irqs:
  - irq: 010
    affinity: ~
)YAML");
  const auto result = parse(y).irqs;
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].irq, 10);
}

TEST(ParseIrqs, RejectsHexIrq)
{
  // yaml-cpp would read "0x10" as 16; only plain decimal digits are valid.
  expect_error(
    R"YAML(
irqs:
  - irq: 0x10
    affinity: ~
)YAML",
    "'irq' must be a non-negative decimal integer, got '0x10'");
}

TEST(ParseIrqs, RejectsDuplicateIrq)
{
  expect_error(
    R"YAML(
irqs:
  - irq: 5
    affinity: ~
  - irq: 5
    affinity: ~
)YAML",
    "Duplicate irq entry: irq=5");
}

TEST(ParseIrqs, RejectsScalarAndOutOfRangeAffinity)
{
  // A scalar (e.g. a cpu-list string copied from a scan) would otherwise
  // silently mean "leave alone".
  expect_error(
    R"YAML(
irqs:
  - irq: 10
    affinity: 0-3
)YAML",
    "'affinity' must be a list");
  expect_error(
    R"YAML(
irqs:
  - irq: 10
    affinity: [-1]
)YAML",
    "'affinity' CPU -1 must be in");
}

TEST(ParseIrqs, RejectsNonListSection)
{
  expect_error("irqs: oops\n", "'irqs' must be a list");
  expect_error("irqs:\n  irq: 5\n", "'irqs' must be a list");
}

TEST(ParseIrqs, RejectsScalarListEntry)
{
  // A plausible shorthand (a list of bare IRQ numbers) must fail with the
  // entry diagnostic, not a raw yaml-cpp BadSubscript.
  expect_error("irqs: [42]\n", "entry #0 must be a mapping");
}

// ---------- all sections ----------

TEST(ParseConfig, ParsesAllFourSectionsIntoTheirVectors)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads:
  - name: worker
    policy: SCHED_RR
    priority: 30
    affinity: [0]
kernel_threads:
  - comm: agnocast_exit_w
    policy: SCHED_FIFO
    priority: 10
    affinity: [0]
irqs:
  - irq: 103
    name: nvidia
    affinity: [0, 1]
)YAML");
  const auto config = parse(y);
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].id, "my_cbg");
  ASSERT_EQ(config.non_ros_threads.size(), 1u);
  EXPECT_EQ(config.non_ros_threads[0].name, "worker");
  EXPECT_EQ(config.non_ros_threads[0].attrs.policy, acie::SchedPolicy::Rr);
  EXPECT_EQ(config.non_ros_threads[0].attrs.rt_priority, 30);
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_EQ(config.kernel_threads[0].comm, "agnocast_exit_w");
  ASSERT_EQ(config.irqs.size(), 1u);
  EXPECT_EQ(config.irqs[0].irq, 103);
}

TEST(ParseConfig, MissingOrNullSectionsYieldEmpty)
{
  for (const char * yaml :
       {"{}\n", "callback_groups: ~\nnon_ros_threads: ~\nkernel_threads: ~\nirqs: ~\n"}) {
    const auto config = parse(yaml_from_str(yaml));
    EXPECT_TRUE(config.callback_groups.empty()) << yaml;
    EXPECT_TRUE(config.non_ros_threads.empty()) << yaml;
    EXPECT_TRUE(config.kernel_threads.empty()) << yaml;
    EXPECT_TRUE(config.irqs.empty()) << yaml;
  }
}

TEST(ParseConfig, UnmanageableSentinelIsNotRecognized)
{
  // The sentinel is scoped to kernel_threads/irqs (the tool writes it there);
  // in the hand-written sections it must fail like any other invalid value,
  // not silently mean "leave the attribute alone".
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    policy: SCHED_FIFO
    priority: 50
    affinity: UNMANAGEABLE
non_ros_threads: []
)YAML");
  EXPECT_THROW(parse(y), std::runtime_error);

  auto y2 = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: my_thread
    policy: SCHED_OTHER
    nice: UNMANAGEABLE
    affinity: ~
)YAML");
  EXPECT_THROW(parse(y2), std::runtime_error);
}
