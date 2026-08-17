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
}  // namespace

// ---------- parse_yaml ----------

TEST(ParseYaml, ParsesEmptyConfig)
{
  auto y = yaml_from_str("callback_groups: []\nnon_ros_threads: []\n");
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  EXPECT_TRUE(cb.empty());
  EXPECT_TRUE(nrt.empty());
}

TEST(ParseYaml, ParsesCallbackGroupSchedFifo)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].thread_str, "my_cbg");
  EXPECT_EQ(cb[0].domain_id, 3u);
  EXPECT_EQ(cb[0].policy, "SCHED_FIFO");
  EXPECT_EQ(cb[0].priority, 50);
  EXPECT_EQ(cb[0].affinity, (std::vector<int>{0, 1}));
  EXPECT_EQ(cb[0].thread_id, -1);
  EXPECT_FALSE(cb[0].applied);
  EXPECT_FALSE(cb[0].is_wildcard());
}

TEST(ParseYaml, ParsesNiceForCfsPolicies)
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
    std::vector<acie::ThreadConfig> cb, nrt;
    ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt)) << policy;
    ASSERT_EQ(cb.size(), 1u);
    EXPECT_EQ(cb[0].nice, -10) << policy;
    EXPECT_EQ(cb[0].priority, 0) << policy;
  }
}

TEST(ParseYaml, IgnoresStrayKeyOfTheOtherPolicyClass)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 2u);
  EXPECT_EQ(cb[0].nice, -5);
  EXPECT_EQ(cb[0].priority, 0);
  EXPECT_EQ(cb[1].priority, 50);
  EXPECT_EQ(cb[1].nice, 0);
}

TEST(ParseYaml, RejectsMissingNiceOnSchedOther)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_OTHER
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsNiceOutOfRange)
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
    std::vector<acie::ThreadConfig> cb, nrt;
    EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error)
      << "nice=" << bad_nice;
  }
}

TEST(ParseYaml, TreatsNullNiceAsMissing)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  try {
    acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("requires 'nice'"), std::string::npos) << e.what();
  }
}

TEST(ParseYaml, ReportsEntryOnNonIntegerNice)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  try {
    acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'nice' must be an integer"), std::string::npos) << what;
    EXPECT_NE(what.find("id=my_cbg"), std::string::npos) << what;
  }
}

TEST(ParseYaml, ReportsEntryOnNonIntegerRtPriority)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  try {
    acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("'priority' must be an integer"), std::string::npos) << what;
    EXPECT_NE(what.find("id=my_cbg"), std::string::npos) << what;
  }
}

TEST(ParseYaml, RejectsMissingPriorityOnRtPolicy)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 0
    policy: SCHED_FIFO
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsRtPriorityOutOfRange)
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
    std::vector<acie::ThreadConfig> cb, nrt;
    EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error)
      << "priority=" << bad_priority;
  }
}

TEST(ParseYaml, FallsBackToDefaultDomainId)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].domain_id, kTestDefaultDomain);
}

TEST(ParseYaml, ParsesSchedDeadline)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].policy, "SCHED_DEADLINE");
  EXPECT_EQ(cb[0].runtime, 1000000u);
  EXPECT_EQ(cb[0].period, 5000000u);
  EXPECT_EQ(cb[0].deadline, 5000000u);
}

TEST(ParseYaml, RejectsUnknownPolicyOnCallbackGroup)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsUnknownPolicyOnNonRosThread)
{
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: bad_worker
    policy: NOT_A_POLICY
    priority: 0
    affinity: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsSchedDeadlineMissingRuntimeField)
{
  // SCHED_DEADLINE requires runtime/period/deadline. parse_yaml calls
  // .as<unsigned int>() on a missing node, which makes yaml-cpp throw
  // YAML::TypedBadConversion (a subclass of YAML::Exception, which is
  // a subclass of std::runtime_error). The test catches the most general
  // shape so it does not couple to the precise yaml-cpp exception type.
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: dl_cbg
    domain_id: 0
    policy: SCHED_DEADLINE
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, ParsesNonRosThread)
{
  auto y = yaml_from_str(R"YAML(
callback_groups: []
non_ros_threads:
  - name: worker
    policy: SCHED_RR
    priority: 30
    affinity: [0]
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(nrt.size(), 1u);
  EXPECT_EQ(nrt[0].thread_str, "worker");
  EXPECT_EQ(nrt[0].policy, "SCHED_RR");
  EXPECT_EQ(nrt[0].priority, 30);
}

TEST(ParseYaml, ClearsOutputVectorsOnReparse)
{
  auto y1 = yaml_from_str(R"YAML(
callback_groups:
  - id: alpha
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y1, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);

  auto y2 = yaml_from_str(R"YAML(
callback_groups:
  - id: beta
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
    affinity: []
non_ros_threads: []
)YAML");
  acie::parse_yaml(y2, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].thread_str, "beta");
}

TEST(ParseYaml, RejectsDuplicateCallbackGroupKey)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, AllowsSameIdInDifferentDomains)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 2u);
}

TEST(ParseYaml, RejectsDuplicateNonRosThreadName)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

// ---------- wildcard ("<node name>/*") callback-group ids ----------

TEST(ParseYaml, ParsesWildcardCallbackGroupId)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_TRUE(cb[0].is_wildcard());
  EXPECT_EQ(cb[0].wildcard_prefix(), "/perception/lidar_node");
  EXPECT_EQ(cb[0].thread_str, "/perception/lidar_node/*");  // kept as written
  EXPECT_TRUE(cb[0].matched_tids.empty());
}

TEST(ParseYaml, RejectsWildcardWithEmptyNodePart)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsStrayAsteriskInId)
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
    std::vector<acie::ThreadConfig> cb, nrt;
    EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error)
      << "id=" << bad_id;
  }
}

TEST(ParseYaml, RejectsAtSignInWildcardPrefix)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, RejectsDuplicateWildcardKey)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error);
}

TEST(ParseYaml, AllowsSameWildcardInDifferentDomains)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 2u);
}

TEST(ParseYaml, AllowsExactAndWildcardForSameNode)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 2u);
  EXPECT_TRUE(cb[0].is_wildcard());
  EXPECT_FALSE(cb[1].is_wildcard());
}

TEST(ParseYaml, NonRosThreadNameEndingInSlashStarStaysExact)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(nrt.size(), 1u);
  EXPECT_EQ(nrt[0].thread_str, "worker/*");
}

// ---------- affinity validation ----------

TEST(ParseYaml, NormalizesAffinityToSortedUnique)
{
  // Only CPUs 0 and 1 are used so the test also passes on small CI machines
  // now that parse_affinity bounds values by the actual CPU count.
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].affinity, (std::vector<int>{0, 1}));
  ASSERT_EQ(nrt.size(), 1u);
  EXPECT_EQ(nrt[0].affinity, (std::vector<int>{0, 1}));
}

TEST(ParseYaml, TreatsAbsentOrNullAffinityAsUnmanaged)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 2u);
  EXPECT_TRUE(cb[0].affinity.empty());
  EXPECT_TRUE(cb[1].affinity.empty());
}

TEST(ParseYaml, RejectsAffinityCpuOutOfRange)
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
    std::vector<acie::ThreadConfig> cb, nrt;
    EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error)
      << "affinity=" << bad_affinity;
  }
}

TEST(ParseYaml, AcceptsHighestValidAffinityCpu)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  ASSERT_NO_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt));
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].affinity, (std::vector<int>{max_cpu}));
}

TEST(ParseYaml, RejectsScalarAffinity)
{
  // A scalar iterates zero times, so before validation these were silently
  // treated as "no affinity".
  for (const char * bad_affinity : {"2", "\"0-3\""}) {
    auto y = yaml_from_str(("callback_groups: []\n"
                            "non_ros_threads:\n"
                            "  - name: my_thread\n"
                            "    policy: SCHED_OTHER\n"
                            "    nice: 0\n"
                            "    affinity: " +
                            std::string(bad_affinity) + "\n")
                             .c_str());
    std::vector<acie::ThreadConfig> cb, nrt;
    EXPECT_THROW(acie::parse_yaml(y, kTestDefaultDomain, cb, nrt), std::runtime_error)
      << "affinity=" << bad_affinity;
  }
}

TEST(ParseYaml, ReportsEntryOnNonIntegerAffinityElement)
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
  std::vector<acie::ThreadConfig> cb, nrt;
  try {
    acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
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
