#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

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
}

TEST(ParseYaml, FallsBackToDefaultDomainId)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg
    policy: SCHED_OTHER
    priority: 0
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
    affinity: [2]
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
    affinity: [3]
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(nrt.size(), 1u);
  EXPECT_EQ(nrt[0].thread_str, "worker");
  EXPECT_EQ(nrt[0].policy, "SCHED_RR");
  EXPECT_EQ(nrt[0].priority, 30);
}

TEST(ParseYaml, StripsTrailingWaitableSuffix)
{
  auto y = yaml_from_str(R"YAML(
callback_groups:
  - id: my_cbg@Waitable@Waitable
    domain_id: 0
    policy: SCHED_OTHER
    priority: 0
    affinity: []
non_ros_threads: []
)YAML");
  std::vector<acie::ThreadConfig> cb, nrt;
  acie::parse_yaml(y, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].thread_str, "my_cbg");
}

TEST(ParseYaml, ClearsOutputVectorsOnReparse)
{
  auto y1 = yaml_from_str(R"YAML(
callback_groups:
  - id: alpha
    domain_id: 0
    policy: SCHED_OTHER
    priority: 0
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
    priority: 0
    affinity: []
non_ros_threads: []
)YAML");
  acie::parse_yaml(y2, kTestDefaultDomain, cb, nrt);
  ASSERT_EQ(cb.size(), 1u);
  EXPECT_EQ(cb[0].thread_str, "beta");
}
