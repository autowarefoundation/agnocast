#include "agnocast_cie_thread_configurator/sched_policy.hpp"

#include <gtest/gtest.h>
#include <linux/sched.h>
#include <sched.h>

#include <optional>

namespace acie = agnocast_cie_thread_configurator;

namespace
{
constexpr acie::SchedPolicy kAllPolicies[] = {
  acie::SchedPolicy::Other, acie::SchedPolicy::Batch, acie::SchedPolicy::Idle,
  acie::SchedPolicy::Fifo,  acie::SchedPolicy::Rr,    acie::SchedPolicy::Deadline,
};
}  // namespace

TEST(SchedPolicy, NameRoundTrip)
{
  for (const auto policy : kAllPolicies) {
    EXPECT_EQ(acie::parse_sched_policy(acie::to_string(policy)), policy);
  }
  EXPECT_EQ(acie::to_string(acie::SchedPolicy::Other), "SCHED_OTHER");
  EXPECT_EQ(acie::to_string(acie::SchedPolicy::Deadline), "SCHED_DEADLINE");
}

TEST(SchedPolicy, RejectsNamesThatAreNotExactMatches)
{
  EXPECT_EQ(acie::parse_sched_policy("sched_fifo"), std::nullopt);
  EXPECT_EQ(acie::parse_sched_policy("FIFO"), std::nullopt);
  EXPECT_EQ(acie::parse_sched_policy("SCHED_FIFO "), std::nullopt);
  EXPECT_EQ(acie::parse_sched_policy("UNKNOWN(4)"), std::nullopt);
  EXPECT_EQ(acie::parse_sched_policy(""), std::nullopt);
}

TEST(SchedPolicy, KernelConstantRoundTrip)
{
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Other), SCHED_OTHER);
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Batch), SCHED_BATCH);
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Idle), SCHED_IDLE);
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Fifo), SCHED_FIFO);
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Rr), SCHED_RR);
  EXPECT_EQ(acie::to_kernel_policy(acie::SchedPolicy::Deadline), SCHED_DEADLINE);
  for (const auto policy : kAllPolicies) {
    EXPECT_EQ(acie::from_kernel_policy(acie::to_kernel_policy(policy)), policy);
  }
}

TEST(SchedPolicy, RejectsKernelValuesWithoutYamlName)
{
  // 4 is SCHED_ISO, reserved but never implemented; -1 and 100 never existed.
  EXPECT_EQ(acie::from_kernel_policy(4), std::nullopt);
  EXPECT_EQ(acie::from_kernel_policy(-1), std::nullopt);
  EXPECT_EQ(acie::from_kernel_policy(100), std::nullopt);
}

TEST(SchedPolicy, IsCfsSplitsNiceFromRtPriorityPolicies)
{
  EXPECT_TRUE(acie::is_cfs(acie::SchedPolicy::Other));
  EXPECT_TRUE(acie::is_cfs(acie::SchedPolicy::Batch));
  EXPECT_TRUE(acie::is_cfs(acie::SchedPolicy::Idle));
  EXPECT_FALSE(acie::is_cfs(acie::SchedPolicy::Fifo));
  EXPECT_FALSE(acie::is_cfs(acie::SchedPolicy::Rr));
  EXPECT_FALSE(acie::is_cfs(acie::SchedPolicy::Deadline));
}

TEST(SchedPolicy, NamesListsEveryPolicyInEnumeratorOrder)
{
  EXPECT_EQ(
    acie::sched_policy_names(),
    "SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, SCHED_DEADLINE");
}
