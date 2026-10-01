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

acie::ParsedConfig parse(const std::string & yaml)
{
  return acie::parse_config(YAML::Load(yaml), kTestDefaultDomain);
}

// EXPECT_THROW alone cannot tell WHICH validation fired (every failure path
// derives from std::runtime_error), so the negative tests also match a
// distinguishing fragment of the message.
void expect_error(const std::string & yaml, const std::string & fragment)
{
  try {
    parse(yaml);
    FAIL() << "expected std::runtime_error for:\n" << yaml;
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos)
      << "message: " << e.what() << "\nexpected fragment: " << fragment;
  }
}

// The three sections whose entries go through the shared attribute parser.
// A body placed under each header must be accepted or rejected identically;
// only the entry description in the message differs.
struct ThreadSection
{
  const char * header;
  const char * desc;
  const acie::SchedAttrs & (*attrs)(const acie::ParsedConfig &);
};

const ThreadSection kThreadSections[] = {
  {"callback_groups:\n  - id: x\n", "id=x",
   [](const acie::ParsedConfig & c) -> const acie::SchedAttrs & {
     return c.callback_groups.at(0).attrs;
   }},
  {"non_ros_threads:\n  - name: x\n", "name=x",
   [](const acie::ParsedConfig & c) -> const acie::SchedAttrs & {
     return c.non_ros_threads.at(0).attrs;
   }},
  {"kernel_threads:\n  - comm: x\n", "comm=x",
   [](const acie::ParsedConfig & c) -> const acie::SchedAttrs & {
     return c.kernel_threads.at(0).attrs;
   }},
};

// `body` is the entry's attribute lines, indented four spaces.
acie::SchedAttrs attrs_of(const ThreadSection & section, const std::string & body)
{
  return section.attrs(parse(section.header + body));
}

struct Section
{
  const char * name;
  const char * key;
};

constexpr Section kSections[] = {
  {"callback_groups", "id"},
  {"non_ros_threads", "name"},
  {"kernel_threads", "comm"},
  {"irqs", "irq"},
};
}  // namespace

// ---------- sections ----------

TEST(ParseConfig, MissingOrNullOrEmptySectionsYieldEmpty)
{
  for (const char * yaml :
       {"{}\n", "callback_groups: ~\nnon_ros_threads: ~\nkernel_threads: ~\nirqs: ~\n",
        "callback_groups: []\nnon_ros_threads: []\nkernel_threads: []\nirqs: []\n"}) {
    const auto config = parse(yaml);
    EXPECT_TRUE(config.callback_groups.empty()) << yaml;
    EXPECT_TRUE(config.non_ros_threads.empty()) << yaml;
    EXPECT_TRUE(config.kernel_threads.empty()) << yaml;
    EXPECT_TRUE(config.irqs.empty()) << yaml;
  }
}

TEST(ParseConfig, RejectsNonListSection)
{
  // A scalar or map section would otherwise silently parse as empty.
  for (const auto & section : kSections) {
    const std::string name = section.name;
    expect_error(name + ": oops\n", "'" + name + "' must be a list");
    expect_error(name + ":\n  " + section.key + ": x\n", "'" + name + "' must be a list");
  }
}

TEST(ParseConfig, RejectsScalarListEntry)
{
  // A plausible shorthand (a list of bare ids/comms/IRQ numbers) must fail
  // with the entry diagnostic, not a raw yaml-cpp BadSubscript.
  for (const auto & section : kSections) {
    const std::string name = section.name;
    expect_error(
      name + ": [x]\n", name + " entry #0 must be a mapping (e.g. '- " + section.key + ": ...')");
  }
}

TEST(ParseConfig, ParsesAllFourSectionsIntoTheirVectors)
{
  const auto config = parse(R"YAML(
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

// ---------- shared attribute parsing (callback_groups / non_ros_threads / kernel_threads)
// ----------

TEST(ParseSchedAttrs, ParsesFifoPriorityAndAffinity)
{
  for (const auto & section : kThreadSections) {
    const auto attrs =
      attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n    affinity: [0, 1]\n");
    EXPECT_EQ(attrs.policy, acie::SchedPolicy::Fifo) << section.desc;
    EXPECT_EQ(attrs.rt_priority, 50) << section.desc;
    EXPECT_EQ(attrs.nice, 0) << section.desc;
    EXPECT_EQ(attrs.affinity, (std::vector<int>{0, 1})) << section.desc;
  }
}

TEST(ParseSchedAttrs, ParsesNiceForCfsPolicies)
{
  const std::pair<const char *, acie::SchedPolicy> policies[] = {
    {"SCHED_OTHER", acie::SchedPolicy::Other},
    {"SCHED_BATCH", acie::SchedPolicy::Batch},
    {"SCHED_IDLE", acie::SchedPolicy::Idle},
  };
  for (const auto & section : kThreadSections) {
    for (const auto & [name, policy] : policies) {
      const auto attrs = attrs_of(
        section, "    policy: " + std::string(name) + "\n    nice: -10\n    affinity: []\n");
      EXPECT_EQ(attrs.policy, policy) << section.desc << " " << name;
      EXPECT_EQ(attrs.nice, -10) << section.desc << " " << name;
      EXPECT_EQ(attrs.rt_priority, 0) << section.desc << " " << name;
    }
  }
}

TEST(ParseSchedAttrs, IgnoresStrayKeyOfTheOtherPolicyClass)
{
  for (const auto & section : kThreadSections) {
    const auto cfs = attrs_of(section, "    policy: SCHED_OTHER\n    nice: -5\n    priority: 50\n");
    EXPECT_EQ(cfs.nice, -5) << section.desc;
    EXPECT_EQ(cfs.rt_priority, 0) << section.desc;
    const auto rt = attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n    nice: 10\n");
    EXPECT_EQ(rt.rt_priority, 50) << section.desc;
    EXPECT_EQ(rt.nice, 0) << section.desc;
  }
}

TEST(ParseSchedAttrs, RejectsMissingOrNullNiceOnCfsPolicy)
{
  for (const auto & section : kThreadSections) {
    const std::string fragment =
      std::string("Policy 'SCHED_OTHER' requires 'nice' for ") + section.desc;
    expect_error(section.header + std::string("    policy: SCHED_OTHER\n"), fragment);
    expect_error(section.header + std::string("    policy: SCHED_OTHER\n    nice:\n"), fragment);
    // 'priority' does not satisfy the requirement.
    expect_error(
      section.header + std::string("    policy: SCHED_OTHER\n    priority: 5\n"), fragment);
  }
}

TEST(ParseSchedAttrs, RejectsMissingOrNullPriorityOnRtPolicy)
{
  for (const auto & section : kThreadSections) {
    const std::string fragment =
      std::string("Policy 'SCHED_FIFO' requires 'priority' for ") + section.desc;
    expect_error(section.header + std::string("    policy: SCHED_FIFO\n"), fragment);
    expect_error(
      section.header + std::string("    policy: SCHED_FIFO\n    priority: ~\n"), fragment);
    expect_error(
      section.header + std::string("    policy: SCHED_RR\n    nice: 5\n"),
      "Policy 'SCHED_RR' requires 'priority'");
  }
}

TEST(ParseSchedAttrs, RejectsNonDecimalNiceAndPriority)
{
  // yaml-cpp's as<int>() would read "0x10" as 16; only decimal digits (with
  // an optional sign) are valid.
  for (const auto & section : kThreadSections) {
    expect_error(
      section.header + std::string("    policy: SCHED_OTHER\n    nice: low\n"),
      std::string("'nice' must be a decimal integer for ") + section.desc + ", got 'low'");
    expect_error(
      section.header + std::string("    policy: SCHED_OTHER\n    nice: 0x5\n"),
      std::string("'nice' must be a decimal integer for ") + section.desc + ", got '0x5'");
    expect_error(
      section.header + std::string("    policy: SCHED_FIFO\n    priority: high\n"),
      std::string("'priority' must be a decimal integer for ") + section.desc + ", got 'high'");
  }
}

TEST(ParseSchedAttrs, RejectsNiceOutOfRange)
{
  for (const auto & section : kThreadSections) {
    for (const char * bad_nice : {"-21", "20", "50"}) {
      expect_error(
        section.header + std::string("    policy: SCHED_OTHER\n    nice: ") + bad_nice + "\n",
        "'nice' must be in [-20, 19]");
    }
  }
}

TEST(ParseSchedAttrs, RejectsRtPriorityOutOfRange)
{
  for (const auto & section : kThreadSections) {
    for (const char * bad_priority : {"0", "100", "-1"}) {
      expect_error(
        section.header + std::string("    policy: SCHED_FIFO\n    priority: ") + bad_priority +
          "\n",
        "'priority' must be in [1, 99]");
    }
  }
}

TEST(ParseSchedAttrs, RejectsUnknownPolicy)
{
  for (const auto & section : kThreadSections) {
    expect_error(
      section.header + std::string("    policy: SCHED_BOGUS\n    priority: 5\n"),
      std::string("Unknown scheduling policy 'SCHED_BOGUS' for ") + section.desc);
  }
}

TEST(ParseSchedAttrs, RejectsNonStringPolicy)
{
  for (const auto & section : kThreadSections) {
    expect_error(
      section.header + std::string("    policy: [a, b]\n"),
      std::string("'policy' must be a string for ") + section.desc);
  }
}

TEST(ParseSchedAttrs, ParsesSchedDeadline)
{
  for (const auto & section : kThreadSections) {
    const auto attrs = attrs_of(
      section,
      "    policy: SCHED_DEADLINE\n    runtime: 1000000\n    period: 5000000\n    deadline: "
      "5000000\n    affinity: [0]\n");
    EXPECT_EQ(attrs.policy, acie::SchedPolicy::Deadline) << section.desc;
    EXPECT_EQ(attrs.deadline.runtime, 1000000u) << section.desc;
    EXPECT_EQ(attrs.deadline.period, 5000000u) << section.desc;
    EXPECT_EQ(attrs.deadline.deadline, 5000000u) << section.desc;
    EXPECT_EQ(attrs.nice, 0) << section.desc;
    EXPECT_EQ(attrs.rt_priority, 0) << section.desc;
  }
}

TEST(ParseSchedAttrs, AcceptsDeadlineParamsBeyond32Bits)
{
  // sched_attr carries nanoseconds in 64-bit fields; a 5 s period exceeds
  // 2^32 ns.
  for (const auto & section : kThreadSections) {
    const auto attrs = attrs_of(
      section,
      "    policy: SCHED_DEADLINE\n    runtime: 1000000000\n    period: 5000000000\n    "
      "deadline: 5000000000\n");
    EXPECT_EQ(attrs.deadline.period, 5000000000u) << section.desc;
  }
}

TEST(ParseSchedAttrs, RejectsSchedDeadlineMissingFields)
{
  for (const auto & section : kThreadSections) {
    const std::string fragment =
      std::string("SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for ") + section.desc;
    expect_error(section.header + std::string("    policy: SCHED_DEADLINE\n"), fragment);
    expect_error(
      section.header + std::string("    policy: SCHED_DEADLINE\n    runtime: 1000000\n"), fragment);
  }
}

TEST(ParseSchedAttrs, RejectsMalformedSchedDeadlineFields)
{
  for (const auto & section : kThreadSections) {
    expect_error(
      section.header +
        std::string(
          "    policy: SCHED_DEADLINE\n    runtime: -5\n    period: 5000000\n    deadline: "
          "5000000\n"),
      std::string("'runtime' must be a non-negative decimal integer for ") + section.desc);
    expect_error(
      section.header +
        std::string(
          "    policy: SCHED_DEADLINE\n    runtime: 1000000\n    period: not_a_number\n    "
          "deadline: 5000000\n"),
      std::string("'period' must be a non-negative decimal integer for ") + section.desc);
    // yaml-cpp would read "0x10" as 16; only plain decimal digits are valid.
    expect_error(
      section.header +
        std::string(
          "    policy: SCHED_DEADLINE\n    runtime: 1000000\n    period: 5000000\n    deadline: "
          "0x10\n"),
      std::string("'deadline' must be a non-negative decimal integer for ") + section.desc);
  }
}

TEST(ParseSchedAttrs, ParsesZeroPaddedDeadlineFieldAsBase10)
{
  // yaml-cpp's base auto-detection would read "0500000" as octal; a
  // zero-padded column must mean decimal.
  for (const auto & section : kThreadSections) {
    const auto attrs = attrs_of(
      section,
      "    policy: SCHED_DEADLINE\n    runtime: 0500000\n    period: 5000000\n    deadline: "
      "5000000\n");
    EXPECT_EQ(attrs.deadline.runtime, 500000u) << section.desc;
  }
}

// ---------- affinity (shared by all four sections) ----------

TEST(ParseSchedAttrs, NormalizesAffinityToSortedUnique)
{
  // parse_affinity bounds values by the machine CPU count, so only CPUs 0
  // and 1 are used to keep this test valid on small CI machines.
  for (const auto & section : kThreadSections) {
    const auto attrs =
      attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n    affinity: [1, 0, 1]\n");
    EXPECT_EQ(attrs.affinity, (std::vector<int>{0, 1})) << section.desc;
  }
  const auto config = parse("irqs:\n  - irq: 5\n    affinity: [1, 0, 1]\n");
  EXPECT_EQ(config.irqs.at(0).affinity, (std::vector<int>{0, 1}));
}

TEST(ParseSchedAttrs, ParsesZeroPaddedAffinityCpuAsBase10)
{
  for (const auto & section : kThreadSections) {
    const auto attrs =
      attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n    affinity: [01]\n");
    EXPECT_EQ(attrs.affinity, (std::vector<int>{1})) << section.desc;
  }
}

TEST(ParseSchedAttrs, TreatsAbsentOrNullAffinityAsUnmanaged)
{
  for (const auto & section : kThreadSections) {
    EXPECT_TRUE(attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n").affinity.empty())
      << section.desc;
    EXPECT_TRUE(attrs_of(section, "    policy: SCHED_FIFO\n    priority: 50\n    affinity: ~\n")
                  .affinity.empty())
      << section.desc;
  }
}

TEST(ParseSchedAttrs, RejectsAffinityCpuOutOfRange)
{
  // CPU_SET(3) / sched_setaffinity(2) would silently ignore all of these,
  // shrinking the mask without any error report. The valid CPU 0 in front
  // checks that it does not mask the error.
  const long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
  for (const auto & section : kThreadSections) {
    for (const std::string & bad_affinity :
         {std::string("[-1]"), "[0, " + std::to_string(num_cpus) + "]",
          "[0, " + std::to_string(CPU_SETSIZE) + "]"}) {
      expect_error(
        section.header + std::string("    policy: SCHED_FIFO\n    priority: 50\n    affinity: ") +
          bad_affinity + "\n",
        "must be in [0, ");
    }
  }
  expect_error("irqs:\n  - irq: 5\n    affinity: [-1]\n", "'affinity' CPU -1 must be in");
}

TEST(ParseSchedAttrs, AcceptsHighestValidAffinityCpu)
{
  // Pins the accept side of the [0, min(CPU_SETSIZE, num_cpus)) boundary.
  const int max_cpu =
    static_cast<int>(std::min<long>(CPU_SETSIZE, sysconf(_SC_NPROCESSORS_CONF))) - 1;
  for (const auto & section : kThreadSections) {
    const auto attrs = attrs_of(
      section, "    policy: SCHED_FIFO\n    priority: 50\n    affinity: [" +
                 std::to_string(max_cpu) + "]\n");
    EXPECT_EQ(attrs.affinity, (std::vector<int>{max_cpu})) << section.desc;
  }
}

TEST(ParseSchedAttrs, RejectsScalarAffinity)
{
  // A scalar (e.g. a cpu-list string copied from a scan) iterates zero times
  // and would otherwise silently mean "no affinity".
  for (const auto & section : kThreadSections) {
    for (const char * bad_affinity : {"2", "\"0-3\""}) {
      expect_error(
        section.header + std::string("    policy: SCHED_FIFO\n    priority: 50\n    affinity: ") +
          bad_affinity + "\n",
        std::string("'affinity' must be a list of CPU numbers (e.g. [2, 3]) for ") + section.desc);
    }
  }
  expect_error("irqs:\n  - irq: 5\n    affinity: 0-3\n", "'affinity' must be a list");
}

TEST(ParseSchedAttrs, RejectsNonDecimalAffinityElement)
{
  for (const auto & section : kThreadSections) {
    expect_error(
      section.header +
        std::string("    policy: SCHED_FIFO\n    priority: 50\n    affinity: [0, all]\n"),
      std::string("'affinity' must contain only decimal integers for ") + section.desc +
        ", got 'all'");
    expect_error(
      section.header +
        std::string("    policy: SCHED_FIFO\n    priority: 50\n    affinity: [0x1]\n"),
      "got '0x1'");
  }
}

// ---------- callback_groups ----------

TEST(ParseCallbackGroups, ParsesEntry)
{
  const auto config = parse(R"YAML(
callback_groups:
  - id: my_cbg
    domain_id: 3
    policy: SCHED_FIFO
    priority: 50
    affinity: [0, 1]
)YAML");
  ASSERT_EQ(config.callback_groups.size(), 1u);
  const auto & entry = config.callback_groups[0];
  EXPECT_EQ(entry.id, "my_cbg");
  EXPECT_EQ(entry.domain_id, 3u);
  EXPECT_EQ(entry.attrs.policy, acie::SchedPolicy::Fifo);
  EXPECT_EQ(entry.attrs.rt_priority, 50);
  EXPECT_EQ(entry.attrs.affinity, (std::vector<int>{0, 1}));
  EXPECT_FALSE(entry.is_wildcard());
}

TEST(ParseCallbackGroups, RequiresPolicy)
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

TEST(ParseCallbackGroups, FallsBackToDefaultDomainId)
{
  const auto config =
    parse("callback_groups:\n  - id: my_cbg\n    policy: SCHED_OTHER\n    nice: 0\n");
  ASSERT_EQ(config.callback_groups.size(), 1u);
  EXPECT_EQ(config.callback_groups[0].domain_id, kTestDefaultDomain);
}

TEST(ParseCallbackGroups, RejectsDuplicateKey)
{
  expect_error(
    R"YAML(
callback_groups:
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
)YAML",
    "Duplicate callback_group entry: domain_id=0, id=cg");
}

TEST(ParseCallbackGroups, AllowsSameIdInDifferentDomains)
{
  const auto config = parse(R"YAML(
callback_groups:
  - id: cg
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
  - id: cg
    domain_id: 1
    policy: SCHED_OTHER
    nice: 0
)YAML");
  EXPECT_EQ(config.callback_groups.size(), 2u);
}

TEST(ParseCallbackGroups, UnmanageableSentinelIsNotRecognized)
{
  // The sentinel is scoped to kernel_threads/irqs (the tool writes it there);
  // in the hand-written sections it must fail like any other invalid value,
  // not silently mean "leave the attribute alone".
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    policy: SCHED_FIFO\n    priority: 50\n    affinity: "
    "UNMANAGEABLE\n",
    "'affinity' must be a list");
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    policy: SCHED_OTHER\n    nice: UNMANAGEABLE\n",
    "'nice' must be a decimal integer for id=my_cbg, got 'UNMANAGEABLE'");
  expect_error(
    "callback_groups:\n  - id: my_cbg\n    policy: UNMANAGEABLE\n",
    "Unknown scheduling policy 'UNMANAGEABLE'");
  expect_error(
    "non_ros_threads:\n  - name: my_thread\n    policy: SCHED_OTHER\n    nice: UNMANAGEABLE\n",
    "'nice' must be a decimal integer for name=my_thread, got 'UNMANAGEABLE'");
}

// ---------- wildcard ("<node name>/*") callback-group ids ----------

TEST(ParseCallbackGroups, ParsesWildcardId)
{
  const auto config = parse(R"YAML(
callback_groups:
  - id: /perception/lidar_node/*
    domain_id: 0
    policy: SCHED_FIFO
    priority: 50
    affinity: [0]
)YAML");
  ASSERT_EQ(config.callback_groups.size(), 1u);
  const auto & entry = config.callback_groups[0];
  EXPECT_TRUE(entry.is_wildcard());
  EXPECT_EQ(entry.wildcard_prefix(), "/perception/lidar_node");
  EXPECT_EQ(entry.id, "/perception/lidar_node/*");  // kept as written
}

TEST(ParseCallbackGroups, RejectsWildcardWithEmptyNodePart)
{
  expect_error(
    "callback_groups:\n  - id: /*\n    policy: SCHED_OTHER\n    nice: 0\n",
    "the part before \"/*\" must be a non-empty node name");
}

TEST(ParseCallbackGroups, RejectsStrayAsteriskInId)
{
  for (const char * bad_id :
       {"/node*", "/node/**", "/node/*x", "/a/*/b", "*", "/node/*@Waitable"}) {
    expect_error(
      std::string("callback_groups:\n  - id: \"") + bad_id +
        "\"\n    policy: SCHED_OTHER\n    nice: 0\n",
      "'*' is only allowed as a trailing \"/*\" wildcard");
  }
}

TEST(ParseCallbackGroups, RejectsAtSignInWildcardPrefix)
{
  // A full callback-group id copied from the template with "/*" appended.
  expect_error(
    "callback_groups:\n  - id: /node@Timer(100)/*\n    policy: SCHED_OTHER\n    nice: 0\n",
    "must be a plain node name, not a full callback-group id");
}

TEST(ParseCallbackGroups, RejectsDuplicateWildcardKey)
{
  expect_error(
    R"YAML(
callback_groups:
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
)YAML",
    "Duplicate callback_group entry: domain_id=0, id=/node/*");
}

TEST(ParseCallbackGroups, AllowsSameWildcardInDifferentDomainsAndExactForSameNode)
{
  const auto config = parse(R"YAML(
callback_groups:
  - id: /node/*
    domain_id: 0
    policy: SCHED_OTHER
    nice: 0
  - id: /node/*
    domain_id: 1
    policy: SCHED_OTHER
    nice: 0
  - id: /node@Timer(100)
    domain_id: 0
    policy: SCHED_FIFO
    priority: 80
)YAML");
  ASSERT_EQ(config.callback_groups.size(), 3u);
  EXPECT_TRUE(config.callback_groups[0].is_wildcard());
  EXPECT_TRUE(config.callback_groups[1].is_wildcard());
  EXPECT_FALSE(config.callback_groups[2].is_wildcard());
}

// ---------- non_ros_threads ----------

TEST(ParseNonRosThreads, NamesAreOpaque)
{
  // Wildcards are a callback_groups-only feature; a name is matched exactly
  // and no '*' rule applies to it.
  const auto config = parse(R"YAML(
non_ros_threads:
  - name: worker/*
    policy: SCHED_OTHER
    nice: 0
  - name: w*rk
    policy: SCHED_OTHER
    nice: 0
)YAML");
  ASSERT_EQ(config.non_ros_threads.size(), 2u);
  EXPECT_EQ(config.non_ros_threads[0].name, "worker/*");
  EXPECT_EQ(config.non_ros_threads[1].name, "w*rk");
}

TEST(ParseNonRosThreads, RejectsDuplicateName)
{
  expect_error(
    R"YAML(
non_ros_threads:
  - name: t
    policy: SCHED_OTHER
    nice: 0
  - name: t
    policy: SCHED_OTHER
    nice: 0
)YAML",
    "Duplicate non_ros_thread entry: name=t");
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

TEST(ParseKernelThreads, ParsesPolicyPriorityAffinity)
{
  const auto config = parse(R"YAML(
kernel_threads:
  - comm: agnocast_exit_w
    policy: SCHED_FIFO
    priority: 10
    affinity: [0, 1]
)YAML");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  const auto & entry = config.kernel_threads[0];
  EXPECT_EQ(entry.comm, "agnocast_exit_w");
  EXPECT_EQ(entry.attrs.policy, acie::SchedPolicy::Fifo);
  EXPECT_EQ(entry.attrs.rt_priority, 10);
  EXPECT_EQ(entry.attrs.affinity, (std::vector<int>{0, 1}));
  EXPECT_TRUE(entry.is_managed());
}

TEST(ParseKernelThreads, AllNullEntryIsUnmanaged)
{
  const auto config = parse(R"YAML(
kernel_threads:
  - comm: rcu_preempt
    policy: ~
    nice: ~
    priority: ~
    affinity: ~
)YAML");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_FALSE(config.kernel_threads[0].attrs.policy.has_value());
  EXPECT_TRUE(config.kernel_threads[0].attrs.affinity.empty());
  EXPECT_FALSE(config.kernel_threads[0].is_managed());
}

TEST(ParseKernelThreads, UnmanageableSentinelEqualsNull)
{
  const auto config = parse(R"YAML(
kernel_threads:
  - comm: ksoftirqd/0
    policy: UNMANAGEABLE
    nice: UNMANAGEABLE
    priority: UNMANAGEABLE
    affinity: UNMANAGEABLE
)YAML");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_FALSE(config.kernel_threads[0].attrs.policy.has_value());
  EXPECT_TRUE(config.kernel_threads[0].attrs.affinity.empty());
  EXPECT_FALSE(config.kernel_threads[0].is_managed());
}

TEST(ParseKernelThreads, AffinityOnlyEntryIsManaged)
{
  const auto config = parse(R"YAML(
kernel_threads:
  - comm: agnocast_exit_w
    policy: ~
    priority: ~
    affinity: [1]
)YAML");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_FALSE(config.kernel_threads[0].attrs.policy.has_value());
  EXPECT_EQ(config.kernel_threads[0].attrs.affinity, (std::vector<int>{1}));
  EXPECT_TRUE(config.kernel_threads[0].is_managed());
}

TEST(ParseKernelThreads, PolicyWithUnmanageableAffinityIsManaged)
{
  const auto config = parse(R"YAML(
kernel_threads:
  - comm: ksoftirqd/0
    policy: SCHED_FIFO
    priority: 5
    affinity: UNMANAGEABLE
)YAML");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_TRUE(config.kernel_threads[0].attrs.policy.has_value());
  EXPECT_TRUE(config.kernel_threads[0].attrs.affinity.empty());
  EXPECT_TRUE(config.kernel_threads[0].is_managed());
}

TEST(ParseKernelThreads, LowercaseSentinelIsNotRecognized)
{
  // Only the exact uppercase sentinel disengages an attribute; anything else
  // must fall through to the normal validation (here: unknown policy). No
  // other attribute key is set, so a case-insensitive sentinel match would
  // parse cleanly and fail the test.
  expect_error(
    "kernel_threads:\n  - comm: rcu_preempt\n    policy: unmanageable\n    affinity: ~\n",
    "Unknown scheduling policy 'unmanageable'");
}

TEST(ParseKernelThreads, SentinelTunableIsReportedAsUnset)
{
  // The sentinel counts as unset exactly like null does, and the message
  // says so because the key is visibly present.
  expect_error(
    "kernel_threads:\n  - comm: rcu_preempt\n    policy: SCHED_FIFO\n    priority: UNMANAGEABLE\n",
    "requires 'priority' for comm=rcu_preempt (UNMANAGEABLE counts as unset)");
}

TEST(ParseKernelThreads, RejectsMissingEmptyOrNonStringComm)
{
  expect_error(
    "kernel_threads:\n  - policy: ~\n    affinity: ~\n",
    "kernel_threads entry #0 is missing a non-empty 'comm'");
  expect_error(
    "kernel_threads:\n  - comm: \"\"\n    policy: ~\n    affinity: ~\n",
    "kernel_threads entry #0 is missing a non-empty 'comm'");
  expect_error(
    "kernel_threads:\n  - comm: [a, b]\n    policy: ~\n    affinity: ~\n",
    "kernel_threads entry #0: 'comm' must be a string");
}

TEST(ParseKernelThreads, RejectsKworkerComm)
{
  expect_error(
    "kernel_threads:\n  - comm: kworker/0:0H-events_highpri\n    policy: ~\n    affinity: ~\n",
    "kworker comms are ephemeral");
}

TEST(ParseKernelThreads, AcceptsCommLongerThan15Chars)
{
  // Since Linux 5.17 /proc reports a kthread's full name untruncated (e.g.
  // the kmod's "agnocast_exit_worker"), so the scanner returns comms longer
  // than TASK_COMM_LEN - 1 and they must round-trip through the parser.
  const auto config =
    parse("kernel_threads:\n  - comm: agnocast_exit_worker\n    policy: ~\n    affinity: [0]\n");
  ASSERT_EQ(config.kernel_threads.size(), 1u);
  EXPECT_EQ(config.kernel_threads[0].comm, "agnocast_exit_worker");
  EXPECT_TRUE(config.kernel_threads[0].is_managed());
}

TEST(ParseKernelThreads, RejectsDuplicateComm)
{
  expect_error(
    R"YAML(
kernel_threads:
  - comm: nfsd
    policy: ~
    affinity: ~
  - comm: nfsd
    policy: ~
    affinity: ~
)YAML",
    "Duplicate kernel_thread entry: comm=nfsd");
}

TEST(ParseKernelThreads, RejectsPolicyDependentFieldsWithoutPolicy)
{
  // Such an entry would otherwise parse cleanly as unmanaged, i.e. silently
  // dead configuration.
  expect_error(
    "kernel_threads:\n  - comm: rcu_preempt\n    policy: ~\n    priority: 5\n",
    "'priority' requires 'policy' for comm=rcu_preempt: set both or leave both unset");
  expect_error(
    "kernel_threads:\n  - comm: rcu_preempt\n    policy: ~\n    nice: 5\n",
    "'nice' requires 'policy'");
  expect_error(
    "kernel_threads:\n  - comm: dl_thread\n    policy: ~\n    runtime: 1000000\n    period: "
    "5000000\n    deadline: 5000000\n",
    "'runtime' requires 'policy'");
}

// ---------- irqs ----------

TEST(ParseIrqs, ParsesFilledAffinityAndName)
{
  const auto config = parse(R"YAML(
irqs:
  - irq: 103
    name: nvidia
    affinity: [1, 0]
)YAML");
  ASSERT_EQ(config.irqs.size(), 1u);
  EXPECT_EQ(config.irqs[0].irq, 103);
  EXPECT_EQ(config.irqs[0].name, "nvidia");
  EXPECT_EQ(config.irqs[0].affinity, (std::vector<int>{0, 1}));
  EXPECT_TRUE(config.irqs[0].is_managed());
}

TEST(ParseIrqs, NullOrUnmanageableAffinityIsUnmanaged)
{
  const auto config = parse(R"YAML(
irqs:
  - irq: 0
    name: timer
    affinity: UNMANAGEABLE
  - irq: 1
    name: i8042
    affinity: ~
)YAML");
  ASSERT_EQ(config.irqs.size(), 2u);
  EXPECT_FALSE(config.irqs[0].is_managed());
  EXPECT_FALSE(config.irqs[1].is_managed());
}

TEST(ParseIrqs, MissingOrNullNameMeansNoVerification)
{
  for (const char * name_line : {"", "    name: ~\n"}) {
    const auto config =
      parse(std::string("irqs:\n  - irq: 42\n") + name_line + "    affinity: [1]\n");
    ASSERT_EQ(config.irqs.size(), 1u);
    EXPECT_TRUE(config.irqs[0].name.empty());
    EXPECT_TRUE(config.irqs[0].is_managed());
  }
}

TEST(ParseIrqs, RejectsNonStringName)
{
  expect_error(
    "irqs:\n  - irq: 5\n    name: [a]\n    affinity: ~\n", "'name' must be a string for irq=5");
}

TEST(ParseIrqs, RejectsMissingOrNegativeOrNonIntIrq)
{
  expect_error(
    "irqs:\n  - name: orphan\n    affinity: ~\n",
    "irqs entry #0 is missing a non-negative integer 'irq'");
  expect_error(
    "irqs:\n  - irq: -1\n    affinity: ~\n",
    "irqs entry #0: 'irq' must be a non-negative decimal integer, got '-1'");
  expect_error(
    "irqs:\n  - irq: not_a_number\n    affinity: ~\n",
    "'irq' must be a non-negative decimal integer, got 'not_a_number'");
}

TEST(ParseIrqs, ParsesZeroPaddedIrqAsBase10)
{
  // yaml-cpp's as<int>() would read "010" as octal 8 and silently target a
  // different interrupt; a zero-padded column must mean decimal 10.
  const auto config = parse("irqs:\n  - irq: 010\n    affinity: ~\n");
  ASSERT_EQ(config.irqs.size(), 1u);
  EXPECT_EQ(config.irqs[0].irq, 10);
}

TEST(ParseIrqs, RejectsHexIrq)
{
  // yaml-cpp would read "0x10" as 16; only plain decimal digits are valid.
  expect_error(
    "irqs:\n  - irq: 0x10\n    affinity: ~\n",
    "'irq' must be a non-negative decimal integer, got '0x10'");
}

TEST(ParseIrqs, RejectsDuplicateIrq)
{
  expect_error(
    "irqs:\n  - irq: 5\n    affinity: ~\n  - irq: 5\n    affinity: ~\n",
    "Duplicate irq entry: irq=5");
}
