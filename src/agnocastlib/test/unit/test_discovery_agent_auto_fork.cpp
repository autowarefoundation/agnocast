#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

namespace agnocast
{
// Defined in agnocast.cpp; gates the discovery-agent auto-fork in initialize_agnocast.
bool discovery_agent_auto_fork_disabled();
}  // namespace agnocast

namespace
{
void set_opt_out(const char * value)
{
  if (value != nullptr) {
    setenv("AGNOCAST_NO_DISCOVERY_AGENT", value, 1);
  } else {
    unsetenv("AGNOCAST_NO_DISCOVERY_AGENT");
  }
}
}  // namespace

// Restore AGNOCAST_NO_DISCOVERY_AGENT after each test: it is process-global, so leaving it
// set would make later tests in the same binary order-dependent.
class DiscoveryAgentAutoFork : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * original = std::getenv("AGNOCAST_NO_DISCOVERY_AGENT");
    had_original_ = original != nullptr;
    if (had_original_) {
      original_ = original;
    }
  }

  void TearDown() override
  {
    if (had_original_) {
      setenv("AGNOCAST_NO_DISCOVERY_AGENT", original_.c_str(), 1);
    } else {
      unsetenv("AGNOCAST_NO_DISCOVERY_AGENT");
    }
  }

private:
  bool had_original_ = false;
  std::string original_;
};

TEST_F(DiscoveryAgentAutoFork, EnabledByDefault)
{
  set_opt_out(nullptr);
  EXPECT_FALSE(agnocast::discovery_agent_auto_fork_disabled());
}

TEST_F(DiscoveryAgentAutoFork, DisabledByTruthyValues)
{
  for (const char * v : {"1", "true", "TRUE", "yes", "Yes"}) {
    set_opt_out(v);
    EXPECT_TRUE(agnocast::discovery_agent_auto_fork_disabled()) << "value=" << v;
  }
}

TEST_F(DiscoveryAgentAutoFork, EnabledForFalsyOrUnrelatedValues)
{
  // "false"/"no" must NOT disable the agent — only an explicit truthy value does.
  for (const char * v : {"", "0", "false", "no"}) {
    set_opt_out(v);
    EXPECT_FALSE(agnocast::discovery_agent_auto_fork_disabled()) << "value=" << v;
  }
}
