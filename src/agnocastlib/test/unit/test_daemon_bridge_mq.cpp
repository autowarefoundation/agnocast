#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdlib>
#include <string>

namespace
{
// Restores ROS_DOMAIN_ID to its pre-test value on scope exit, so tests that
// mutate it don't leak into later tests (the runner may start with it set).
class ScopedRosDomainId
{
public:
  ScopedRosDomainId()
  {
    const char * value = getenv("ROS_DOMAIN_ID");
    if (value != nullptr) {
      had_value_ = true;
      old_value_ = value;
    }
  }
  ~ScopedRosDomainId()
  {
    if (had_value_) {
      setenv("ROS_DOMAIN_ID", old_value_.c_str(), 1);
    } else {
      unsetenv("ROS_DOMAIN_ID");
    }
  }

private:
  bool had_value_ = false;
  std::string old_value_;
};
}  // namespace

// The discovery daemon (Python) packs MqMsgDaemonBridge by hand, mirroring
// this layout. These checks fail loudly if the C++ struct drifts from the
// daemon's `_MSG_PACK_FORMAT` ('=256s256sIIBB2x', 524 bytes).
TEST(DaemonBridgeMqTest, WireLayoutMatchesDaemonPackFormat)
{
  using agnocast::MqMsgDaemonBridge;
  EXPECT_EQ(sizeof(MqMsgDaemonBridge), 524u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, topic_name), 0u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, type_name), 256u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, direction), 512u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_depth), 516u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_is_transient_local), 520u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_is_reliable), 521u);
}

TEST(DaemonBridgeMqTest, StandardMqNameIsKeyedByPid)
{
  EXPECT_EQ(agnocast::create_mq_name_for_daemon_bridge(4242), "/agnocast_daemon_bridge@4242");
}

TEST(DaemonBridgeMqTest, PerformanceMqNameIsPerNamespace)
{
  const ScopedRosDomainId guard;
  unsetenv("ROS_DOMAIN_ID");
  EXPECT_EQ(
    agnocast::create_mq_name_for_daemon_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_daemon_bridge_perf");
}

TEST(DaemonBridgeMqTest, PerformanceMqNameAppendsDomainId)
{
  const ScopedRosDomainId guard;
  setenv("ROS_DOMAIN_ID", "7", 1);
  EXPECT_EQ(
    agnocast::create_mq_name_for_daemon_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_daemon_bridge_perf_d7");
}

// An empty ROS_DOMAIN_ID (set but "") means "no domain": no `_d` suffix. Both
// name builders must agree on this and with the Python agent.
TEST(DaemonBridgeMqTest, PerformanceMqNameEmptyDomainIdHasNoSuffix)
{
  const ScopedRosDomainId guard;
  setenv("ROS_DOMAIN_ID", "", 1);
  EXPECT_EQ(
    agnocast::create_mq_name_for_daemon_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_daemon_bridge_perf");
  EXPECT_EQ(
    agnocast::create_mq_name_for_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_bridge_manager@" + std::to_string(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID));
}
