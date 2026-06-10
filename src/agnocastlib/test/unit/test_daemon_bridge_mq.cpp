#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <gtest/gtest.h>

#include <chrono>
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

// Performance-mode daemon bridges have no local endpoint to query, so the QoS
// must be rebuilt faithfully from the request's explicit fields.
TEST(DaemonBridgeMqTest, DaemonRequestQosReliableTransientLocal)
{
  agnocast::MqMsgDaemonBridge req{};
  req.qos_depth = 10;
  req.qos_is_reliable = true;
  req.qos_is_transient_local = true;

  const rclcpp::QoS qos = agnocast::daemon_request_qos(req);
  EXPECT_EQ(qos.depth(), 10u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(DaemonBridgeMqTest, DaemonRequestQosBestEffortVolatile)
{
  agnocast::MqMsgDaemonBridge req{};
  req.qos_depth = 1;
  req.qos_is_reliable = false;
  req.qos_is_transient_local = false;

  const rclcpp::QoS qos = agnocast::daemon_request_qos(req);
  EXPECT_EQ(qos.depth(), 1u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

// The daemon-forced lease (used by the performance bridge_manager to keep a
// cross-NS bridge alive without a same-graph DDS counterpart) is active for the
// half-open window [registered, registered + DAEMON_FORCE_TTL).
TEST(DaemonBridgeMqTest, DaemonForceLeaseWindowIsHalfOpen)
{
  const std::chrono::steady_clock::time_point t0{};
  const auto deadline = agnocast::daemon_force_deadline(t0);

  // The lease lasts exactly DAEMON_FORCE_TTL.
  EXPECT_EQ(deadline - t0, agnocast::DAEMON_FORCE_TTL);

  EXPECT_TRUE(agnocast::is_daemon_force_active(deadline, t0));  // just registered
  EXPECT_TRUE(agnocast::is_daemon_force_active(
    deadline, t0 + agnocast::DAEMON_FORCE_TTL - std::chrono::milliseconds(1)));  // within
  // Boundary: the window is half-open, so the exact deadline is already expired.
  EXPECT_FALSE(agnocast::is_daemon_force_active(deadline, t0 + agnocast::DAEMON_FORCE_TTL));
  EXPECT_FALSE(agnocast::is_daemon_force_active(
    deadline, t0 + agnocast::DAEMON_FORCE_TTL + std::chrono::seconds(1)));  // after
}

// Re-asserting the request (the daemon does so every tick) pushes the deadline
// out, so a continuously-requested bridge never lapses.
TEST(DaemonBridgeMqTest, DaemonForceLeaseRenewalExtendsDeadline)
{
  const std::chrono::steady_clock::time_point t0{};
  const auto first = agnocast::daemon_force_deadline(t0);
  const auto renewed = agnocast::daemon_force_deadline(t0 + std::chrono::seconds(1));

  EXPECT_GT(renewed, first);
  // Still active at the original deadline because it was renewed before lapsing.
  EXPECT_TRUE(agnocast::is_daemon_force_active(renewed, first));
}
