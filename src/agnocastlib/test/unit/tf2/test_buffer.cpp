#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/tf2/buffer.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer_interface.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

namespace
{
geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent_frame, const std::string & child_frame, double tx,
  const tf2::TimePoint & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.header.stamp = tf2_ros::toMsg(stamp);
  t.child_frame_id = child_frame;
  t.transform.translation.x = tx;
  t.transform.rotation.w = 1.0;
  return t;
}

constexpr tf2::TimePoint kStamp{std::chrono::seconds(10)};
}  // namespace

class BufferTest : public ::testing::Test
{
protected:
  // agnocast::init() is required so agnocast::ok() returns true; otherwise
  // the canTransform poll loop short-circuits on the first iteration.
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  }

  void TearDown() override { agnocast::shutdown(); }

  rclcpp::Clock::SharedPtr clock_;
};

// =========================================
// Constructor
// =========================================

TEST_F(BufferTest, throws_when_clock_is_null)
{
  // Arrange / Act / Assert: a null clock must be rejected eagerly.
  EXPECT_THROW({ agnocast::Buffer buffer(nullptr); }, std::invalid_argument);
}

TEST_F(BufferTest, getClock_returns_provided_clock)
{
  // Arrange
  agnocast::Buffer buffer(clock_);

  // Act
  auto returned = buffer.getClock();

  // Assert: the buffer must hold the very same clock we passed in.
  EXPECT_EQ(returned, clock_);
}

TEST_F(BufferTest, accepts_null_node_argument_without_logger_binding)
{
  // Arrange: explicit null node — covers the `if (node)` false branch.
  std::shared_ptr<agnocast::Node> null_node;

  // Act / Assert: construction must succeed and getLogger() falls back to default.
  EXPECT_NO_THROW({
    agnocast::Buffer buffer(clock_, tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), null_node);
  });
}

TEST_F(BufferTest, binds_node_logger_when_node_passed)
{
  // Arrange: spin up an Agnocast node so the logger interface can be extracted.
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  auto node = std::make_shared<agnocast::Node>("buffer_logger_test_node", options);

  // Act: pass the node — buffer should bind the node's logger via NodeLoggingInterface.
  agnocast::Buffer buffer(clock_, tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node);

  // Assert: indirectly verify by triggering the threading-error path, which
  // routes through getLogger(). We only care that no crash occurs and the
  // err-string mechanism still functions when a node logger is bound.
  std::string err;
  EXPECT_FALSE(buffer.canTransform("a", "b", kStamp, tf2::durationFromSec(0.01), &err));
  EXPECT_FALSE(err.empty());
}

// =========================================
// setTransform / lookupTransform — synchronous data path
// =========================================

TEST_F(BufferTest, setTransform_then_lookup_returns_same_translation)
{
  // Arrange
  agnocast::Buffer buffer(clock_);
  buffer.setTransform(make_transform("map", "base_link", 1.5, kStamp), "test_authority");

  // Act
  auto out = buffer.lookupTransform("map", "base_link", kStamp, tf2::Duration(0));

  // Assert: the round-tripped transform retains frame ids and translation.
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_EQ(out.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 1.5);
}

TEST_F(BufferTest, lookup_unknown_frame_throws_lookup_exception)
{
  // Arrange: empty buffer — no frames known.
  agnocast::Buffer buffer(clock_);

  // Act / Assert
  EXPECT_THROW(
    { buffer.lookupTransform("map", "missing", kStamp, tf2::Duration(0)); }, tf2::LookupException);
}

TEST_F(BufferTest, fixed_frame_lookup_composes_two_transforms)
{
  // Arrange: chain map -> odom -> base_link via fixed frame "odom".
  agnocast::Buffer buffer(clock_);
  buffer.setTransform(make_transform("map", "odom", 1.0, kStamp), "test_authority");
  buffer.setTransform(make_transform("odom", "base_link", 2.0, kStamp), "test_authority");

  // Act
  auto out = buffer.lookupTransform("map", kStamp, "base_link", kStamp, "odom", tf2::Duration(0));

  // Assert: x translations compose linearly along the chain (1.0 + 2.0).
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 3.0);
}

// =========================================
// canTransform — direct existence check (no timeout)
// =========================================

TEST_F(BufferTest, canTransform_unknown_frame_returns_false_and_populates_errstr)
{
  // Arrange
  agnocast::Buffer buffer(clock_);
  std::string err;

  // Act
  bool result = buffer.canTransform("map", "missing", kStamp, tf2::Duration(0), &err);

  // Assert: false returned and err carries explanatory text.
  EXPECT_FALSE(result);
  EXPECT_FALSE(err.empty());
}

TEST_F(BufferTest, canTransform_unknown_frame_with_null_errstr_does_not_crash)
{
  // Arrange: covers the `errstr == nullptr` branch of conditionally_append_timeout_info.
  agnocast::Buffer buffer(clock_);

  // Act / Assert: passing nullptr explicitly must not crash.
  EXPECT_FALSE(buffer.canTransform("map", "missing", kStamp, tf2::Duration(0), nullptr));
}

TEST_F(BufferTest, canTransform_fixed_frame_returns_true_for_existing_chain)
{
  // Arrange: setUsingDedicatedThread(true) bypasses the threading guard that fires
  // unconditionally on Humble (rclcpp<28) even for timeout=0. This test is about the
  // fixed-frame chain lookup, not the guard — that behavior is covered separately.
  agnocast::Buffer buffer(clock_);
  buffer.setUsingDedicatedThread(true);
  buffer.setTransform(make_transform("map", "odom", 1.0, kStamp), "test_authority");
  buffer.setTransform(make_transform("odom", "base_link", 2.0, kStamp), "test_authority");

  // Act
  bool result = buffer.canTransform("map", kStamp, "base_link", kStamp, "odom", tf2::Duration(0));

  // Assert
  EXPECT_TRUE(result);
}

// =========================================
// canTransform — threading guard (timeout > 0)
// =========================================

#if RCLCPP_VERSION_MAJOR >= 28
TEST_F(BufferTest, canTransform_zero_timeout_no_dedicated_thread_is_allowed_on_jazzy)
{
  // On Jazzy the threading guard fires only when timeout > 0. With zero timeout
  // and no dedicated thread, canTransform must still execute and answer false
  // (because no transform is set).
  // Arrange
  agnocast::Buffer buffer(clock_);
  std::string err;

  // Act
  bool result = buffer.canTransform("a", "b", kStamp, tf2::Duration(0), &err);

  // Assert
  EXPECT_FALSE(result);
  EXPECT_EQ(err.find("dedicated thread"), std::string::npos)
    << "Threading guard must NOT fire for zero-timeout calls on Jazzy.";
}
#endif

TEST_F(BufferTest, canTransform_3arg_with_timeout_no_thread_returns_false_with_threading_error)
{
  // Arrange: no setUsingDedicatedThread() call.
  agnocast::Buffer buffer(clock_);
  std::string err;

  // Act
  bool result = buffer.canTransform("a", "b", kStamp, tf2::durationFromSec(0.05), &err);

  // Assert: the threading_error sentinel must appear in the err string.
  EXPECT_FALSE(result);
  EXPECT_NE(err.find("dedicated thread"), std::string::npos);
}

TEST_F(BufferTest, canTransform_5arg_with_timeout_no_thread_returns_false_with_threading_error)
{
  // Arrange: same as above but exercises the 5-argument fixed-frame overload
  // — its own threading-guard branch must also fire.
  agnocast::Buffer buffer(clock_);
  std::string err;

  // Act
  bool result =
    buffer.canTransform("a", kStamp, "b", kStamp, "fixed", tf2::durationFromSec(0.05), &err);

  // Assert
  EXPECT_FALSE(result);
  EXPECT_NE(err.find("dedicated thread"), std::string::npos);
}

TEST_F(BufferTest, canTransform_3arg_with_timeout_polls_then_appends_timeout_info)
{
  // Arrange: dedicated thread declared so the poll loop can run.
  agnocast::Buffer buffer(clock_);
  buffer.setUsingDedicatedThread(true);
  std::string err;

  // Act: poll for ~50ms looking for a transform that will never arrive.
  const auto t0 = std::chrono::steady_clock::now();
  bool result = buffer.canTransform("a", "b", kStamp, tf2::durationFromSec(0.05), &err);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  // Assert: false returned, the loop spent meaningful time, and the timeout
  // suffix produced by conditionally_append_timeout_info is present.
  EXPECT_FALSE(result);
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 30);
  EXPECT_NE(err.find("canTransform returned after"), std::string::npos);
}

TEST_F(BufferTest, canTransform_5arg_with_timeout_polls_then_appends_timeout_info)
{
  // Arrange
  agnocast::Buffer buffer(clock_);
  buffer.setUsingDedicatedThread(true);
  std::string err;

  // Act
  bool result =
    buffer.canTransform("a", kStamp, "b", kStamp, "fixed", tf2::durationFromSec(0.05), &err);

  // Assert
  EXPECT_FALSE(result);
  EXPECT_NE(err.find("canTransform returned after"), std::string::npos);
}

// =========================================
// rclcpp::Time / rclcpp::Duration overloads
// =========================================

TEST_F(BufferTest, lookupTransform_3arg_rclcpp_overload_routes_to_tf2_overload)
{
  // Arrange
  agnocast::Buffer buffer(clock_);
  buffer.setTransform(make_transform("map", "base_link", 4.5, kStamp), "test_authority");

  // Act
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  auto out = buffer.lookupTransform("map", "base_link", t, rclcpp::Duration::from_seconds(0));

  // Assert: same payload reaches the underlying tf2-typed implementation.
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 4.5);
}

TEST_F(BufferTest, lookupTransform_5arg_rclcpp_overload_routes_to_tf2_overload)
{
  // Arrange
  agnocast::Buffer buffer(clock_);
  buffer.setTransform(make_transform("map", "odom", 1.0, kStamp), "test_authority");
  buffer.setTransform(make_transform("odom", "base_link", 2.0, kStamp), "test_authority");

  // Act
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  auto out =
    buffer.lookupTransform("map", t, "base_link", t, "odom", rclcpp::Duration::from_seconds(0));

  // Assert: composed translation reaches the rclcpp overload through fromRclcpp().
  EXPECT_DOUBLE_EQ(out.transform.translation.x, 3.0);
}

TEST_F(BufferTest, canTransform_3arg_rclcpp_overload_routes_to_tf2_overload)
{
  // Arrange: setUsingDedicatedThread(true) bypasses the Humble (rclcpp<28) threading
  // guard so this test exercises only the rclcpp->tf2 overload-routing path.
  agnocast::Buffer buffer(clock_);
  buffer.setUsingDedicatedThread(true);
  buffer.setTransform(make_transform("map", "base_link", 1.0, kStamp), "test_authority");

  // Act
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  bool result = buffer.canTransform("map", "base_link", t, rclcpp::Duration::from_seconds(0));

  // Assert
  EXPECT_TRUE(result);
}

TEST_F(BufferTest, canTransform_5arg_rclcpp_overload_routes_to_tf2_overload)
{
  // Arrange: see the 3-arg routing test above for why setUsingDedicatedThread is set.
  agnocast::Buffer buffer(clock_);
  buffer.setUsingDedicatedThread(true);
  buffer.setTransform(make_transform("map", "odom", 1.0, kStamp), "test_authority");
  buffer.setTransform(make_transform("odom", "base_link", 2.0, kStamp), "test_authority");

  // Act
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  bool result =
    buffer.canTransform("map", t, "base_link", t, "odom", rclcpp::Duration::from_seconds(0));

  // Assert
  EXPECT_TRUE(result);
}

// onTimeJump is registered as a clock jump callback in the constructor.
// Triggering a real jump requires driving rcl_clock through APIs not exposed
// publicly on rclcpp::Clock; the handler logic itself (clear() on backward
// jumps and clock-source changes, no-op on forward jumps) is unchanged from
// upstream tf2_ros::Buffer and is covered by upstream tf2_ros tests.
