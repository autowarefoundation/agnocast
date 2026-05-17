// Spec tests for the public Agnocast timer API. White-box tests (internal-state
// peeks, direct calls to internal helpers) live in test_agnocast_timer.cpp.

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

// =========================================
// create_timer free function tests
// =========================================

class CreateTimerFreeFunctionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node = std::make_shared<agnocast::Node>("test_timer_node", options);
  }

  void TearDown() override { node.reset(); }

  std::shared_ptr<agnocast::Node> node;
};

TEST_F(CreateTimerFreeFunctionTest, create_timer_returns_timer_with_specified_clock)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Assert
  ASSERT_NE(timer, nullptr);
  EXPECT_EQ(timer->get_clock()->get_clock_type(), RCL_STEADY_TIME);
}

TEST_F(CreateTimerFreeFunctionTest, callback_is_invoked)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(10));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  timer->execute_callback();

  // Assert
  EXPECT_TRUE(called);
}

TEST_F(CreateTimerFreeFunctionTest, callback_with_timer_base_argument_receives_self)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(10));
  agnocast::TimerBase * captured = nullptr;
  auto timer = agnocast::create_timer(
    node.get(), clock, period, [&captured](agnocast::TimerBase & self) { captured = &self; });

  // Act
  timer->execute_callback();

  // Assert
  EXPECT_EQ(captured, timer.get());
}

TEST_F(CreateTimerFreeFunctionTest, is_steady_reflects_clock_type)
{
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // STEADY_TIME
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    auto timer = agnocast::create_timer(node.get(), clock, period, []() {});
    EXPECT_TRUE(timer->is_steady());
  }

  // ROS_TIME
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto timer = agnocast::create_timer(node.get(), clock, period, []() {});
    EXPECT_FALSE(timer->is_steady());
  }

  // SYSTEM_TIME
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    auto timer = agnocast::create_timer(node.get(), clock, period, []() {});
    EXPECT_FALSE(timer->is_steady());
  }
}

// =========================================
// cancel and reset and time_until_trigger function tests
// =========================================

TEST_F(CreateTimerFreeFunctionTest, new_timer_starts_running)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // Act: agnocast does not support autostart=false, so a freshly created timer
  // must already be running (not canceled) with the next trigger within one period.
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Assert
  EXPECT_FALSE(timer->is_canceled());
  const auto tut = timer->time_until_trigger();
  EXPECT_GT(tut, std::chrono::nanoseconds(0));
  EXPECT_LE(tut, period.to_chrono<std::chrono::nanoseconds>());
}

TEST_F(CreateTimerFreeFunctionTest, cancel_timer)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  timer->cancel();

  // Assert
  EXPECT_TRUE(timer->is_canceled());
  EXPECT_FALSE(called);
}

TEST_F(CreateTimerFreeFunctionTest, time_until_trigger_cancel_and_reset)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  auto tut_before_cancel = timer->time_until_trigger();
  timer->cancel();
  auto tut_after_cancel = timer->time_until_trigger();
  timer->reset();
  auto tut_after_reset = timer->time_until_trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto tut_after_wait = timer->time_until_trigger();

  // Assert
  EXPECT_TRUE(tut_before_cancel < std::chrono::milliseconds(100));
  EXPECT_EQ(tut_after_cancel, std::chrono::nanoseconds::max());
  EXPECT_TRUE(tut_after_reset < std::chrono::milliseconds(100));
  EXPECT_LT(tut_after_wait, std::chrono::nanoseconds(0));
  EXPECT_FALSE(called);
}

TEST_F(CreateTimerFreeFunctionTest, time_until_trigger_cancel_and_reset_ros_time)
{
  // Arrange — activate ROS time so the clock is fully controlled (no wall-clock noise).
  constexpr int64_t kT0Ns = 1'000'000'000;     // 1s
  constexpr int64_t kPeriodNs = 100'000'000;   // 100ms
  constexpr int64_t kAdvanceNs = 200'000'000;  // 200ms past t0
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns), RCL_RET_OK);
  }
  const auto period = rclcpp::Duration(std::chrono::nanoseconds(kPeriodNs));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  auto tut_before_cancel = timer->time_until_trigger();
  timer->cancel();
  auto tut_after_cancel = timer->time_until_trigger();
  timer->reset();
  auto tut_after_reset = timer->time_until_trigger();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns + kAdvanceNs), RCL_RET_OK);
  }
  auto tut_after_wait = timer->time_until_trigger();

  // Assert — values are exact because ROS time advances only when we say so.
  EXPECT_EQ(tut_before_cancel, std::chrono::nanoseconds(kPeriodNs));
  EXPECT_EQ(tut_after_cancel, std::chrono::nanoseconds::max());
  EXPECT_EQ(tut_after_reset, std::chrono::nanoseconds(kPeriodNs));
  EXPECT_EQ(tut_after_wait, std::chrono::nanoseconds(kPeriodNs - kAdvanceNs));
  EXPECT_FALSE(called);
}

TEST_F(CreateTimerFreeFunctionTest, reset_re_anchors_next_call_when_time_has_advanced)
{
  // Arrange
  constexpr int64_t kT0Ns = 1'000'000'000;
  constexpr int64_t kPeriodNs = 100'000'000;
  constexpr int64_t kHalfPeriodNs = 50'000'000;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns), RCL_RET_OK);
  }
  const auto period = rclcpp::Duration(std::chrono::nanoseconds(kPeriodNs));
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Act
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns + kHalfPeriodNs), RCL_RET_OK);
  }
  const auto tut_before_reset = timer->time_until_trigger();
  const bool was_canceled_before_reset = timer->is_canceled();
  timer->reset();
  const auto tut_after_reset = timer->time_until_trigger();

  // Assert
  EXPECT_FALSE(was_canceled_before_reset);
  EXPECT_EQ(tut_before_reset, std::chrono::nanoseconds(kPeriodNs - kHalfPeriodNs));
  EXPECT_EQ(tut_after_reset, std::chrono::nanoseconds(kPeriodNs));
}

// =========================================
// set_period function tests
// =========================================

TEST_F(CreateTimerFreeFunctionTest, set_period_does_not_change_immediate_time_until_trigger)
{
  // Arrange — pin ROS time so before/after time_until_trigger comparisons are exact.
  constexpr int64_t kT0Ns = 1'000'000'000;
  constexpr int64_t kPeriodNs = 100'000'000;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns), RCL_RET_OK);
  }
  const auto period = rclcpp::Duration(std::chrono::nanoseconds(kPeriodNs));
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Act — swap to a deliberately different period.
  const auto tut_before = timer->time_until_trigger();
  timer->set_period(std::chrono::nanoseconds{kPeriodNs * 5});
  const auto tut_after = timer->time_until_trigger();

  // Assert — the already-scheduled next firing must keep its time across the period swap.
  EXPECT_EQ(tut_before, std::chrono::nanoseconds(kPeriodNs));
  EXPECT_EQ(tut_after, tut_before);
}

TEST_F(CreateTimerFreeFunctionTest, set_period_takes_effect_on_subsequent_reset)
{
  // Arrange
  constexpr int64_t kT0Ns = 1'000'000'000;
  constexpr int64_t kInitialPeriodNs = 100'000'000;
  constexpr int64_t kNewPeriodNs = 50'000'000;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, kT0Ns), RCL_RET_OK);
  }
  auto timer = agnocast::create_timer(
    node.get(), clock, rclcpp::Duration(std::chrono::nanoseconds(kInitialPeriodNs)), []() {});

  // Act — reset() anchors next_call at now + period using the latest period value.
  timer->set_period(std::chrono::nanoseconds{kNewPeriodNs});
  timer->reset();

  // Assert
  EXPECT_EQ(timer->time_until_trigger(), std::chrono::nanoseconds(kNewPeriodNs));
}

TEST_F(CreateTimerFreeFunctionTest, set_period_preserves_canceled_state)
{
  // Arrange — set_period and cancel are orthogonal; calling set_period on a canceled
  // timer must not implicitly un-cancel it.
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});
  timer->cancel();
  ASSERT_TRUE(timer->is_canceled());

  // Act
  timer->set_period(std::chrono::nanoseconds{50'000'000});

  // Assert
  EXPECT_TRUE(timer->is_canceled());
}
