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
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

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

