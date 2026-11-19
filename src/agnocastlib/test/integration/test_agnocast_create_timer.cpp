#include "agnocast/agnocast_timer_info.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class CreateTimerTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Clean up any timer info registered during tests
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info.clear();
  }
};

// =============================================================================
// Category 1: Wall-clock driven firing (no ROS time override)
//
// Specification:
//   - A timer fires its callback periodically based on the wall clock when
//     ROS time is not active.
//   - Multiple timers with different periods each fire at their own rate;
//     shorter periods fire at least as often as longer periods.
//   - A zero-period timer is always ready and fires repeatedly.
// =============================================================================

TEST_F(CreateTimerTest, CreateTimer_WallClock_CallbackFires)
{
  auto node = std::make_shared<agnocast::Node>("test_wall_clock_timer");

  std::atomic<int> callback_count{0};
  auto timer = node->create_timer(50ms, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for at least one callback (max 2s)
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (callback_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }

  executor->cancel();
  spin_thread.join();

  EXPECT_GT(callback_count.load(), 0) << "Timer callback should have fired at least once";
}

TEST_F(CreateTimerTest, CreateTimer_MultipleTimers_AllFire)
{
  auto node = std::make_shared<agnocast::Node>("test_multiple_timers");

  std::atomic<int> callback_count_50ms{0};
  std::atomic<int> callback_count_100ms{0};
  std::atomic<int> callback_count_150ms{0};

  auto timer_50ms = node->create_timer(50ms, [&callback_count_50ms]() { callback_count_50ms++; });
  auto timer_100ms =
    node->create_timer(100ms, [&callback_count_100ms]() { callback_count_100ms++; });
  auto timer_150ms =
    node->create_timer(150ms, [&callback_count_150ms]() { callback_count_150ms++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for all timers to fire at least once (max 2s)
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((callback_count_50ms.load() == 0 || callback_count_100ms.load() == 0 ||
          callback_count_150ms.load() == 0) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }

  executor->cancel();
  spin_thread.join();

  EXPECT_GT(callback_count_50ms.load(), 0) << "50ms timer should have fired";
  EXPECT_GT(callback_count_100ms.load(), 0) << "100ms timer should have fired";
  EXPECT_GT(callback_count_150ms.load(), 0) << "150ms timer should have fired";

  // 50ms timer should fire more frequently than 100ms and 150ms timers
  EXPECT_GE(callback_count_50ms.load(), callback_count_100ms.load())
    << "50ms timer should fire at least as often as 100ms timer";
  EXPECT_GE(callback_count_100ms.load(), callback_count_150ms.load())
    << "100ms timer should fire at least as often as 150ms timer";
}

TEST_F(CreateTimerTest, CreateTimer_ZeroPeriod_AlwaysReady)
{
  auto node = std::make_shared<agnocast::Node>("test_zero_period");

  std::atomic<int> callback_count{0};
  auto timer = node->create_timer(0ms, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for multiple callbacks (zero period should fire rapidly)
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (callback_count.load() < 10 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }

  executor->cancel();
  spin_thread.join();

  // Zero period timer should fire many times in 500ms
  EXPECT_GE(callback_count.load(), 10)
    << "Zero period timer should have fired many times (at least 10)";
}

// =============================================================================
// Category 2: ROS time activation (mid-flight transition)
//
// Specification:
//   - Once ROS time is activated, the wall clock no longer drives the timer;
//     no callback fires unless sim-time is advanced.
//   - Sim-time advances past next_call_time then fire the callback.
// =============================================================================

TEST_F(CreateTimerTest, CreateTimer_SimTimeActivation_CallbackFires)
{
  auto node = std::make_shared<agnocast::Node>("test_sim_time_activation");

  std::atomic<int> callback_count{0};
  auto timer = node->create_timer(100ms, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for at least one wall-clock callback first
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(callback_count.load(), 0) << "Wall-clock callback should have fired first";
  }

  const int count_before_sim = callback_count.load();

  // Enable ROS time override (timer transitions from wall-clock driven to sim-time driven)
  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to enable ros time override";
  }

  // After activation, the wall clock must no longer drive the timer.
  // Wait > period in wall-clock time without advancing sim time and verify no firing.
  const int count_at_activation = callback_count.load();
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(callback_count.load(), count_at_activation)
    << "Wall clock should not drive the timer after ROS time activation";

  // First set: ROS time goes from 0 → 1s. Since next_call_time was based on system time (~epoch),
  // this looks like a backward jump from the timer's perspective and resets next_call_time
  // to 1s + period, but does NOT trigger a firing.
  const rcl_time_point_value_t sim_time_1s = 1000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, sim_time_1s);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set ros time override (1s)";
  }

  std::this_thread::sleep_for(50ms);

  // Second set: forward jump from 1s → 1.2s, past the (reset) next_call_time of 1.1s.
  const rcl_time_point_value_t sim_time_1200ms = 1200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, sim_time_1200ms);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set ros time override (1.2s)";
  }

  // Wait for callback to fire after the sim-time advance
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() <= count_before_sim &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  executor->cancel();
  spin_thread.join();

  EXPECT_GT(callback_count.load(), count_before_sim)
    << "Callback should have fired after sim-time advance past next_call_time";
}

// =============================================================================
// Category 3: ROS time as the driver from create
//
// Specification:
//   - A timer created while ROS time is already active is never driven by
//     the wall clock; only sim-time advances fire it.
//   - Multiple timers with different periods fire when sim-time crosses
//     each timer's next_call_time, independently and at most once per crossing.
// =============================================================================

TEST_F(CreateTimerTest, CreateTimer_RosTimeAlreadyActive)
{
  auto node = std::make_shared<agnocast::Node>("test_ros_time_already_active");

  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();

  // Enable ROS time BEFORE creating the timer
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to enable ros time override";
  }

  // Set initial ROS time
  const rcl_time_point_value_t initial_time = 1000000000LL;  // 1s
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, initial_time);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set initial ros time";
  }

  // Now create the timer (ROS time is already active)
  std::atomic<int> callback_count{0};
  const auto period = 100ms;
  auto timer = node->create_timer(period, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wall clock must not drive the timer when ROS time is active at create.
  // Wait > period in wall-clock without advancing sim time and verify no firing.
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(callback_count.load(), 0)
    << "Wall clock should not drive the timer when ROS time was already active at create";

  // Forward jump to trigger callback (1s → 1.2s, next_call = 1.1s)
  const rcl_time_point_value_t time_1200ms = 1200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_1200ms);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set ros time override";
  }

  // Wait for callback
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  executor->cancel();
  spin_thread.join();

  EXPECT_GT(callback_count.load(), 0)
    << "Callback should have fired after sim-time advance when ROS time was already active";
}

TEST_F(CreateTimerTest, CreateTimer_MultipleTimers_SimTime_AllFire)
{
  auto node = std::make_shared<agnocast::Node>("test_multiple_timers_sim_time");

  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();

  // Enable ROS time before creating timers
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to enable ros time override";
  }

  // Set initial ROS time to 1s
  const rcl_time_point_value_t initial_time = 1000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, initial_time);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set initial ros time";
  }

  // Create multiple timers with different periods
  std::atomic<int> callback_count_100ms{0};
  std::atomic<int> callback_count_200ms{0};
  std::atomic<int> callback_count_300ms{0};

  auto timer_100ms =
    node->create_timer(100ms, [&callback_count_100ms]() { callback_count_100ms++; });
  auto timer_200ms =
    node->create_timer(200ms, [&callback_count_200ms]() { callback_count_200ms++; });
  auto timer_300ms =
    node->create_timer(300ms, [&callback_count_300ms]() { callback_count_300ms++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Small delay for executor to start
  std::this_thread::sleep_for(50ms);

  // Forward jump to 1.15s: fires 100ms timer (next_call=1.1s), not others
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, 1150000000LL);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // Wait for 100ms timer callback
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (callback_count_100ms.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }
  EXPECT_EQ(callback_count_100ms.load(), 1) << "100ms timer should have fired once at 1.15s";
  EXPECT_EQ(callback_count_200ms.load(), 0) << "200ms timer should not have fired yet";
  EXPECT_EQ(callback_count_300ms.load(), 0) << "300ms timer should not have fired yet";

  // Forward jump to 1.25s: fires 100ms timer again (next_call=1.2s), 200ms timer (next_call=1.2s)
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, 1250000000LL);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // Wait for callbacks
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while ((callback_count_100ms.load() < 2 || callback_count_200ms.load() < 1) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }
  EXPECT_EQ(callback_count_100ms.load(), 2) << "100ms timer should have fired twice by 1.25s";
  EXPECT_EQ(callback_count_200ms.load(), 1) << "200ms timer should have fired once by 1.25s";
  EXPECT_EQ(callback_count_300ms.load(), 0) << "300ms timer should not have fired yet";

  // Forward jump to 1.35s: fires 100ms (next_call=1.3s), 300ms (next_call=1.3s)
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, 1350000000LL);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // Wait for callbacks
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while ((callback_count_100ms.load() < 3 || callback_count_300ms.load() < 1) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }
  EXPECT_EQ(callback_count_100ms.load(), 3) << "100ms timer should have fired 3 times by 1.35s";
  EXPECT_EQ(callback_count_200ms.load(), 1) << "200ms timer should still be at 1 by 1.35s";
  EXPECT_EQ(callback_count_300ms.load(), 1) << "300ms timer should have fired once by 1.35s";

  executor->cancel();
  spin_thread.join();
}

// =============================================================================
// Category 4: Sim-time jumps
//
// Specification:
//   - A backward sim-time jump does not fire the callback by itself; it
//     resets next_call_time so that a subsequent forward advance past the
//     reset value fires the callback.
//   - A large forward sim-time jump fires the callback at most a small
//     bounded number of times, not once per skipped period.
// =============================================================================

TEST_F(CreateTimerTest, CreateTimer_SimTime_BackwardJump)
{
  auto node = std::make_shared<agnocast::Node>("test_sim_time_backward_jump");

  std::atomic<int> callback_count{0};
  const auto period = 100ms;
  auto timer = node->create_timer(period, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for initial wall-clock callback
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(callback_count.load(), 0);
  }

  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();

  // Enable ROS time override
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK);
  }
  std::this_thread::sleep_for(50ms);

  // Initialize sim time to 1s (backward jump from system time perspective, resets next_call)
  const rcl_time_point_value_t time_1s = 1000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_1s);
    ASSERT_EQ(ret, RCL_RET_OK);
  }
  std::this_thread::sleep_for(50ms);

  // Forward jump to 1.2s to fire the callback (next_call = 1.1s <= 1.2s)
  const rcl_time_point_value_t time_1200ms = 1200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_1200ms);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  const int count_after_forward = callback_count.load();
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() <= count_after_forward &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(callback_count.load(), count_after_forward)
      << "Callback should have fired at 1.2s sim time";
  }

  const int count_before_backward = callback_count.load();

  // Now jump backward to 0.2s
  const rcl_time_point_value_t time_200ms = 200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_200ms);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // Backward jump itself must not fire the callback.
  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(callback_count.load(), count_before_backward)
    << "Backward sim-time jump should not fire the callback";

  // Advance clock past new next_call_time to trigger callback
  const int count_before_advance = callback_count.load();
  const rcl_time_point_value_t time_400ms = 400000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_400ms);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() <= count_before_advance &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  executor->cancel();
  spin_thread.join();

  EXPECT_GT(callback_count.load(), count_before_advance)
    << "Callback should have fired after advancing past the reset next_call_time";
}

TEST_F(CreateTimerTest, CreateTimer_LargeForwardJump_SingleCallback)
{
  auto node = std::make_shared<agnocast::Node>("test_large_forward_jump");

  std::atomic<int> callback_count{0};
  const auto period = 100ms;
  auto timer = node->create_timer(period, [&callback_count]() { callback_count++; });

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  executor->add_node(node);

  std::thread spin_thread([&executor]() { executor->spin(); });

  // Wait for initial wall-clock callback
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(callback_count.load(), 0) << "Initial callback should have fired";
  }

  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();

  // Enable ROS time override
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK);
  }
  std::this_thread::sleep_for(50ms);

  // Set initial ROS time to 1s
  const rcl_time_point_value_t time_1s = 1000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_1s);
    ASSERT_EQ(ret, RCL_RET_OK);
  }
  std::this_thread::sleep_for(50ms);

  const int count_before_jump = callback_count.load();

  // Large forward jump: 1s → 2s (10 periods worth of jump)
  // The jump itself should fire the callback at most a small bounded number of times,
  // not once per skipped period.
  const rcl_time_point_value_t time_2s = 2000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_2s);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // Wait for callback to fire
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (callback_count.load() <= count_before_jump &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  // Small additional wait to ensure no extra callbacks fire
  std::this_thread::sleep_for(100ms);

  executor->cancel();
  spin_thread.join();

  const int callbacks_from_jump = callback_count.load() - count_before_jump;

  // Large forward jump should result in exactly 1 callback trigger from the jump itself
  // (The callback execution may update next_call_time, but the jump only writes once)
  EXPECT_GE(callbacks_from_jump, 1)
    << "At least one callback should have fired after the large forward jump";
  EXPECT_LE(callbacks_from_jump, 2)
    << "Large forward jump should not cause excessive callbacks (expected 1-2, got "
    << callbacks_from_jump << ")";
}
