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
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override
  {
    // Clean up any timer info registered during tests
    {
      std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
      agnocast::id2_timer_info.clear();
    }
    rclcpp::shutdown();
  }
};

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

  // Enable ROS time override (triggers RCL_ROS_TIME_ACTIVATED jump → closes timer_fd)
  auto clock = node->get_clock();
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_enable_ros_time_override(rcl_clock);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to enable ros time override";
  }

  // Small delay to let the executor process the time jump
  std::this_thread::sleep_for(50ms);

  // First set: ROS time goes from 0 → 1s. Since next_call_time was based on system time (~epoch),
  // this looks like a backward jump from the timer's perspective and resets next_call_time
  // to 1s + period, but does NOT write to clock_eventfd.
  const rcl_time_point_value_t sim_time_1s = 1000000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, sim_time_1s);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set ros time override (1s)";
  }

  std::this_thread::sleep_for(50ms);

  // Second set: forward jump from 1s → 1.2s. Now next_call_time = 1.1s <= 1.2s,
  // so this triggers clock_eventfd write and fires the callback.
  const rcl_time_point_value_t sim_time_1200ms = 1200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, sim_time_1200ms);
    ASSERT_EQ(ret, RCL_RET_OK) << "Failed to set ros time override (1.2s)";
  }

  // Wait for callback to fire via clock_eventfd path
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
    << "Callback should have fired after sim time activation via clock_eventfd";
}

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

  // Now jump backward to 0.2s
  const rcl_time_point_value_t time_200ms = 200000000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    auto ret = rcl_set_ros_time_override(rcl_clock, time_200ms);
    ASSERT_EQ(ret, RCL_RET_OK);
  }

  // After backward jump, next_call_time should be reset to now + period
  std::this_thread::sleep_for(50ms);

  // Verify next_call_time_ns was reset appropriately
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    std::shared_ptr<agnocast::TimerInfo> found_info;
    for (const auto & [id, info] : agnocast::id2_timer_info) {
      if (info->timer.lock() == timer) {
        found_info = info;
        break;
      }
    }
    ASSERT_NE(found_info, nullptr) << "Timer info should be registered";
    const int64_t next_call_ns = found_info->next_call_time_ns.load(std::memory_order_relaxed);
    // After backward jump, next_call_time should be around time_200ms + period_ns
    const int64_t expected_next = time_200ms + std::chrono::nanoseconds(period).count();
    EXPECT_NEAR(next_call_ns, expected_next, std::chrono::nanoseconds(50ms).count())
      << "next_call_time_ns should be reset to now + period after backward jump";
  }

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
