#include "node_for_executor_test.hpp"

#include <gtest/gtest.h>

#include <thread>

class SingleThreadedAgnocastExecutorNoStarvationTest : public ::testing::TestWithParam<int>
{
protected:
  void SetUp() override
  {
    int next_exec_timeout_ms = GetParam();

    // Set the execution time of each callback
    uint64_t num_cbs = NUM_AGNOCAST_SUB_CBS + NUM_AGNOCAST_CBS_TO_BE_ADDED + NUM_ROS2_SUB_CBS;
    std::chrono::milliseconds cb_exec_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      PUB_PERIOD * CPU_UTILIZATION / (num_cbs));

    // Set the spin duration
    std::chrono::seconds buffer = std::chrono::seconds(3);  // Rough value
    spin_duration_ = std::max(
                       std::chrono::seconds(
                         (next_exec_timeout_ms + cb_exec_time.count()) *
                         (NUM_AGNOCAST_SUB_CBS + NUM_AGNOCAST_CBS_TO_BE_ADDED) / 1000),
                       std::chrono::duration_cast<std::chrono::seconds>(
                         PUB_PERIOD * NUM_AGNOCAST_CBS_TO_BE_ADDED)) +
                     buffer;

    // Initialize the executor and the test node
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>(
      rclcpp::ExecutorOptions{}, GetParam());
    test_node_ = std::make_shared<NodeForExecutorTest>(
      NUM_AGNOCAST_SUB_CBS, NUM_ROS2_SUB_CBS, NUM_AGNOCAST_CBS_TO_BE_ADDED, PUB_PERIOD,
      cb_exec_time);
    executor_->add_node(test_node_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<NodeForExecutorTest> test_node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::chrono::seconds spin_duration_;

  // Fixed Parameters
  const uint64_t NUM_ROS2_SUB_CBS = 10;
  const uint64_t NUM_AGNOCAST_SUB_CBS = 10;
  const uint64_t NUM_AGNOCAST_CBS_TO_BE_ADDED = 5;
  const std::chrono::milliseconds PUB_PERIOD = std::chrono::milliseconds(100);
  const float CPU_UTILIZATION = 0.7;
};

INSTANTIATE_TEST_SUITE_P(
  SingleThreadedAgnocastExecutorNoStarvationTests, SingleThreadedAgnocastExecutorNoStarvationTest,
  ::testing::Values(25, 50, 100, 200, 400));  // next_exec_timeout_ms

TEST_P(SingleThreadedAgnocastExecutorNoStarvationTest, test_no_starvation)
{
  // Act
  std::thread spin_thread([this]() { this->executor_->spin(); });

  auto deadline = std::chrono::steady_clock::now() + spin_duration_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (test_node_->is_all_ros2_sub_cbs_called() && test_node_->is_all_agnocast_sub_cbs_called()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor_->cancel();
  spin_thread.join();

  // Assert
  EXPECT_TRUE(test_node_->is_all_ros2_sub_cbs_called());
  EXPECT_TRUE(test_node_->is_all_agnocast_sub_cbs_called());
}
