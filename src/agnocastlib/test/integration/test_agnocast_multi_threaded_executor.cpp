#include "node_for_executor_test.hpp"

#include <gtest/gtest.h>

#include <thread>

class MultiThreadedAgnocastExecutorTest
: public ::testing::TestWithParam<std::tuple<bool, int, std::string>>
{
protected:
  void SetUp() override
  {
    bool yield_before_execute = std::get<0>(GetParam());
    int next_exec_timeout_ms = std::get<1>(GetParam());
    cbg_type_ = std::get<2>(GetParam());
    int agnocast_next_exec_timeout_ms = next_exec_timeout_ms;
    std::chrono::nanoseconds ros2_next_exec_timeout(
      static_cast<int64_t>(next_exec_timeout_ms) * 1000 * 1000);

    // Set the execution time of each callback
    uint64_t num_cbs = NUM_AGNOCAST_SUB_CBS + NUM_AGNOCAST_CBS_TO_BE_ADDED + NUM_ROS2_SUB_CBS;
    std::chrono::milliseconds cb_exec_time =
      (cbg_type_ == "mutually_exclusive")
        ? std::chrono::duration_cast<std::chrono::milliseconds>(
            PUB_PERIOD * CPU_UTILIZATION / (num_cbs))
        : std::chrono::duration_cast<std::chrono::milliseconds>(
            PUB_PERIOD * CPU_UTILIZATION * (NUMBER_OF_AGNOCAST_THREADS + NUMBER_OF_ROS2_THREADS) /
            (num_cbs));

    // Set the spin duration
    std::chrono::seconds buffer = std::chrono::seconds(1);  // Rough value
    spin_duration_ = std::max(
                       std::chrono::seconds(
                         (agnocast_next_exec_timeout_ms + cb_exec_time.count()) *
                         (NUM_AGNOCAST_SUB_CBS + NUM_AGNOCAST_CBS_TO_BE_ADDED) / 1000),
                       std::chrono::duration_cast<std::chrono::seconds>(
                         PUB_PERIOD * NUM_AGNOCAST_CBS_TO_BE_ADDED)) +
                     buffer;

    // Initialize the executor and the test node
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<agnocast::MultiThreadedAgnocastExecutor>(
      rclcpp::ExecutorOptions{}, NUMBER_OF_ROS2_THREADS, NUMBER_OF_AGNOCAST_THREADS,
      yield_before_execute, ros2_next_exec_timeout, agnocast_next_exec_timeout_ms);
    test_node_ = std::make_shared<NodeForExecutorTest>(
      NUM_AGNOCAST_SUB_CBS, NUM_ROS2_SUB_CBS, NUM_AGNOCAST_CBS_TO_BE_ADDED, PUB_PERIOD,
      cb_exec_time, cbg_type_);
    executor_->add_node(test_node_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::chrono::seconds spin_duration_;
  std::shared_ptr<NodeForExecutorTest> test_node_;
  std::shared_ptr<agnocast::MultiThreadedAgnocastExecutor> executor_;
  std::string cbg_type_;

  // Fixed parameters
  const std::chrono::milliseconds PUB_PERIOD = std::chrono::milliseconds(100);
  const size_t NUMBER_OF_ROS2_THREADS = 3;
  const size_t NUMBER_OF_AGNOCAST_THREADS = 3;
  const uint64_t NUM_ROS2_SUB_CBS = NUMBER_OF_ROS2_THREADS * 3;
  const uint64_t NUM_AGNOCAST_SUB_CBS = NUMBER_OF_AGNOCAST_THREADS * 3;
  const uint64_t NUM_AGNOCAST_CBS_TO_BE_ADDED = NUMBER_OF_AGNOCAST_THREADS * 2;
  const float CPU_UTILIZATION = 0.7;
};

INSTANTIATE_TEST_SUITE_P(
  MultiThreadedAgnocastExecutorTests, MultiThreadedAgnocastExecutorTest,
  ::testing::Combine(
    ::testing::Values(true, false),  // yield_before_execute
    ::testing::Values(
      25, 50, 100, 200, 400),  // ros2_next_exec_timeout and agnocast_next_exec_timeout_ms
    ::testing::Values("individual", "mutually_exclusive", "reentrant")));

TEST_P(MultiThreadedAgnocastExecutorTest, test_no_starvation_and_callback_group)
{
  // Act
  std::thread spin_thread([this]() { this->executor_->spin(); });
  std::this_thread::sleep_for(spin_duration_);
  executor_->cancel();
  spin_thread.join();

  // Assert
  EXPECT_TRUE(test_node_->is_all_ros2_sub_cbs_called());
  EXPECT_TRUE(test_node_->is_all_agnocast_sub_cbs_called());

  // The success rate of subsequent tests depends on the number of callbacks and CPU utilization.
  // With the current configuration, the test is almost certain to pass.
  if (cbg_type_ == "mutually_exclusive") {
    EXPECT_TRUE(test_node_->is_mutually_exclusive_agnocast());
    EXPECT_TRUE(test_node_->is_mutually_exclusive_ros2());
  } else {
    EXPECT_FALSE(test_node_->is_mutually_exclusive_agnocast());
    EXPECT_FALSE(test_node_->is_mutually_exclusive_ros2());
  }
}
