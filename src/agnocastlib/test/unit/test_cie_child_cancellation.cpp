#include "agnocast/agnocast_callback_isolated_executor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

namespace
{

using namespace std::chrono_literals;

class CieChildCancellationTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  // A deadline bounds a failing test, not the production shutdown protocol.
  bool wait_for_registration(
    const rclcpp::CallbackGroup::SharedPtr & group, const std::future<void> & finished)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!group->get_associated_with_executor_atomic().load()) {
      if (
        finished.wait_for(0ms) == std::future_status::ready ||
        std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::yield();
    }
    return true;
  }
};

// Stress the narrow interval between child registration and entry into spin(). The helper's
// unit test forces that ordering deterministically; this test covers the real CIE wiring.
TEST_F(CieChildCancellationTest, CancelImmediatelyAfterChildRegistration)
{
  for (int iteration = 0; iteration < 32; ++iteration) {
    SCOPED_TRACE(iteration);
    auto node = std::make_shared<rclcpp::Node>("cie_cancel_startup_test");
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto executor =
      std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>(rclcpp::ExecutorOptions{}, 1, 1);
    executor->add_callback_group(group, node->get_node_base_interface());

    std::packaged_task<void()> spin([executor]() { executor->spin(); });
    auto finished = spin.get_future();
    std::thread worker(std::move(spin));

    const bool registered = wait_for_registration(group, finished);
    executor->cancel();
    const auto result = finished.wait_for(2s);
    if (result != std::future_status::ready) {
      rclcpp::shutdown();
    }
    worker.join();
    ASSERT_TRUE(registered);
    ASSERT_EQ(result, std::future_status::ready);
    EXPECT_NO_THROW(finished.get());
  }
}

TEST_F(CieChildCancellationTest, StopCallbackGroupImmediatelyAfterChildRegistration)
{
  for (int iteration = 0; iteration < 32; ++iteration) {
    SCOPED_TRACE(iteration);
    auto node = std::make_shared<rclcpp::Node>("cie_stop_startup_test");
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto executor =
      std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>(rclcpp::ExecutorOptions{}, 1, 1);
    executor->add_callback_group(group, node->get_node_base_interface());

    std::packaged_task<void()> spin([executor]() { executor->spin(); });
    auto finished = spin.get_future();
    std::thread worker(std::move(spin));
    const bool registered = wait_for_registration(group, finished);

    auto stopped =
      std::async(std::launch::async, [executor, group]() { executor->stop_callback_group(group); });
    const auto result = stopped.wait_for(2s);
    executor->cancel();
    if (result != std::future_status::ready || finished.wait_for(2s) != std::future_status::ready) {
      rclcpp::shutdown();
    }
    worker.join();
    stopped.wait();
    ASSERT_TRUE(registered);
    ASSERT_EQ(result, std::future_status::ready);
    EXPECT_NO_THROW(stopped.get());
    EXPECT_NO_THROW(finished.get());
  }
}

}  // namespace
