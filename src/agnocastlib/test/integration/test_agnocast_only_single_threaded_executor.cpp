#include <agnocast/agnocast.hpp>
#include <agnocast/node/agnocast_only_executor.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace
{

template <class Predicate>
bool wait_until(Predicate predicate, std::chrono::nanoseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();

  while (!predicate()) {
    if ((std::chrono::steady_clock::now() - start) >= timeout) {
      return false;
    }

    std::this_thread::sleep_for(1ms);
  }

  return true;
}

template <class Function>
std::chrono::steady_clock::duration measure_elapsed_time(Function && function)
{
  const auto start = std::chrono::steady_clock::now();
  function();
  return std::chrono::steady_clock::now() - start;
}

}  // namespace

class AgnocastOnlySingleThreadedExecutorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    agnocast::init(0, nullptr);

    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();

    node = std::make_shared<agnocast::Node>("node", test_name.str());
  }

  void TearDown() override
  {
    node.reset();

    agnocast::shutdown();
  }

  std::shared_ptr<agnocast::Node> node;
};

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_is_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  ASSERT_FALSE(executor.is_spinning());

  std::thread spinner([&]() { executor.spin(); });

  ASSERT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));

  // Act
  executor.cancel();
  spinner.join();

  // Assert
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> timer_completed{false};
  [[maybe_unused]] auto timer =
    this->node->create_wall_timer(1ms, [&]() { timer_completed.store(true); });

  executor.add_node(this->node);

  // Act
  executor.spin_once();

  // Assert
  EXPECT_TRUE(wait_until([&]() { return timer_completed.load(); }, 10s));

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_with_no_timeout_blocks_until_cancel)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::atomic<bool> spin_once_returned{false};

  std::thread spinner([&]() {
    executor.spin_once(std::chrono::nanoseconds(-1));
    spin_once_returned.store(true);
  });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));
  EXPECT_FALSE(spin_once_returned.load());

  // Act
  executor.cancel();
  spinner.join();

  // Assert
  EXPECT_TRUE(spin_once_returned.load());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_with_zero_timeout_returns_immediately)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_once(0ns); });

  // Assert
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_waits_until_timeout)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_once(100ms); });

  // Assert
  EXPECT_GE(elapsed, 90ms);
  EXPECT_LT(elapsed, 1s);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_executes_only_one_ready_callback)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<int> callback_count{0};

  [[maybe_unused]] auto timer1 =
    this->node->create_wall_timer(1ms, [&]() { callback_count.fetch_add(1); });

  [[maybe_unused]] auto timer2 =
    this->node->create_wall_timer(1ms, [&]() { callback_count.fetch_add(1); });

  executor.add_node(this->node);

  ASSERT_TRUE(wait_until([&]() { return callback_count.load() == 0; }, 10ms));

  std::this_thread::sleep_for(10ms);

  // Act
  executor.spin_once(100ms);

  // Assert
  EXPECT_EQ(1, callback_count.load());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_throws_if_already_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::thread spinner([&]() { executor.spin(); });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));

  // Act / Assert
  EXPECT_THROW(executor.spin_once(0ns), std::runtime_error);

  // Cleanup
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once_sets_is_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::atomic<bool> spin_once_returned{false};

  std::thread spinner([&]() {
    executor.spin_once(std::chrono::nanoseconds(-1));
    spin_once_returned.store(true);
  });

  // Act
  const bool became_spinning = wait_until([&]() { return executor.is_spinning(); }, 10s);

  // Assert
  EXPECT_TRUE(became_spinning);
  EXPECT_FALSE(spin_once_returned.load());

  // Cleanup
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_until_future_complete)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  auto shared_future = future.share();

  promise.set_value(true);

  // Act
  const auto elapsed = measure_elapsed_time([&]() {
    const auto ret = executor.spin_until_future_complete(shared_future, 1s);

    // Assert
    EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  });

  // Assert
  EXPECT_LT(elapsed, 500ms);

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest,
  test_spin_until_future_complete_with_no_timeout_blocks_until_cancel)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  std::promise<rclcpp::FutureReturnCode> return_code_promise;
  auto return_code_future = return_code_promise.get_future();

  std::atomic<bool> spin_returned{false};

  std::thread spinner([&]() {
    const auto ret =
      executor.spin_until_future_complete(shared_future, std::chrono::nanoseconds(-1));
    return_code_promise.set_value(ret);
    spin_returned.store(true);
  });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));
  EXPECT_FALSE(spin_returned.load());

  // Act
  executor.cancel();
  spinner.join();

  // Assert
  EXPECT_TRUE(spin_returned.load());
  EXPECT_EQ(rclcpp::FutureReturnCode::INTERRUPTED, return_code_future.get());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest,
  test_spin_until_future_complete_with_zero_timeout_returns_immediately)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  rclcpp::FutureReturnCode ret = rclcpp::FutureReturnCode::SUCCESS;

  // Act
  const auto elapsed =
    measure_elapsed_time([&]() { ret = executor.spin_until_future_complete(shared_future, 0ns); });

  // Assert
  EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_until_future_complete_waits_until_timeout)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  rclcpp::FutureReturnCode ret = rclcpp::FutureReturnCode::SUCCESS;

  // Act
  const auto elapsed = measure_elapsed_time(
    [&]() { ret = executor.spin_until_future_complete(shared_future, 100ms); });

  // Assert
  EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
  EXPECT_GE(elapsed, 90ms);
  EXPECT_LT(elapsed, 1s);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest,
  test_spin_until_future_complete_returns_success_when_future_is_completed_by_callback)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  std::atomic<bool> promise_set{false};

  [[maybe_unused]] auto timer = this->node->create_wall_timer(1ms, [&]() {
    if (!promise_set.exchange(true)) {
      promise.set_value(true);
    }
  });

  executor.add_node(this->node);

  // Act
  const auto ret = executor.spin_until_future_complete(shared_future, 1s);

  // Assert
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  EXPECT_TRUE(promise_set.load());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest,
  test_spin_until_future_complete_throws_if_already_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  std::thread spinner([&]() { executor.spin(); });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));

  // Act / Assert
  EXPECT_THROW(executor.spin_until_future_complete(shared_future, 1ms), std::runtime_error);

  // Cleanup
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_until_future_complete_sets_is_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  auto shared_future = promise.get_future().share();

  std::promise<rclcpp::FutureReturnCode> return_code_promise;
  auto return_code_future = return_code_promise.get_future();

  std::atomic<bool> spin_returned{false};

  std::thread spinner([&]() {
    const auto ret =
      executor.spin_until_future_complete(shared_future, std::chrono::nanoseconds(-1));
    return_code_promise.set_value(ret);
    spin_returned.store(true);
  });

  // Act
  const bool became_spinning = wait_until([&]() { return executor.is_spinning(); }, 10s);

  // Assert
  EXPECT_TRUE(became_spinning);
  EXPECT_FALSE(spin_returned.load());

  // Cleanup
  executor.cancel();
  spinner.join();

  EXPECT_EQ(rclcpp::FutureReturnCode::INTERRUPTED, return_code_future.get());

  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_some)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> timer_completed{false};
  [[maybe_unused]] auto timer =
    this->node->create_wall_timer(1ms, [&]() { timer_completed.store(true); });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  // Act
  executor.spin_some(1s);

  // Assert
  EXPECT_TRUE(timer_completed.load());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest, test_spin_some_with_zero_max_duration_returns_immediately)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_some(0ns); });

  // Assert
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_some_does_not_wait_until_max_duration)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_some(100ms); });

  // Assert
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_some_executes_ready_callback)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<int> callback_count{0};

  [[maybe_unused]] auto timer =
    this->node->create_wall_timer(1ms, [&]() { callback_count.fetch_add(1); });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  // Act
  executor.spin_some(1s);

  // Assert
  EXPECT_GE(callback_count.load(), 1);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_some_throws_if_already_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::thread spinner([&]() { executor.spin(); });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));

  // Act / Assert
  EXPECT_THROW(executor.spin_some(0ns), std::runtime_error);

  // Cleanup
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_some_sets_is_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> callback_started{false};
  std::atomic<bool> allow_callback_return{false};
  std::atomic<bool> spin_some_returned{false};

  [[maybe_unused]] auto timer = this->node->create_wall_timer(1ms, [&]() {
    callback_started.store(true);

    while (!allow_callback_return.load()) {
      std::this_thread::sleep_for(1ms);
    }
  });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  std::thread spinner([&]() {
    executor.spin_some(1s);
    spin_some_returned.store(true);
  });

  // Act
  const bool became_spinning =
    wait_until([&]() { return callback_started.load() && executor.is_spinning(); }, 10s);

  // Assert
  EXPECT_TRUE(became_spinning);
  EXPECT_FALSE(spin_some_returned.load());

  // Cleanup
  allow_callback_return.store(true);
  spinner.join();

  EXPECT_TRUE(spin_some_returned.load());
  EXPECT_FALSE(executor.is_spinning());

  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_all)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> timer_completed{false};
  [[maybe_unused]] auto timer =
    this->node->create_wall_timer(1ms, [&]() { timer_completed.store(true); });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  // Act
  executor.spin_all(1s);

  // Assert
  EXPECT_TRUE(timer_completed.load());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest, test_spin_all_with_zero_max_duration_returns_immediately)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_all(0ns); });

  // Assert
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_all_does_not_wait_until_max_duration)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_all(100ms); });

  // Assert
  EXPECT_LT(elapsed, 100ms);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.remove_node(this->node, true);
}

TEST_F(
  AgnocastOnlySingleThreadedExecutorTest,
  test_spin_all_executes_callback_that_becomes_ready_while_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<int> first_callback_count{0};
  std::atomic<int> second_callback_count{0};

  agnocast::TimerBase::SharedPtr first_timer;
  agnocast::TimerBase::SharedPtr second_timer;

  first_timer = this->node->create_wall_timer(1ms, [&]() {
    first_callback_count.fetch_add(1);
    first_timer->cancel();

    // Make the second timer become ready after the initial non-blocking wait.
    std::this_thread::sleep_for(50ms);
  });

  second_timer = this->node->create_wall_timer(30ms, [&]() {
    second_callback_count.fetch_add(1);
    second_timer->cancel();
  });

  executor.add_node(this->node);

  // Make only the first timer ready before spin_all starts.
  std::this_thread::sleep_for(10ms);

  // Act
  executor.spin_all(1s);

  // Assert
  EXPECT_EQ(1, first_callback_count.load());
  EXPECT_EQ(1, second_callback_count.load());
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_all_respects_max_duration)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<int> callback_count{0};

  [[maybe_unused]] auto timer = this->node->create_wall_timer(1ms, [&]() {
    callback_count.fetch_add(1);
    std::this_thread::sleep_for(2ms);
  });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  // Act
  const auto elapsed = measure_elapsed_time([&]() { executor.spin_all(100ms); });

  // Assert
  EXPECT_GE(elapsed, 90ms);
  EXPECT_LT(elapsed, 1s);
  EXPECT_GT(callback_count.load(), 1);
  EXPECT_FALSE(executor.is_spinning());

  // Cleanup
  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_all_throws_if_already_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::thread spinner([&]() { executor.spin(); });

  EXPECT_TRUE(wait_until([&]() { return executor.is_spinning(); }, 10s));

  // Act / Assert
  EXPECT_THROW(executor.spin_all(0ns), std::runtime_error);

  // Cleanup
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_all_sets_is_spinning)
{
  // Arrange
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> callback_started{false};
  std::atomic<bool> allow_callback_return{false};
  std::atomic<bool> spin_all_returned{false};

  agnocast::TimerBase::SharedPtr timer;

  timer = this->node->create_wall_timer(1ms, [&]() {
    callback_started.store(true);
    timer->cancel();

    while (!allow_callback_return.load()) {
      std::this_thread::sleep_for(1ms);
    }
  });

  executor.add_node(this->node);

  std::this_thread::sleep_for(10ms);

  std::thread spinner([&]() {
    executor.spin_all(1s);
    spin_all_returned.store(true);
  });

  // Act
  const bool became_spinning =
    wait_until([&]() { return callback_started.load() && executor.is_spinning(); }, 10s);

  // Assert
  EXPECT_TRUE(became_spinning);
  EXPECT_FALSE(spin_all_returned.load());

  // Cleanup
  allow_callback_return.store(true);
  spinner.join();

  EXPECT_TRUE(spin_all_returned.load());
  EXPECT_FALSE(executor.is_spinning());

  executor.cancel();
  executor.remove_node(this->node, true);
}