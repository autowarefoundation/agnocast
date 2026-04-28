#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <gtest/gtest.h>

class CallbackIsolatedAgnocastExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor =
      std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>(rclcpp::ExecutorOptions());
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor;
};

TEST_F(CallbackIsolatedAgnocastExecutorTest, add_remove_callback_group)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  executor->add_callback_group(group, node->get_node_base_interface());

  auto groups = executor->get_manually_added_callback_groups();
  EXPECT_EQ(groups.size(), 1);
  EXPECT_TRUE(groups[0].lock() == group);

  executor->remove_callback_group(group);
  groups = executor->get_manually_added_callback_groups();
  EXPECT_EQ(groups.size(), 0);
}

TEST_F(CallbackIsolatedAgnocastExecutorTest, add_remove_node)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  executor->add_node(node->get_node_base_interface());

  auto nodes = executor->get_automatically_added_callback_groups_from_nodes();
  EXPECT_EQ(nodes.size(), 1);

  executor->remove_node(node->get_node_base_interface());
  nodes = executor->get_automatically_added_callback_groups_from_nodes();
  EXPECT_EQ(nodes.size(), 0);
}

TEST_F(CallbackIsolatedAgnocastExecutorTest, get_all_callback_groups)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto node2 = std::make_shared<rclcpp::Node>("test_node2");

  executor->add_callback_group(group, node->get_node_base_interface());
  executor->add_node(node2);

  auto groups = executor->get_all_callback_groups();
  EXPECT_EQ(groups.size(), 2);

  executor->remove_callback_group(group);
  groups = executor->get_all_callback_groups();
  EXPECT_EQ(groups.size(), 1);

  executor->remove_node(node2);
  groups = executor->get_all_callback_groups();
  EXPECT_EQ(groups.size(), 0);
}

// Verify that agnocast::Node's get_notify_guard_condition() does not throw
// when created with a valid context (i.e., after rclcpp::init()).
// This is required because rclcpp::Executor::add_node() calls
// get_notify_guard_condition() in add_callback_group_to_map() to register
// the node's guard condition in the executor's wait set.
// agnocast::Node uses its own epoll-based dispatch and does not need this
// guard condition, but it must be available for executor compatibility.
TEST_F(CallbackIsolatedAgnocastExecutorTest, agnocast_node_add_to_executor)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  auto node = std::make_shared<agnocast::Node>("test_agnocast_node", options);
  auto node_base = node->get_node_base_interface();

  EXPECT_NO_THROW(node_base->get_notify_guard_condition());
  EXPECT_NO_THROW(executor->add_node(node_base));

  auto groups = executor->get_automatically_added_callback_groups_from_nodes();
  EXPECT_EQ(groups.size(), 1);

  executor->remove_node(node_base);
  groups = executor->get_automatically_added_callback_groups_from_nodes();
  EXPECT_EQ(groups.size(), 0);
}

TEST_F(CallbackIsolatedAgnocastExecutorTest, cancel)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_node");
  bool timer_called = false;
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(10), [&timer_called]() { timer_called = true; });
  executor->add_node(node);
  bool spin_finished = false;
  std::thread spin_thread([this, &spin_finished]() {
    executor->spin();
    spin_finished = true;
  });
  while (!timer_called) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Act
  EXPECT_NO_THROW(executor->cancel());
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  // Assert
  EXPECT_TRUE(spin_finished) << "Spin should have finished after cancel";
}

// Regression test: stop_callback_group() must (1) cancel the correct child executor,
// (2) wait for an in-flight callback to finish, and (3) prevent further callbacks for that group.
TEST_F(CallbackIsolatedAgnocastExecutorTest, stop_callback_group_waits_and_stops)
{
  // Arrange: two callback groups, each with a timer.
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto group_a = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto group_b = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  std::atomic_bool callback_a_started{false};
  std::atomic_bool callback_a_finished{false};
  std::atomic_int callback_a_count{0};
  auto timer_a = node->create_wall_timer(
    std::chrono::milliseconds(10),
    [&]() {
      callback_a_started = true;
      // Hold the callback long enough for the main thread to call stop_callback_group()
      // while this callback is still in-flight.
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      callback_a_count++;
      callback_a_finished = true;
    },
    group_a);

  std::atomic_int callback_b_count{0};
  auto timer_b =
    node->create_wall_timer(std::chrono::milliseconds(10), [&]() { callback_b_count++; }, group_b);

  executor->add_node(node);

  std::thread spin_thread([this]() { executor->spin(); });

  // Wait for group_a's callback to start (it will sleep for 300ms inside).
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!callback_a_started) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline)
        << "Timed out waiting for callback_a to start";
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Act: stop group_a while its callback is in-flight.
  // This must block until the callback finishes.
  EXPECT_FALSE(callback_a_finished) << "Callback should still be running when stop is called";
  executor->stop_callback_group(group_a);

  // Assert (2): callback completed before stop_callback_group() returned.
  EXPECT_TRUE(callback_a_finished) << "stop_callback_group() must wait for in-flight callback";

  // Assert (3): no further callbacks for group_a after stop.
  int count_after_stop = callback_a_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(callback_a_count.load(), count_after_stop)
    << "No further callbacks should run for the stopped group";

  // Assert (1): group_b is unaffected and still running.
  int count_b_before = callback_b_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_GT(callback_b_count.load(), count_b_before) << "Other callback groups must keep running";

  // Cleanup
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

// Regression test: stop_callback_group() must not deadlock with spin() shutdown.
// spin()'s shutdown path also acquires child_resources_mutex_ to cancel and join child threads.
// If stop_callback_group() and spin() shutdown race, they must not deadlock.
TEST_F(CallbackIsolatedAgnocastExecutorTest, stop_callback_group_does_not_deadlock_with_cancel)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  std::atomic_bool callback_started{false};
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(10),
    [&]() {
      callback_started = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    },
    group);

  executor->add_node(node);

  std::atomic_bool spin_finished{false};
  std::thread spin_thread([this, &spin_finished]() {
    executor->spin();
    spin_finished = true;
  });

  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!callback_started) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline)
        << "Timed out waiting for callback to start";
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Race: stop_callback_group and cancel at the same time.
  std::thread stop_thread([this, &group]() { executor->stop_callback_group(group); });
  executor->cancel();

  auto start = std::chrono::steady_clock::now();
  constexpr auto timeout = std::chrono::seconds(10);
  while (!spin_finished) {
    ASSERT_LT(std::chrono::steady_clock::now() - start, timeout)
      << "Deadlock detected: spin() did not exit";
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (stop_thread.joinable()) {
    stop_thread.join();
  }
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  EXPECT_TRUE(spin_finished);
}

// Regression test: cancel() called from within a child thread's callback must not deadlock.
// cancel() acquires child_resources_mutex_, so spin()'s shutdown path must not hold
// that mutex while joining child threads.
//
// Deadlock sequence if child_resources_mutex_ is held during thread join:
//   1. Timer callback fires in a child thread, starts a long sleep
//   2. External cancel() sets spinning=false, causing spin()'s monitoring loop to exit
//   3. spin()'s shutdown acquires child_resources_mutex_ and tries to join child threads
//   4. Timer callback wakes from sleep, calls cancel(), blocks on child_resources_mutex_
//   5. Child thread can't exit → spin() can't join → deadlock
TEST_F(CallbackIsolatedAgnocastExecutorTest, cancel_from_child_callback_does_not_deadlock)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_node");
  std::atomic_bool callback_started{false};
  auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
    callback_started = true;
    // Keep this callback alive while external cancel triggers spin()'s shutdown path.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Call cancel() on the parent executor. If child_resources_mutex_ is held during
    // thread join in spin()'s shutdown, this blocks and causes a deadlock.
    executor->cancel();
  });
  executor->add_node(node);

  std::atomic_bool spin_finished{false};
  std::thread spin_thread([this, &spin_finished]() {
    executor->spin();
    spin_finished = true;
  });

  // Wait for the child callback to start executing.
  while (!callback_started) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Act: cancel externally while the child callback is still sleeping.
  executor->cancel();

  // Assert: spin() must exit within the timeout. A deadlock would hang forever.
  auto start = std::chrono::steady_clock::now();
  constexpr auto timeout = std::chrono::seconds(10);
  while (!spin_finished) {
    ASSERT_LT(std::chrono::steady_clock::now() - start, timeout)
      << "Deadlock detected: spin() did not exit after cancel() from child callback";
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  EXPECT_TRUE(spin_finished);
}
