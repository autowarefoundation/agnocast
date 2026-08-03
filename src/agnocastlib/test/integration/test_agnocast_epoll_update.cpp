#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_callback_isolated_executor.hpp"
#include "agnocast/node/agnocast_only_executor.hpp"
#include "agnocast/node/agnocast_only_multi_threaded_executor.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

#include <rclcpp/node.hpp>

#include <agnocast_cie_config_msgs/msg/callback_group_info.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

template <typename ExecutorType>
class EpollUpdateTest : public ::testing::Test
{
  std::shared_ptr<ExecutorType> executor_;
  std::thread spin_thread_;

protected:
  void SetUp() override
  {
    if constexpr (std::is_base_of_v<agnocast::AgnocastOnlyExecutor, ExecutorType>) {
      agnocast::init(0, nullptr);
    } else {
      rclcpp::init(0, nullptr);
    }
    executor_ = std::make_shared<ExecutorType>();
  }

  void TearDown() override
  {
    if constexpr (std::is_base_of_v<agnocast::AgnocastOnlyExecutor, ExecutorType>) {
      agnocast::shutdown();
    } else {
      rclcpp::shutdown();
    }
  }

  void start_spinning()
  {
    if (!spin_thread_.joinable()) {
      spin_thread_ = std::thread([this]() { this->executor_->spin(); });
    }
  }

  void stop_spinning()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  template <typename Predicate>
  bool wait_for_condition(Predicate condition, std::chrono::milliseconds timeout = 1000ms)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  auto create_test_node(const std::string & name = "test_node")
  {
    if constexpr (std::is_base_of_v<agnocast::AgnocastOnlyExecutor, ExecutorType>) {
      return std::make_shared<agnocast::Node>(name);
    } else {
      return std::make_shared<rclcpp::Node>(name);
    }
  }

  template <typename NodeT>
  rclcpp::CallbackGroup::SharedPtr create_cbg(
    const std::shared_ptr<NodeT> & node,
    rclcpp::CallbackGroupType type = rclcpp::CallbackGroupType::MutuallyExclusive)
  {
    return node->create_callback_group(type);
  }

  template <typename NodeT>
  void add_node_to_executor(const std::shared_ptr<NodeT> & node)
  {
    executor_->add_node(node->get_node_base_interface());
  }

  template <typename NodeT>
  void add_callback_group_to_executor(
    const rclcpp::CallbackGroup::SharedPtr & cbg, const std::shared_ptr<NodeT> & node)
  {
    executor_->add_callback_group(cbg, node->get_node_base_interface());
  }

  template <typename NodeT>
  agnocast::TimerBase::SharedPtr create_flag_timer(
    const std::shared_ptr<NodeT> & node, const rclcpp::CallbackGroup::SharedPtr & cbg,
    std::atomic_bool & flag_to_set, std::chrono::milliseconds period = 50ms)
  {
    return agnocast::create_timer(
      node, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), period,
      [&flag_to_set]() { flag_to_set = true; }, cbg);
  }

  static constexpr bool is_callback_isolated_executor()
  {
    return std::is_same_v<ExecutorType, agnocast::CallbackIsolatedAgnocastExecutor> ||
           std::is_same_v<ExecutorType, agnocast::AgnocastOnlyCallbackIsolatedExecutor>;
  }

  // The callback-isolated executors pick a child executor per callback group from a snapshot of
  // the group's agnocast entities taken when the group is first spawned, and never revisit it.
  // Tests that create the timer after the group has been spawned therefore cannot pass on these
  // executors yet. See issue #1263.
  //
  // NOTE: This only reports whether the test should be skipped. GTEST_SKIP() must be invoked
  // directly in the test body, because it expands to a `return` that only exits the immediately
  // enclosing function; calling it from a helper would skip nothing and let the test body run on.
  static bool should_skip_entity_added_after_spawn() { return is_callback_isolated_executor(); }
};

#define SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN                                                  \
  "callback-isolated executor classifies a group before the later-created timer exists; see " \
  "issue #1263"

using ExecutorTypes = ::testing::Types<
  agnocast::SingleThreadedAgnocastExecutor, agnocast::MultiThreadedAgnocastExecutor,
  agnocast::CallbackIsolatedAgnocastExecutor, agnocast::AgnocastOnlySingleThreadedExecutor,
  agnocast::AgnocastOnlyMultiThreadedExecutor
  // AgnocastOnlyCallbackIsolatedExecutor is commented out because it
  // unexpectedly terminates when adding CallbackGroup via add_callback_group.
  // TODO(ruth561): Fix the issue and re-enable this executor in the test suite.
  // agnocast::AgnocastOnlyCallbackIsolatedExecutor
  >;

TYPED_TEST_SUITE(EpollUpdateTest, ExecutorTypes);

// Exhaustively test execution order permutations.
//
// Constraints:
// 1. The execution order of `Cbg` -> `Timer` is fixed.
// 2. Processes prior to `Spin` do not need to be altered.
//
// Glossary:
// Cbg: Create callback group
// Timer: Create timer
// Add: Add node to executor
// Addcbg: Add callback group to executor
// Spin: Start executor spin in a dedicated thread

// Spin at the end.
TYPED_TEST(EpollUpdateTest, CbgTimerAddSpin)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  this->add_node_to_executor(node);
  this->start_spinning();

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

// Spin -> Add.
TYPED_TEST(EpollUpdateTest, CbgTimerSpinAdd)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  this->add_node_to_executor(node);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, CbgSpinAddTimer)
{
  if (this->should_skip_entity_added_after_spawn()) {
    GTEST_SKIP() << SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN;
  }
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  this->add_node_to_executor(node);
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, SpinAddCbgTimer)
{
  if (this->should_skip_entity_added_after_spawn()) {
    GTEST_SKIP() << SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN;
  }
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  this->add_node_to_executor(node);
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

// Spin -> Timer.
TYPED_TEST(EpollUpdateTest, CbgAddSpinTimer)
{
  if (this->should_skip_entity_added_after_spawn()) {
    GTEST_SKIP() << SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN;
  }
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  this->add_node_to_executor(node);
  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, CbgSpinTimerAdd)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  std::this_thread::sleep_for(250ms);
  this->add_node_to_executor(node);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

// Spin -> Cbg.
TYPED_TEST(EpollUpdateTest, AddSpinCbgTimer)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->add_node_to_executor(node);
  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, SpinCbgAddTimer)
{
  if (this->should_skip_entity_added_after_spawn()) {
    GTEST_SKIP() << SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN;
  }
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  this->add_node_to_executor(node);
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, SpinCbgTimerAdd)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  std::this_thread::sleep_for(250ms);
  this->add_node_to_executor(node);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

// Tests using add_callback_group instead of add_node.
// Exhaustive execution-order tests are omitted here because they are already
// covered by the add_node tests. Instead, we extract a few selected test cases
// to run similar tests using add_callback_group.

TYPED_TEST(EpollUpdateTest, CbgTimerAddcbgSpin)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  this->add_callback_group_to_executor(cbg, node);
  this->start_spinning();

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, SpinCbgTimerAddcbg)
{
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);
  std::this_thread::sleep_for(250ms);
  this->add_callback_group_to_executor(cbg, node);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}

TYPED_TEST(EpollUpdateTest, SpinCbgAddcbgTimer)
{
  if (this->should_skip_entity_added_after_spawn()) {
    GTEST_SKIP() << SKIP_REASON_ENTITY_ADDED_AFTER_SPAWN;
  }
  std::atomic_bool callback_started{false};
  auto node = this->create_test_node();

  this->start_spinning();
  std::this_thread::sleep_for(250ms);
  auto cbg = this->create_cbg(node);
  this->add_callback_group_to_executor(cbg, node);
  std::this_thread::sleep_for(250ms);
  [[maybe_unused]] auto timer = this->create_flag_timer(node, cbg, callback_started);

  bool success = this->wait_for_condition([&]() { return callback_started.load(); });
  EXPECT_TRUE(success) << "Timer callback was not called, epoll update might have failed";

  this->stop_spinning();
}
