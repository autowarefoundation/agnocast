#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_callback_isolated_executor.hpp"
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
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<ExecutorType>();
  }

  void TearDown() override { rclcpp::shutdown(); }

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

  std::shared_ptr<rclcpp::Node> create_test_node(const std::string & name = "test_node")
  {
    return std::make_shared<rclcpp::Node>(name);
  }

  rclcpp::CallbackGroup::SharedPtr create_cbg(
    const std::shared_ptr<rclcpp::Node> & node,
    rclcpp::CallbackGroupType type = rclcpp::CallbackGroupType::MutuallyExclusive)
  {
    return node->create_callback_group(type);
  }

  void add_node_to_executor(std::shared_ptr<rclcpp::Node> & node)
  {
    executor_->add_node(node->get_node_base_interface());
  }

  void add_callback_group_to_executor(
    const rclcpp::CallbackGroup::SharedPtr & cbg, const std::shared_ptr<rclcpp::Node> & node)
  {
    executor_->add_callback_group(cbg, node->get_node_base_interface());
  }

  agnocast::TimerBase::SharedPtr create_flag_timer(
    std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr cbg,
    std::atomic_bool & flag_to_set, std::chrono::milliseconds period = 50ms)
  {
    return agnocast::create_timer(
      node, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), period,
      [&flag_to_set]() { flag_to_set = true; }, cbg);
  }
};

using ExecutorTypes = ::testing::Types<
  agnocast::SingleThreadedAgnocastExecutor, agnocast::MultiThreadedAgnocastExecutor,
  // CallbackIsolatedAgnocastExecutor is commented out because this test fails
  // when the Agnocast callback is created after associating CallbackGroup
  // with the executor.
  // See issue #1263 for details:
  // https://github.com/autowarefoundation/agnocast/issues/1263
  // TODO(ruth561): Fix the issue and re-enable this executor in the test suite.
  // agnocast::CallbackIsolatedAgnocastExecutor,
  agnocast::AgnocastOnlySingleThreadedExecutor, agnocast::AgnocastOnlyMultiThreadedExecutor
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
