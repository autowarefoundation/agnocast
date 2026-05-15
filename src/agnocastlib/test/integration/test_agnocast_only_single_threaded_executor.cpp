#include <agnocast/agnocast.hpp>
#include <agnocast/node/agnocast_only_executor.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

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
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  EXPECT_FALSE(executor.is_spinning());

  std::thread spinner([&]() { executor.spin(); });

  auto start = std::chrono::steady_clock::now();
  while (!executor.is_spinning() && (std::chrono::steady_clock::now() - start) < 10s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(executor.is_spinning());

  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);

  EXPECT_FALSE(executor.is_spinning());
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_once)
{
  agnocast::AgnocastOnlySingleThreadedExecutor executor;

  std::atomic<bool> timer_completed = false;
  auto timer = this->node->create_wall_timer(1ms, [&]() { timer_completed = true; });
  executor.add_node(this->node);

  executor.spin_once();

  auto start = std::chrono::steady_clock::now();
  while (!timer_completed && (std::chrono::steady_clock::now() - start) < 10s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(timer_completed);

  executor.cancel();
  executor.remove_node(this->node, true);
}

TEST_F(AgnocastOnlySingleThreadedExecutorTest, test_spin_until_future_complete)
{
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(this->node);

  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);

  auto start = std::chrono::steady_clock::now();
  auto shared_future = future.share();
  auto ret = executor.spin_until_future_complete(shared_future, 1s);
  executor.remove_node(this->node, true);

  EXPECT_GT(500ms, (std::chrono::steady_clock::now() - start));
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
}
