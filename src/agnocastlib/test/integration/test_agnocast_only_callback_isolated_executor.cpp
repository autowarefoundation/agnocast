#include <agnocast/agnocast.hpp>
#include <agnocast/node/agnocast_context.hpp>
#include <agnocast/node/agnocast_only_callback_isolated_executor.hpp>

#include <agnocast_cie_config_msgs/msg/callback_group_info.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class AgnocastOnlyDummyNode : public agnocast::Node
{
public:
  AgnocastOnlyDummyNode() : agnocast::Node("agnocast_only_dummy_node")
  {
    callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_2_;
};

class AgnocastOnlyCIEInfoReceiverNode : public rclcpp::Node
{
public:
  AgnocastOnlyCIEInfoReceiverNode()
  : Node("agnocast_only_cie_info_receiver", "/agnocast_cie_thread_configurator")
  {
    subscription_ = this->create_subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
      "/agnocast_cie_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all(),
      [this](const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_messages_.push_back(*msg);
      });
  }

  std::vector<agnocast_cie_config_msgs::msg::CallbackGroupInfo> get_received_messages()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_messages_;
  }

private:
  rclcpp::Subscription<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;
  std::mutex mutex_;
  std::vector<agnocast_cie_config_msgs::msg::CallbackGroupInfo> received_messages_;
};

class AgnocastOnlyCallbackIsolatedExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    agnocast::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
    // TODO(Koichi98): Call agnocast::shutdown() once available.
  }
};

TEST_F(AgnocastOnlyCallbackIsolatedExecutorTest, test_spin_publishes_callback_group_info)
{
  // Arrange
  auto receiver_node = std::make_shared<AgnocastOnlyCIEInfoReceiverNode>();
  rclcpp::executors::SingleThreadedExecutor receiver_executor;
  receiver_executor.add_node(receiver_node);
  std::thread receiver_thread([&receiver_executor]() { receiver_executor.spin(); });

  auto test_node = std::make_shared<AgnocastOnlyDummyNode>();
  auto callback_isolated_executor =
    std::make_shared<agnocast::AgnocastOnlyCallbackIsolatedExecutor>();
  callback_isolated_executor->add_node(test_node);

  // Act
  std::thread callback_isolated_thread(
    [&callback_isolated_executor]() { callback_isolated_executor->spin(); });

  auto start_time = std::chrono::steady_clock::now();
  constexpr auto timeout = std::chrono::seconds(10);
  while (receiver_node->get_received_messages().size() < 3u) {
    ASSERT_LT(std::chrono::steady_clock::now() - start_time, timeout)
      << "Timed out waiting for 3 callback group info messages";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  callback_isolated_executor->cancel();
  if (callback_isolated_thread.joinable()) {
    callback_isolated_thread.join();
  }
  receiver_executor.cancel();
  if (receiver_thread.joinable()) {
    receiver_thread.join();
  }

  // Assert
  ASSERT_EQ(receiver_node->get_received_messages().size(), 3u);  // 1 default + 2 created
}
