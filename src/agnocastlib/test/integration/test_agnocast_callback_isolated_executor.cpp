#include <agnocast/agnocast.hpp>
#include <agnocast/agnocast_callback_isolated_executor.hpp>

#include <cie_config_msgs/msg/callback_group_info.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class DummyNode : public rclcpp::Node
{
public:
  DummyNode() : Node("dummy_node")
  {
    rclcpp::CallbackGroup::SharedPtr callback_group_1 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::PublisherOptions pub_options_1;
    pub_options_1.callback_group = callback_group_1;
    ros2_pub_1_ = this->create_publisher<std_msgs::msg::Bool>("/test_topic_1", 10, pub_options_1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        ros2_pub_1_->publish(msg);
        published_ = true;
      },
      callback_group_1);

    rclcpp::CallbackGroup::SharedPtr callback_group_2 =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options_1;
    sub_options_1.callback_group = callback_group_2;
    ros2_sub_1_ = this->create_subscription<std_msgs::msg::Bool>(
      "/test_topic_1", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        (void)msg;
        subscribed_ = true;
      },
      sub_options_1);
  }

  bool is_published() const { return published_.load(); }
  bool is_subscribed() const { return subscribed_.load(); }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ros2_pub_1_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ros2_sub_1_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> published_{false};
  std::atomic<bool> subscribed_{false};
};

class CallbackGroupInfoReceiverNode : public rclcpp::Node
{
public:
  CallbackGroupInfoReceiverNode() : Node("callback_group_info_receiver", "/cie_thread_configurator")
  {
    subscription_ = this->create_subscription<cie_config_msgs::msg::CallbackGroupInfo>(
      "/cie_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all(),
      [this](const cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_messages_.push_back(*msg);
      });
  }

  std::vector<cie_config_msgs::msg::CallbackGroupInfo> get_received_messages()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_messages_;
  }

private:
  rclcpp::Subscription<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr subscription_;
  std::mutex mutex_;
  std::vector<cie_config_msgs::msg::CallbackGroupInfo> received_messages_;
};

class CallbackIsolatedExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(CallbackIsolatedExecutorTest, test_spin_publishes_callback_group_info)
{
  // Arrange
  auto receiver_node = std::make_shared<CallbackGroupInfoReceiverNode>();
  rclcpp::executors::SingleThreadedExecutor receiver_executor;
  receiver_executor.add_node(receiver_node);
  std::thread receiver_thread([&receiver_executor]() { receiver_executor.spin(); });

  auto test_node = std::make_shared<DummyNode>();
  auto callback_isolated_executor = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  callback_isolated_executor->add_node(test_node);

  // Act
  std::thread callback_isolated_thread(
    [&callback_isolated_executor]() { callback_isolated_executor->spin(); });

  while (test_node->is_published() == false || test_node->is_subscribed() == false) {
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
