#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

using StringMsg = std_msgs::msg::String;

// Subscriber counts as seen by a publisher. The counts must agree with what rclcpp reports for the
// same node and endpoint configuration, so each expectation below is the rclcpp one.
class PublisherCountIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override { agnocast::init(0, nullptr); }

  void TearDown() override
  {
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
  }

  static std::shared_ptr<agnocast::Node> make_node(const std::string & name, bool use_intra_process)
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(use_intra_process);
    return std::make_shared<agnocast::Node>(name, options);
  }
};

TEST_F(PublisherCountIntegrationTest, counts_are_zero_without_a_subscriber)
{
  auto node = make_node("test_pub_count_no_sub", true);
  auto pub = node->create_publisher<StringMsg>("/test_pub_count_no_sub", 1);

  EXPECT_EQ(pub->get_subscription_count(), 0u);
  EXPECT_EQ(pub->get_intra_process_subscription_count(), 0u);
}

TEST_F(PublisherCountIntegrationTest, same_process_subscriber_is_counted)
{
  auto node = make_node("test_pub_count_same_process", false);
  const std::string topic = "/test_pub_count_same_process";
  auto pub = node->create_publisher<StringMsg>(topic, 1);
  auto sub = node->create_subscription<StringMsg>(
    topic, 1, [](const agnocast::ipc_shared_ptr<StringMsg> &) {});

  EXPECT_EQ(pub->get_subscription_count(), 1u);
  EXPECT_EQ(pub->get_intra_process_subscription_count(), 0u);
}

TEST_F(PublisherCountIntegrationTest, intra_process_count_follows_use_intra_process_comms)
{
  auto node = make_node("test_pub_count_intra_process", true);
  const std::string topic = "/test_pub_count_intra_process";
  auto pub = node->create_publisher<StringMsg>(topic, 1);
  auto sub = node->create_subscription<StringMsg>(
    topic, 1, [](const agnocast::ipc_shared_ptr<StringMsg> &) {});

  EXPECT_EQ(pub->get_subscription_count(), 1u);
  EXPECT_EQ(pub->get_intra_process_subscription_count(), 1u);
}

TEST_F(PublisherCountIntegrationTest, counts_drop_when_the_subscriber_is_destroyed)
{
  auto node = make_node("test_pub_count_sub_destroyed", true);
  const std::string topic = "/test_pub_count_sub_destroyed";
  auto pub = node->create_publisher<StringMsg>(topic, 1);
  {
    auto sub = node->create_subscription<StringMsg>(
      topic, 1, [](const agnocast::ipc_shared_ptr<StringMsg> &) {});
    ASSERT_EQ(pub->get_subscription_count(), 1u);
    ASSERT_EQ(pub->get_intra_process_subscription_count(), 1u);
  }

  EXPECT_EQ(pub->get_subscription_count(), 0u);
  EXPECT_EQ(pub->get_intra_process_subscription_count(), 0u);
}
