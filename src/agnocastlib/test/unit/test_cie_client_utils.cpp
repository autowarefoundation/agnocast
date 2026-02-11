#include "agnocast/cie_client_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

class CreateCallbackGroupIdTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(CreateCallbackGroupIdTest, EmptyGroup)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics;

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node");
}

TEST_F(CreateCallbackGroupIdTest, SubscriptionOnly)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;
  auto sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic_a", 10, [](const std::shared_ptr<const std_msgs::msg::Bool>) {}, sub_options);

  std::vector<std::string> agnocast_topics;

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node@Subscription(/topic_a)");
}

TEST_F(CreateCallbackGroupIdTest, TimerOnly)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto timer = node_->create_wall_timer(std::chrono::seconds(1), []() {}, group);

  std::vector<std::string> agnocast_topics;

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node@Timer(1000000000)");
}

TEST_F(CreateCallbackGroupIdTest, MultipleEntriesSorted)
{
  // Arrange: create subscriptions in reverse alphabetical order
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;
  auto sub_b = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic_b", 10, [](const std::shared_ptr<const std_msgs::msg::Bool>) {}, sub_options);
  auto sub_a = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic_a", 10, [](const std::shared_ptr<const std_msgs::msg::Bool>) {}, sub_options);
  auto timer = node_->create_wall_timer(std::chrono::seconds(1), []() {}, group);

  std::vector<std::string> agnocast_topics;

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert: entries should be sorted alphabetically
  EXPECT_EQ(id, "/test_node@Subscription(/topic_a)@Subscription(/topic_b)@Timer(1000000000)");
}

TEST_F(CreateCallbackGroupIdTest, AgnocastSubscription)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics = {"/topic_a"};

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node@Subscription(/topic_a)");
}

TEST_F(CreateCallbackGroupIdTest, AgnocastService)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics = {"/AGNOCAST_SRV_REQUEST/my_service"};

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node@Service(/my_service)");
}

TEST_F(CreateCallbackGroupIdTest, AgnocastClient)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics = {"/AGNOCAST_SRV_RESPONSE/my_service_SEP_client_node"};

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node@Client(/my_service)");
}

TEST_F(CreateCallbackGroupIdTest, AgnocastAndRclcppProduceSameOutput)
{
  // Arrange: non-Agnocast case with rclcpp subscription
  auto group_rclcpp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group_rclcpp;
  auto sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic_a", 10, [](const std::shared_ptr<const std_msgs::msg::Bool>) {}, sub_options);
  std::vector<std::string> empty_agnocast_topics;

  // Arrange: Agnocast case with agnocast topic instead of rclcpp subscription
  auto group_agnocast = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics = {"/topic_a"};

  // Act
  auto id_rclcpp = agnocast::create_callback_group_id(group_rclcpp, node_, empty_agnocast_topics);
  auto id_agnocast = agnocast::create_callback_group_id(group_agnocast, node_, agnocast_topics);

  // Assert
  EXPECT_EQ(id_rclcpp, id_agnocast);
}

TEST_F(CreateCallbackGroupIdTest, MixedAgnocastAndRclcppSorted)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group;
  auto sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/topic_b", 10, [](const std::shared_ptr<const std_msgs::msg::Bool>) {}, sub_options);
  std::vector<std::string> agnocast_topics = {"/topic_a"};

  // Act
  auto id = agnocast::create_callback_group_id(group, node_, agnocast_topics);

  // Assert: both labeled "Subscription" and sorted alphabetically
  EXPECT_EQ(id, "/test_node@Subscription(/topic_a)@Subscription(/topic_b)");
}

TEST_F(CreateCallbackGroupIdTest, NodeBaseInterfaceOverload)
{
  // Arrange
  auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  std::vector<std::string> agnocast_topics;

  // Act: Call the NodeBaseInterface overload directly
  auto id =
    agnocast::create_callback_group_id(group, node_->get_node_base_interface(), agnocast_topics);

  // Assert
  EXPECT_EQ(id, "/test_node");
}
