#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/node_interfaces/node_graph.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

using StringMsg = std_msgs::msg::String;

class NodeGraphIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_node_graph");
    graph_ = node_->get_node_graph_interface();
  }

  void TearDown() override
  {
    graph_.reset();
    node_.reset();
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
  }

  std::shared_ptr<agnocast::Node> node_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
};

TEST_F(NodeGraphIntegrationTest, get_node_graph_interface_returns_node_graph)
{
  ASSERT_NE(graph_, nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<agnocast::node_interfaces::NodeGraph>(graph_), nullptr);
}

TEST_F(NodeGraphIntegrationTest, count_publishers_is_zero_for_unknown_topic)
{
  EXPECT_EQ(graph_->count_publishers("/test_node_graph_no_such_topic"), 0u);
}

TEST_F(NodeGraphIntegrationTest, count_subscribers_is_zero_for_unknown_topic)
{
  EXPECT_EQ(graph_->count_subscribers("/test_node_graph_no_such_topic"), 0u);
}

TEST_F(NodeGraphIntegrationTest, count_publishers_counts_a_created_publisher)
{
  const std::string topic = "/test_node_graph_count_pub";
  EXPECT_EQ(graph_->count_publishers(topic), 0u);

  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  EXPECT_EQ(graph_->count_publishers(topic), 1u);
}

TEST_F(NodeGraphIntegrationTest, count_publishers_counts_every_publisher_on_the_topic)
{
  const std::string topic = "/test_node_graph_count_two_pubs";

  auto pub1 = node_->create_publisher<StringMsg>(topic, 1);
  auto pub2 = node_->create_publisher<StringMsg>(topic, 1);
  EXPECT_EQ(graph_->count_publishers(topic), 2u);
}

TEST_F(NodeGraphIntegrationTest, count_publishers_drops_when_the_publisher_is_destroyed)
{
  const std::string topic = "/test_node_graph_pub_destroyed";
  {
    auto pub = node_->create_publisher<StringMsg>(topic, 1);
    ASSERT_EQ(graph_->count_publishers(topic), 1u);
  }
  EXPECT_EQ(graph_->count_publishers(topic), 0u);
}

// count_subscribers() reports ret_other_process_subscriber_num, so a subscriber living in the
// caller's own process is not counted. This is the documented limitation, not an accident: the
// underlying helper exists to tell a publisher how many peers receive a published message.
TEST_F(NodeGraphIntegrationTest, count_subscribers_does_not_see_same_process_subscriber)
{
  const std::string topic = "/test_node_graph_count_sub";

  auto sub = node_->create_subscription<StringMsg>(
    topic, 1, [](const agnocast::ipc_shared_ptr<StringMsg> &) {});
  EXPECT_EQ(graph_->count_subscribers(topic), 0u);
}

TEST_F(NodeGraphIntegrationTest, count_publishers_resolves_a_relative_topic_name)
{
  auto node = std::make_shared<agnocast::Node>("test_node_graph_rel", "/test_ns");
  auto graph = node->get_node_graph_interface();

  auto pub = node->create_publisher<StringMsg>("relative_topic", 1);

  EXPECT_EQ(graph->count_publishers("relative_topic"), 1u);
  EXPECT_EQ(graph->count_publishers("/test_ns/relative_topic"), 1u);
  EXPECT_EQ(graph->count_publishers("/relative_topic"), 0u);
}

TEST_F(NodeGraphIntegrationTest, node_delegates_counting_to_the_node_graph_interface)
{
  const std::string topic = "/test_node_graph_delegation";
  auto pub = node_->create_publisher<StringMsg>(topic, 1);
  EXPECT_EQ(node_->count_publishers(topic), 1u);
}

// agnocast has no graph events, but rclcpp calls these unconditionally, so they must not throw.
TEST_F(NodeGraphIntegrationTest, notify_graph_change_and_notify_shutdown_are_no_ops)
{
  EXPECT_NO_THROW(graph_->notify_graph_change());
  EXPECT_NO_THROW(graph_->notify_shutdown());
}

TEST_F(NodeGraphIntegrationTest, unsupported_methods_throw_runtime_error)
{
  EXPECT_THROW(graph_->get_node_names(), std::runtime_error);
  EXPECT_THROW(graph_->get_topic_names_and_types(), std::runtime_error);
  EXPECT_THROW(graph_->get_service_names_and_types(), std::runtime_error);
  EXPECT_THROW(graph_->get_service_names_and_types_by_node("n", "/"), std::runtime_error);
  EXPECT_THROW(graph_->get_client_names_and_types_by_node("n", "/"), std::runtime_error);
  EXPECT_THROW(graph_->get_publisher_names_and_types_by_node("n", "/"), std::runtime_error);
  EXPECT_THROW(graph_->get_subscriber_names_and_types_by_node("n", "/"), std::runtime_error);
  EXPECT_THROW(graph_->get_node_names_with_enclaves(), std::runtime_error);
  EXPECT_THROW(graph_->get_node_names_and_namespaces(), std::runtime_error);
  EXPECT_THROW(graph_->get_graph_guard_condition(), std::runtime_error);
  EXPECT_THROW(graph_->get_graph_event(), std::runtime_error);
  EXPECT_THROW(
    graph_->wait_for_graph_change(nullptr, std::chrono::nanoseconds(0)), std::runtime_error);
  EXPECT_THROW(graph_->count_graph_users(), std::runtime_error);
  EXPECT_THROW(graph_->get_publishers_info_by_topic("/t"), std::runtime_error);
  EXPECT_THROW(graph_->get_subscriptions_info_by_topic("/t"), std::runtime_error);
}
