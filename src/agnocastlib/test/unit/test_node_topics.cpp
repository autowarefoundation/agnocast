#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

class NodeTopicsExpandTest : public ::testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}

  agnocast::Node::SharedPtr node_;

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr create_node_topics(
    const std::string & node_name, const std::string & node_namespace)
  {
    node_ = std::make_shared<agnocast::Node>(node_name, node_namespace);
    return node_->get_node_topics_interface();
  }
};

// =========================================
// Absolute path tests
// =========================================

TEST_F(NodeTopicsExpandTest, absolute_path_no_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("/chatter", true), "/chatter");
}

TEST_F(NodeTopicsExpandTest, absolute_path_with_node_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("/{node}", true), "/my_node");
}

TEST_F(NodeTopicsExpandTest, absolute_path_with_ns_substitution_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // Produces invalid "//", which is rejected by rmw_validate_full_topic_name
  EXPECT_THROW(node_topics->resolve_topic_name("/{ns}", true), std::runtime_error);
}

TEST_F(NodeTopicsExpandTest, absolute_path_with_namespace_substitution_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // Produces invalid "//", which is rejected by rmw_validate_full_topic_name
  EXPECT_THROW(node_topics->resolve_topic_name("/{namespace}", true), std::runtime_error);
}

// =========================================
// Relative path tests
// =========================================

TEST_F(NodeTopicsExpandTest, relative_path_simple)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("chatter", true), "/my_ns/chatter");
}

TEST_F(NodeTopicsExpandTest, relative_path_with_node_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("{node}/chatter", true), "/my_ns/my_node/chatter");
}

TEST_F(NodeTopicsExpandTest, relative_path_node_only)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("{node}", true), "/my_ns/my_node");
}

TEST_F(NodeTopicsExpandTest, relative_path_ns_only)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("{ns}", true), "/my_ns");
}

// =========================================
// Tilde expansion tests (private topics)
// =========================================

TEST_F(NodeTopicsExpandTest, tilde_only)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("~", true), "/my_ns/my_node");
}

TEST_F(NodeTopicsExpandTest, tilde_with_topic)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("~/ping", true), "/my_ns/my_node/ping");
}

TEST_F(NodeTopicsExpandTest, tilde_with_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("~/{node}", true), "/my_ns/my_node/my_node");
}

// =========================================
// Root namespace tests (namespace = "/")
// =========================================

TEST_F(NodeTopicsExpandTest, root_namespace_relative_path)
{
  auto node_topics = create_node_topics("my_node", "/");
  EXPECT_EQ(node_topics->resolve_topic_name("ping", true), "/ping");
}

TEST_F(NodeTopicsExpandTest, root_namespace_tilde_only)
{
  auto node_topics = create_node_topics("my_node", "/");
  EXPECT_EQ(node_topics->resolve_topic_name("~", true), "/my_node");
}

TEST_F(NodeTopicsExpandTest, root_namespace_tilde_with_topic)
{
  auto node_topics = create_node_topics("my_node", "/");
  EXPECT_EQ(node_topics->resolve_topic_name("~/ping", true), "/my_node/ping");
}

TEST_F(NodeTopicsExpandTest, root_namespace_node_substitution)
{
  auto node_topics = create_node_topics("my_node", "/");
  EXPECT_EQ(node_topics->resolve_topic_name("{node}", true), "/my_node");
}

TEST_F(NodeTopicsExpandTest, root_namespace_ns_substitution_throws)
{
  auto node_topics = create_node_topics("my_node", "/");
  // Produces "/" which ends with '/', rejected by rmw_validate_full_topic_name
  EXPECT_THROW(node_topics->resolve_topic_name("{ns}", true), std::runtime_error);
}

// =========================================
// Multiple substitutions tests
// =========================================

TEST_F(NodeTopicsExpandTest, multiple_substitutions)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("{ns}/{node}/topic", true), "/my_ns/my_node/topic");
}

TEST_F(NodeTopicsExpandTest, tilde_with_multiple_substitutions_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // Produces "//" in the middle, which is rejected by rmw_validate_full_topic_name
  EXPECT_THROW(node_topics->resolve_topic_name("~/{ns}/{node}", true), std::runtime_error);
}

// =========================================
// Error cases
// =========================================

TEST_F(NodeTopicsExpandTest, empty_topic_name_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_THROW(node_topics->resolve_topic_name("", true), std::runtime_error);
}

TEST_F(NodeTopicsExpandTest, unknown_substitution_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_THROW(node_topics->resolve_topic_name("{unknown}", true), std::runtime_error);
}

// =========================================
// Nested namespace tests
// =========================================

TEST_F(NodeTopicsExpandTest, nested_namespace_relative)
{
  auto node_topics = create_node_topics("my_node", "/ns1/ns2");
  EXPECT_EQ(node_topics->resolve_topic_name("topic", true), "/ns1/ns2/topic");
}

TEST_F(NodeTopicsExpandTest, nested_namespace_tilde)
{
  auto node_topics = create_node_topics("my_node", "/ns1/ns2");
  EXPECT_EQ(node_topics->resolve_topic_name("~/topic", true), "/ns1/ns2/my_node/topic");
}

TEST_F(NodeTopicsExpandTest, nested_namespace_substitution)
{
  auto node_topics = create_node_topics("my_node", "/ns1/ns2");
  EXPECT_EQ(node_topics->resolve_topic_name("{ns}/topic", true), "/ns1/ns2/topic");
}
