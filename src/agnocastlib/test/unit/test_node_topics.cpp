#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

class NodeTopicsExpandTest : public ::testing::Test
{
protected:
  void SetUp() override { GTEST_SKIP() << "Skipped until resolve_topic_name() is implemented"; }

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

TEST_F(NodeTopicsExpandTest, absolute_path_with_ns_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // Produces invalid "//", but matches rcl_expand_topic_name. rmw_validate_full_topic_name catches
  // this.
  EXPECT_EQ(node_topics->resolve_topic_name("/{ns}", true), "//my_ns");
}

TEST_F(NodeTopicsExpandTest, absolute_path_with_namespace_substitution)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // Same as above: produces invalid "//" but matches rcl_expand_topic_name behavior.
  // Validation should catch this in the full resolve_topic_name flow.
  EXPECT_EQ(node_topics->resolve_topic_name("/{namespace}", true), "//my_ns");
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

TEST_F(NodeTopicsExpandTest, root_namespace_ns_substitution)
{
  auto node_topics = create_node_topics("my_node", "/");
  EXPECT_EQ(node_topics->resolve_topic_name("{ns}", true), "/");
}

// =========================================
// Multiple substitutions tests
// =========================================

TEST_F(NodeTopicsExpandTest, multiple_substitutions)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_EQ(node_topics->resolve_topic_name("{ns}/{node}/topic", true), "/my_ns/my_node/topic");
}

TEST_F(NodeTopicsExpandTest, tilde_with_multiple_substitutions)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  // This also produces "//" in the middle, which is invalid but matches rcl_expand_topic_name.
  // rmw_validate_full_topic_name() should catch this.
  EXPECT_EQ(
    node_topics->resolve_topic_name("~/{ns}/{node}", true), "/my_ns/my_node//my_ns/my_node");
}

// =========================================
// Error cases
// =========================================

TEST_F(NodeTopicsExpandTest, empty_topic_name_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_THROW(node_topics->resolve_topic_name("", true), std::invalid_argument);
}

TEST_F(NodeTopicsExpandTest, unknown_substitution_throws)
{
  auto node_topics = create_node_topics("my_node", "/my_ns");
  EXPECT_THROW(node_topics->resolve_topic_name("{unknown}", true), std::invalid_argument);
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
