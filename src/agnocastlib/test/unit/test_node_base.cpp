#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/node_interfaces/node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <gtest/gtest.h>

class TestNodeBase : public ::testing::Test
{
protected:
  agnocast::Node::SharedPtr node_;

  rclcpp::NodeOptions node_options_without_parameter_services() const
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    return options;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr create_node_base(
    const std::string & node_name, const std::string & node_namespace)
  {
    node_ = std::make_shared<agnocast::Node>(
      node_name, node_namespace, node_options_without_parameter_services());
    return node_->get_node_base_interface();
  }

  // Down-cast to the concrete NodeBase to access methods that are not part of
  // rclcpp::NodeBaseInterface (e.g. set_on_callback_group_created, get_local_args).
  agnocast::node_interfaces::NodeBase::SharedPtr create_concrete_node_base(
    const std::string & node_name, const std::string & node_namespace)
  {
    auto base = create_node_base(node_name, node_namespace);
    return std::static_pointer_cast<agnocast::node_interfaces::NodeBase>(base);
  }
};

// =============================================================================
// Category 1: Node identity (name / namespace / fully qualified name)
//
// Specification:
//   - get_name() returns the node name verbatim.
//   - get_namespace() normalizes the namespace to always start with "/".
//   - get_fully_qualified_name() returns "<namespace>/<name>" (no double
//     slash even under the root namespace).
// =============================================================================

TEST_F(TestNodeBase, get_name_returns_constructed_name)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  const char * name = node_base->get_name();

  // Assert
  EXPECT_STREQ("my_node", name);
}

TEST_F(TestNodeBase, get_namespace_returns_namespace_with_leading_slash)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  const char * ns = node_base->get_namespace();

  // Assert
  EXPECT_STREQ("/my_ns", ns);
}

TEST_F(TestNodeBase, get_namespace_prepends_leading_slash_when_missing)
{
  // Arrange
  auto node_base = create_node_base("my_node", "my_ns");

  // Act
  const char * ns = node_base->get_namespace();

  // Assert
  EXPECT_STREQ("/my_ns", ns);
}

TEST_F(TestNodeBase, get_namespace_normalizes_empty_to_root)
{
  // Arrange
  auto node_base = create_node_base("my_node", "");

  // Act
  const char * ns = node_base->get_namespace();

  // Assert
  EXPECT_STREQ("/", ns);
}

TEST_F(TestNodeBase, get_namespace_keeps_root_as_is)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/");

  // Act
  const char * ns = node_base->get_namespace();

  // Assert
  EXPECT_STREQ("/", ns);
}

TEST_F(TestNodeBase, get_fully_qualified_name_concatenates_namespace_and_name)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  const char * fqn = node_base->get_fully_qualified_name();

  // Assert
  EXPECT_STREQ("/my_ns/my_node", fqn);
}

TEST_F(TestNodeBase, get_fully_qualified_name_avoids_double_slash_under_root_namespace)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/");

  // Act
  const char * fqn = node_base->get_fully_qualified_name();

  // Assert
  EXPECT_STREQ("/my_node", fqn);
}

TEST_F(TestNodeBase, get_fully_qualified_name_supports_nested_namespaces)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/ns1/ns2/ns3");

  // Act
  const char * fqn = node_base->get_fully_qualified_name();

  // Assert
  EXPECT_STREQ("/ns1/ns2/ns3/my_node", fqn);
}

TEST_F(TestNodeBase, constructor_applies_node_and_ns_remap_rules)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.arguments({"--ros-args", "-r", "__node:=remapped_node", "-r", "__ns:=/remapped_ns"});
  node_ = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node_->get_node_base_interface();

  // Act / Assert
  EXPECT_STREQ("remapped_node", node_base->get_name());
  EXPECT_STREQ("/remapped_ns", node_base->get_namespace());
  EXPECT_STREQ("/remapped_ns/remapped_node", node_base->get_fully_qualified_name());
}

TEST_F(TestNodeBase, constructor_throws_when_no_args_available_for_remap)
{
  // Arrange: with both local and global args NULL, rcl_remap_name returns
  // RCL_RET_INVALID_ARGUMENT. Use a local context to avoid touching global state.
  auto context = std::make_shared<rclcpp::Context>();

  // Act / Assert
  EXPECT_THROW(
    agnocast::node_interfaces::NodeBase(
      "my_node", "/my_ns", context, /*local_args=*/nullptr, /*use_global_arguments=*/false),
    std::runtime_error);
}

// =============================================================================
// Category 2: Context
//
// Specification:
//   - get_context() returns the context supplied via NodeOptions::context()
//     (non-null with default options). Until rclcpp::init() is called,
//     is_valid() returns false.
// =============================================================================

TEST_F(TestNodeBase, get_context_returns_non_null_with_default_node_options)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto context = node_base->get_context();

  // Assert
  EXPECT_NE(nullptr, context);
}

TEST_F(TestNodeBase, get_context_returns_invalid_context_without_rclcpp_init)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto context = node_base->get_context();

  // Assert
  ASSERT_NE(nullptr, context);
  EXPECT_FALSE(context->is_valid());
}

// =============================================================================
// Category 3: RCL node handle (unsupported by design)
//
// Specification:
//   - get_rcl_node_handle() and get_shared_rcl_node_handle() (both mutable
//     and const overloads) all throw std::runtime_error, because
//     agnocast::Node has no underlying rcl_node_t.
// =============================================================================

TEST_F(TestNodeBase, every_rcl_node_handle_accessor_throws_runtime_error)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");
  const auto * const_node_base = node_base.get();

  // Act / Assert
  EXPECT_THROW(node_base->get_rcl_node_handle(), std::runtime_error);
  EXPECT_THROW(const_node_base->get_rcl_node_handle(), std::runtime_error);
  EXPECT_THROW(node_base->get_shared_rcl_node_handle(), std::runtime_error);
  EXPECT_THROW(const_node_base->get_shared_rcl_node_handle(), std::runtime_error);
}

// =============================================================================
// Category 4: Callback group management
//
// Specification:
//   - get_default_callback_group() returns the MutuallyExclusive group every
//     node owns at construction.
//   - create_callback_group() returns a group of the requested type and
//     honors the automatically_add_to_executor_with_node flag.
//   - callback_group_in_node() reports membership for groups owned by the node.
//   - for_each_callback_group() iterates every living owned group, skipping
//     expired weak_ptrs.
// =============================================================================

TEST_F(TestNodeBase, get_default_callback_group_returns_mutually_exclusive_auto_add_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto default_group = node_base->get_default_callback_group();

  // Assert
  ASSERT_NE(nullptr, default_group);
  EXPECT_EQ(rclcpp::CallbackGroupType::MutuallyExclusive, default_group->type());
  EXPECT_TRUE(default_group->automatically_add_to_executor_with_node());
}

TEST_F(TestNodeBase, create_callback_group_returns_mutually_exclusive_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto group = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Assert
  ASSERT_NE(nullptr, group);
  EXPECT_EQ(rclcpp::CallbackGroupType::MutuallyExclusive, group->type());
}

TEST_F(TestNodeBase, create_callback_group_returns_reentrant_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto group = node_base->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Assert
  ASSERT_NE(nullptr, group);
  EXPECT_EQ(rclcpp::CallbackGroupType::Reentrant, group->type());
}

TEST_F(TestNodeBase, create_callback_group_propagates_auto_add_flag)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto on = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  auto off = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // Assert
  ASSERT_NE(nullptr, on);
  ASSERT_NE(nullptr, off);
  EXPECT_TRUE(on->automatically_add_to_executor_with_node());
  EXPECT_FALSE(off->automatically_add_to_executor_with_node());
}

TEST_F(TestNodeBase, create_callback_group_default_auto_add_is_true)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act: omit the second argument so the default applies.
  auto group = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Assert
  ASSERT_NE(nullptr, group);
  EXPECT_TRUE(group->automatically_add_to_executor_with_node());
}

TEST_F(TestNodeBase, callback_group_in_node_recognizes_default_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");
  auto default_group = node_base->get_default_callback_group();

  // Act
  bool is_member = node_base->callback_group_in_node(default_group);

  // Assert
  EXPECT_TRUE(is_member);
}

TEST_F(TestNodeBase, callback_group_in_node_recognizes_created_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");
  auto group = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Act
  bool is_member = node_base->callback_group_in_node(group);

  // Assert
  EXPECT_TRUE(is_member);
}

TEST_F(TestNodeBase, callback_group_in_node_rejects_group_owned_by_another_node)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");
  auto other_node = std::make_shared<agnocast::Node>(
    "other_node", "/other_ns", node_options_without_parameter_services());
  auto other_group = other_node->get_node_base_interface()->get_default_callback_group();

  // Act
  bool is_member = node_base->callback_group_in_node(other_group);

  // Assert
  EXPECT_FALSE(is_member);
}

TEST_F(TestNodeBase, for_each_callback_group_initially_visits_only_default_group)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  int count = 0;
  node_base->for_each_callback_group([&count](rclcpp::CallbackGroup::SharedPtr) { ++count; });

  // Assert
  EXPECT_EQ(1, count);
}

TEST_F(TestNodeBase, for_each_callback_group_visits_in_insertion_order)
{
  // Arrange: default group is inserted first; subsequent groups in creation order.
  auto node_base = create_node_base("my_node", "/my_ns");
  auto default_group = node_base->get_default_callback_group();
  auto group1 = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto group2 = node_base->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Act
  std::vector<rclcpp::CallbackGroup::SharedPtr> visited;
  node_base->for_each_callback_group(
    [&visited](rclcpp::CallbackGroup::SharedPtr g) { visited.push_back(g); });

  // Assert
  ASSERT_EQ(3u, visited.size());
  EXPECT_EQ(default_group, visited[0]);
  EXPECT_EQ(group1, visited[1]);
  EXPECT_EQ(group2, visited[2]);
}

TEST_F(TestNodeBase, for_each_callback_group_skips_expired_groups)
{
  // Arrange: create a group, let it go out of scope so its weak_ptr expires.
  auto node_base = create_node_base("my_node", "/my_ns");
  {
    auto transient_group =
      node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    int inner_count = 0;
    node_base->for_each_callback_group(
      [&inner_count](rclcpp::CallbackGroup::SharedPtr) { ++inner_count; });
    ASSERT_EQ(2, inner_count);  // default + transient
  }

  // Act
  int count = 0;
  node_base->for_each_callback_group([&count](rclcpp::CallbackGroup::SharedPtr) { ++count; });

  // Assert: only the still-living default group is visited.
  EXPECT_EQ(1, count);
}

// =============================================================================
// Category 5: Executor association flag
//
// Specification:
//   - get_associated_with_executor_atomic() exposes the atomic_bool used by
//     executors to claim a node; starts false and is freely mutable.
// =============================================================================

TEST_F(TestNodeBase, executor_association_flag_is_initially_false)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  bool initial = node_base->get_associated_with_executor_atomic().load();

  // Assert
  EXPECT_FALSE(initial);
}

TEST_F(TestNodeBase, executor_association_flag_is_mutable)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  node_base->get_associated_with_executor_atomic().store(true);

  // Assert
  EXPECT_TRUE(node_base->get_associated_with_executor_atomic().load());
}

// =============================================================================
// Category 6: Notify guard condition
//
// Specification:
//   - get_notify_guard_condition() returns a GuardCondition when the node was
//     constructed with a valid context (e.g., after rclcpp::init()); it
//     throws std::runtime_error otherwise.
//   - get_shared_notify_guard_condition() and trigger_notify_guard_condition()
//     (rclcpp 28+) are stubs that always throw std::runtime_error.
// =============================================================================

TEST_F(TestNodeBase, get_notify_guard_condition_throws_when_context_is_invalid)
{
  // Arrange: the default fixture creates the node without rclcpp::init(), so
  // the context exists but is not valid.
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act / Assert
  EXPECT_THROW(node_base->get_notify_guard_condition(), std::runtime_error);
}

TEST_F(TestNodeBase, get_notify_guard_condition_returns_guard_when_context_is_valid)
{
  // Arrange: use a local context to avoid coupling with the global default.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    auto options = node_options_without_parameter_services();
    options.context(context);
    auto local_node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
    auto node_base = local_node->get_node_base_interface();

    // Act / Assert
    EXPECT_NO_THROW({
      auto & guard = node_base->get_notify_guard_condition();
      (void)guard;
    });
  }
  context->shutdown("test cleanup");
}

#if RCLCPP_VERSION_MAJOR >= 28
TEST_F(TestNodeBase, get_shared_notify_guard_condition_throws_runtime_error)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act / Assert
  EXPECT_THROW(node_base->get_shared_notify_guard_condition(), std::runtime_error);
}

TEST_F(TestNodeBase, trigger_notify_guard_condition_throws_runtime_error)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act / Assert
  EXPECT_THROW(node_base->trigger_notify_guard_condition(), std::runtime_error);
}
#endif

// =============================================================================
// Category 7: NodeOptions-derived defaults (intra-process / topic statistics)
//
// Specification:
//   - get_use_intra_process_default() and get_enable_topic_statistics_default()
//     return the corresponding NodeOptions flags (false when unset).
// =============================================================================

TEST_F(TestNodeBase, get_use_intra_process_default_is_false_when_unset)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  bool value = node_base->get_use_intra_process_default();

  // Assert
  EXPECT_FALSE(value);
}

TEST_F(TestNodeBase, get_use_intra_process_default_reflects_node_options_true)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.use_intra_process_comms(true);
  auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node->get_node_base_interface();

  // Act
  bool value = node_base->get_use_intra_process_default();

  // Assert
  EXPECT_TRUE(value);
}

TEST_F(TestNodeBase, get_use_intra_process_default_reflects_node_options_false)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.use_intra_process_comms(false);
  auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node->get_node_base_interface();

  // Act
  bool value = node_base->get_use_intra_process_default();

  // Assert
  EXPECT_FALSE(value);
}

TEST_F(TestNodeBase, get_enable_topic_statistics_default_is_false_when_unset)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  bool value = node_base->get_enable_topic_statistics_default();

  // Assert
  EXPECT_FALSE(value);
}

TEST_F(TestNodeBase, get_enable_topic_statistics_default_reflects_node_options_true)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.enable_topic_statistics(true);
  auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node->get_node_base_interface();

  // Act
  bool value = node_base->get_enable_topic_statistics_default();

  // Assert
  EXPECT_TRUE(value);
}

TEST_F(TestNodeBase, get_enable_topic_statistics_default_reflects_node_options_false)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.enable_topic_statistics(false);
  auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node->get_node_base_interface();

  // Act
  bool value = node_base->get_enable_topic_statistics_default();

  // Assert
  EXPECT_FALSE(value);
}

// =============================================================================
// Category 8: Topic / service name resolution
//
// Specification:
//   - resolve_topic_or_service_name() expands names per ROS 2 rules: absolute
//     names pass through, relative names get the node namespace prepended,
//     "~" expands to "<namespace>/<node>", and "{node}" substitutes the node
//     name. Empty input and unknown "{...}" substitutions throw
//     std::runtime_error.
// =============================================================================

TEST_F(TestNodeBase, resolve_topic_name_keeps_absolute_name_unchanged)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("/absolute_topic", false, true);

  // Assert
  EXPECT_EQ("/absolute_topic", resolved);
}

TEST_F(TestNodeBase, resolve_topic_name_prefixes_relative_name_with_namespace)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("relative_topic", false, true);

  // Assert
  EXPECT_EQ("/my_ns/relative_topic", resolved);
}

TEST_F(TestNodeBase, resolve_topic_name_expands_tilde_to_namespace_and_node_name)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("~/private_topic", false, true);

  // Assert
  EXPECT_EQ("/my_ns/my_node/private_topic", resolved);
}

TEST_F(TestNodeBase, resolve_topic_name_substitutes_node_token)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("{node}/topic", false, true);

  // Assert
  EXPECT_EQ("/my_ns/my_node/topic", resolved);
}

TEST_F(TestNodeBase, resolve_service_name_keeps_absolute_name_unchanged)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("/absolute_service", true, true);

  // Assert
  EXPECT_EQ("/absolute_service", resolved);
}

TEST_F(TestNodeBase, resolve_service_name_prefixes_relative_name_with_namespace)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("relative_service", true, true);

  // Assert
  EXPECT_EQ("/my_ns/relative_service", resolved);
}

TEST_F(TestNodeBase, resolve_topic_name_under_root_namespace_yields_single_slash)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/");

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("topic", false, true);

  // Assert
  EXPECT_EQ("/topic", resolved);
}

TEST_F(TestNodeBase, resolve_topic_name_throws_for_empty_input)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act / Assert
  EXPECT_THROW(node_base->resolve_topic_or_service_name("", false, true), std::runtime_error);
}

TEST_F(TestNodeBase, resolve_topic_name_throws_for_unknown_substitution)
{
  // Arrange
  auto node_base = create_node_base("my_node", "/my_ns");

  // Act / Assert
  EXPECT_THROW(
    node_base->resolve_topic_or_service_name("{unknown}", false, true), std::runtime_error);
}

TEST_F(TestNodeBase, resolve_topic_name_applies_remap_rule_when_only_expand_false)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.arguments({"--ros-args", "-r", "/foo:=/bar"});
  node_ = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node_->get_node_base_interface();

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("/foo", false, false);

  // Assert
  EXPECT_EQ("/bar", resolved);
}

TEST_F(TestNodeBase, resolve_service_name_applies_remap_rule_when_only_expand_false)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.arguments({"--ros-args", "-r", "/foo_srv:=/bar_srv"});
  node_ = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base = node_->get_node_base_interface();

  // Act
  auto resolved = node_base->resolve_topic_or_service_name("/foo_srv", true, false);

  // Assert
  EXPECT_EQ("/bar_srv", resolved);
}

// =============================================================================
// Category 9: on_callback_group_created hook
//
// Specification:
//   - set_on_callback_group_created() registers a callback that create_callback_group()
//     fires AFTER pushing the new group into the node's internal list.
//   - Setting a new callback replaces any previous one (single-slot semantics).
// =============================================================================

TEST_F(TestNodeBase, on_callback_group_created_fires_for_each_created_group)
{
  // Arrange
  auto node_base = create_concrete_node_base("my_node", "/my_ns");
  int call_count = 0;
  node_base->set_on_callback_group_created([&call_count]() { ++call_count; });

  // Act
  node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node_base->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Assert
  EXPECT_EQ(2, call_count);
}

TEST_F(TestNodeBase, on_callback_group_created_replaces_previous_callback)
{
  // Arrange
  auto node_base = create_concrete_node_base("my_node", "/my_ns");
  int first_count = 0;
  int second_count = 0;
  node_base->set_on_callback_group_created([&first_count]() { ++first_count; });

  // Act: replace before the first create call so only the second hook ever runs.
  node_base->set_on_callback_group_created([&second_count]() { ++second_count; });
  node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Assert: registration is single-slot — the prior hook does not fire.
  EXPECT_EQ(0, first_count);
  EXPECT_EQ(1, second_count);
}

TEST_F(TestNodeBase, on_callback_group_created_fires_after_group_is_registered)
{
  // Arrange: when the hook fires, the new group must already be discoverable
  // via callback_group_in_node() so the executor can attach to it immediately.
  auto node_base = create_concrete_node_base("my_node", "/my_ns");

  rclcpp::CallbackGroup::SharedPtr captured_last_group;
  bool last_group_known_to_node = false;

  // Act: capture the most recent group inside the hook and check membership.
  // Because the hook does not receive the group as an argument, we observe it
  // through for_each_callback_group instead, taking the last visited group.
  node_base->set_on_callback_group_created([&]() {
    rclcpp::CallbackGroup::SharedPtr last;
    node_base->for_each_callback_group([&last](rclcpp::CallbackGroup::SharedPtr g) { last = g; });
    captured_last_group = last;
    last_group_known_to_node = node_base->callback_group_in_node(last);
  });
  auto created = node_base->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Assert
  EXPECT_EQ(created, captured_last_group);
  EXPECT_TRUE(last_group_known_to_node);
}

// =============================================================================
// Category 10: Constructor-argument accessors (get_local_args / get_global_args)
//
// Specification:
//   - get_local_args() returns the rcl_arguments_t parsed from
//     NodeOptions::arguments() (non-null even with no arguments).
//   - get_global_args() returns nullptr when NodeOptions::use_global_arguments
//     is false, or when it is true but agnocast::init() has not been called.
//     Only when use_global_arguments is true AND agnocast::init() has been
//     called does it return a non-null pointer reflecting the global agnocast
//     context.
// =============================================================================

TEST_F(TestNodeBase, get_local_args_is_non_null_for_default_options)
{
  // Arrange
  auto node_base = create_concrete_node_base("my_node", "/my_ns");

  // Act
  const rcl_arguments_t * local_args = node_base->get_local_args();

  // Assert: parse_arguments always produces a valid object, so the pointer is
  // non-null even when no arguments were supplied.
  EXPECT_NE(nullptr, local_args);
}

TEST_F(TestNodeBase, get_global_args_is_null_when_use_global_arguments_is_false)
{
  // Arrange
  auto options = node_options_without_parameter_services();
  options.use_global_arguments(false);
  auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
  auto node_base =
    std::static_pointer_cast<agnocast::node_interfaces::NodeBase>(node->get_node_base_interface());

  // Act
  const rcl_arguments_t * global_args = node_base->get_global_args();

  // Assert
  EXPECT_EQ(nullptr, global_args);
}

TEST_F(TestNodeBase, get_global_args_is_non_null_when_g_context_is_initialized)
{
  // Arrange: agnocast::init() populates g_context with parsed arguments,
  // which the constructor reads when use_global_arguments=true (default).
  agnocast::init(0, nullptr);
  {
    auto options = node_options_without_parameter_services();
    auto node = std::make_shared<agnocast::Node>("my_node", "/my_ns", options);
    auto node_base = std::static_pointer_cast<agnocast::node_interfaces::NodeBase>(
      node->get_node_base_interface());

    // Act
    const rcl_arguments_t * global_args = node_base->get_global_args();

    // Assert
    EXPECT_NE(nullptr, global_args);
  }
  agnocast::shutdown();
}
