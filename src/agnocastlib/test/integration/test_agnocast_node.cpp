#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace
{
void reset_context_for_test()
{
  std::lock_guard<std::mutex> lock(agnocast::g_context_mtx);
  agnocast::g_context = agnocast::Context{};
}

// Initialize the agnocast context with a process-global `use_sim_time:=true` override.
void init_with_global_use_sim_time()
{
  const char * argv[] = {"test_agnocast_node", "--ros-args", "-p", "use_sim_time:=true"};
  agnocast::init(static_cast<int>(sizeof(argv) / sizeof(argv[0])), argv);
}

std::size_t count_callback_groups(const std::shared_ptr<agnocast::Node> & node)
{
  std::size_t count = 0;
  node->for_each_callback_group([&count](const rclcpp::CallbackGroup::SharedPtr &) { ++count; });
  return count;
}
}  // namespace

// SetUp/TearDown only manage the agnocast context lifecycle; each test calls agnocast::init()
// itself so that tests needing process-global arguments can supply their own argv.
class AgnocastNodeConstructionTest : public ::testing::Test
{
protected:
  void SetUp() override { reset_context_for_test(); }

  void TearDown() override
  {
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
    reset_context_for_test();
  }
};

TEST_F(AgnocastNodeConstructionTest, construct_with_parameter_services_enabled)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.start_parameter_services(true);
  EXPECT_NO_THROW(
    { auto node = std::make_shared<agnocast::Node>("test_node_param_srv_on", options); });
}

TEST_F(AgnocastNodeConstructionTest, construct_with_parameter_services_disabled)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  EXPECT_NO_THROW(
    { auto node = std::make_shared<agnocast::Node>("test_node_param_srv_off", options); });
}

TEST_F(AgnocastNodeConstructionTest, construct_with_default_options)
{
  agnocast::init(0, nullptr);

  EXPECT_NO_THROW({ auto node = std::make_shared<agnocast::Node>("test_node_default"); });
}

// Verifies that agnocast::Node honors NodeOptions::use_global_arguments() when resolving
// parameter overrides. A process-global override (here use_sim_time, which the CIE client
// nodes must not inherit) should leak into the node only when use_global_arguments is true.
TEST_F(AgnocastNodeConstructionTest, use_global_arguments_true_inherits_global_use_sim_time)
{
  init_with_global_use_sim_time();

  rclcpp::NodeOptions options;
  options.use_global_arguments(true);
  auto node = std::make_shared<agnocast::Node>("test_node_global_args_on", options);

  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
}

TEST_F(AgnocastNodeConstructionTest, use_global_arguments_false_ignores_global_use_sim_time)
{
  init_with_global_use_sim_time();

  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  auto node = std::make_shared<agnocast::Node>("test_node_global_args_off", options);

  EXPECT_FALSE(node->get_parameter("use_sim_time").as_bool());
}

// Verifies that agnocast::Node forwards NodeOptions::allow_undeclared_parameters() to
// NodeParameters: getting an undeclared parameter throws only when the option is false.
TEST_F(AgnocastNodeConstructionTest, allow_undeclared_parameters_true_permits_undeclared_get)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto node = std::make_shared<agnocast::Node>("test_node_allow_undeclared_on", options);

  EXPECT_NO_THROW({ node->get_parameter("undeclared_param"); });
}

TEST_F(AgnocastNodeConstructionTest, allow_undeclared_parameters_false_rejects_undeclared_get)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  auto node = std::make_shared<agnocast::Node>("test_node_allow_undeclared_off", options);

  EXPECT_THROW(
    { node->get_parameter("undeclared_param"); },
    rclcpp::exceptions::ParameterNotDeclaredException);
}

// Verifies that agnocast::Node forwards
// NodeOptions::automatically_declare_parameters_from_overrides() to NodeParameters: overrides that
// no declare_parameter() call claimed become declared parameters, so prefix lookups find them.
TEST_F(AgnocastNodeConstructionTest, automatically_declare_parameters_from_overrides_declares_them)
{
  // Arrange
  agnocast::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides(
    {rclcpp::Parameter("group.first", 1), rclcpp::Parameter("group.second", "two")});

  // Act
  auto node = std::make_shared<agnocast::Node>("test_node_auto_declare_on", options);

  // Assert: the overrides are declared and keep their values.
  EXPECT_EQ(node->get_parameter("group.first").as_int(), 1);
  EXPECT_EQ(node->get_parameter("group.second").as_string(), "two");

  // Assert: the point of the option is that a caller which does not know the names can still
  // enumerate them, with the prefix stripped from the keys.
  std::map<std::string, rclcpp::Parameter> values;
  ASSERT_TRUE(node->get_parameters("group", values));
  ASSERT_EQ(values.size(), 2u);
  EXPECT_EQ(values.at("first").as_int(), 1);
  EXPECT_EQ(values.at("second").as_string(), "two");
}

// Overrides are declared with a dynamically typed descriptor, so a later set() may change the type.
TEST_F(
  AgnocastNodeConstructionTest, automatically_declare_parameters_from_overrides_uses_dynamic_typing)
{
  // Arrange
  agnocast::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("group.first", 1)});
  auto node = std::make_shared<agnocast::Node>("test_node_auto_declare_dynamic", options);

  // Act
  const auto descriptor = node->describe_parameter("group.first");

  // Assert
  EXPECT_TRUE(descriptor.dynamic_typing);
}

TEST_F(AgnocastNodeConstructionTest, automatically_declare_parameters_from_overrides_off_by_default)
{
  // Arrange
  agnocast::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("group.first", 1)});

  // Act
  auto node = std::make_shared<agnocast::Node>("test_node_auto_declare_off", options);

  // Assert: the override was resolved but never declared, so it is invisible to lookups.
  EXPECT_FALSE(node->has_parameter("group.first"));
  std::map<std::string, rclcpp::Parameter> values;
  EXPECT_FALSE(node->get_parameters("group", values));
  EXPECT_TRUE(values.empty());
}

// agnocast::Node must forward `options.use_clock_thread()` to NodeTimeSource, which creates
// a dedicated clock callback group only when it is true. These tests pin that by counting
// callback groups. `use_sim_time` is enabled to trigger the clock subscription setup.

TEST_F(AgnocastNodeConstructionTest, clock_thread_disabled_skips_clock_callback_group)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  options.use_clock_thread(false);

  auto node = std::make_shared<agnocast::Node>("test_node_clock_thread_off", options);

  // Only the default callback group exists; no dedicated clock callback group is created.
  EXPECT_EQ(count_callback_groups(node), 1u);
}

TEST_F(AgnocastNodeConstructionTest, clock_thread_enabled_creates_clock_callback_group)
{
  agnocast::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  options.use_clock_thread(true);

  auto node = std::make_shared<agnocast::Node>("test_node_clock_thread_on", options);

  // The default callback group plus the dedicated clock callback group.
  EXPECT_EQ(count_callback_groups(node), 2u);
}
