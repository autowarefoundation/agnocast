#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_rosout.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <mutex>

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

// =========================================
// Per-node rosout publisher tests
// =========================================

class AgnocastNodeRosoutTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * argv[] = {"test", "--ros-args", "--enable-rosout-logs"};
    int argc = 3;
    agnocast::init(argc, const_cast<char **>(argv));
  }

  void TearDown() override { agnocast::shutdown(); }
};

TEST_F(AgnocastNodeRosoutTest, each_node_registers_its_own_publisher)
{
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 0u);

  auto node_a = std::make_shared<agnocast::Node>("rosout_node_a");
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 1u);

  auto node_b = std::make_shared<agnocast::Node>("rosout_node_b");
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 2u);
}

TEST_F(AgnocastNodeRosoutTest, destroying_node_removes_its_publisher)
{
  auto node_a = std::make_shared<agnocast::Node>("rosout_destroy_a");
  auto node_b = std::make_shared<agnocast::Node>("rosout_destroy_b");
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 2u);

  node_a.reset();
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 1u);

  node_b.reset();
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 0u);
}

TEST_F(AgnocastNodeRosoutTest, no_rosout_flag_registers_no_publisher)
{
  agnocast::shutdown();
  agnocast::init(0, nullptr);

  auto node = std::make_shared<agnocast::Node>("rosout_no_flag_node");
  EXPECT_EQ(agnocast::get_rosout_publisher_count(), 0u);
}
