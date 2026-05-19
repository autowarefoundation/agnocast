#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/parameter.hpp"

#include <gtest/gtest.h>

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
}  // namespace

// SetUp/TearDown only manage the agnocast context lifecycle; each test calls agnocast::init()
// itself so that tests needing process-global arguments can supply their own argv.
class AgnocastNodeConstructionTest : public ::testing::Test
{
protected:
  void SetUp() override { reset_context_for_test(); }

  void TearDown() override
  {
    agnocast::shutdown();
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
