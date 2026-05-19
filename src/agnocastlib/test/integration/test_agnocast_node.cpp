#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/parameter.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>

class AgnocastNodeConstructionTest : public ::testing::Test
{
protected:
  void SetUp() override { agnocast::init(0, nullptr); }
  void TearDown() override { agnocast::shutdown(); }
};

TEST_F(AgnocastNodeConstructionTest, construct_with_parameter_services_enabled)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(true);
  EXPECT_NO_THROW(
    { auto node = std::make_shared<agnocast::Node>("test_node_param_srv_on", options); });
}

TEST_F(AgnocastNodeConstructionTest, construct_with_parameter_services_disabled)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  EXPECT_NO_THROW(
    { auto node = std::make_shared<agnocast::Node>("test_node_param_srv_off", options); });
}

TEST_F(AgnocastNodeConstructionTest, construct_with_default_options)
{
  EXPECT_NO_THROW({ auto node = std::make_shared<agnocast::Node>("test_node_default"); });
}

namespace
{
std::size_t count_callback_groups(const std::shared_ptr<agnocast::Node> & node)
{
  std::size_t count = 0;
  node->for_each_callback_group([&count](const rclcpp::CallbackGroup::SharedPtr &) { ++count; });
  return count;
}
}  // namespace

// agnocast::Node must forward `options.use_clock_thread()` to NodeTimeSource, which creates
// a dedicated clock callback group only when it is true. These tests pin that by counting
// callback groups. `use_sim_time` is enabled to trigger the clock subscription setup.

TEST_F(AgnocastNodeConstructionTest, clock_thread_disabled_skips_clock_callback_group)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  options.use_clock_thread(false);

  auto node = std::make_shared<agnocast::Node>("test_node_clock_thread_off", options);

  // Only the default callback group exists; no dedicated clock callback group is created.
  EXPECT_EQ(count_callback_groups(node), 1u);
}

TEST_F(AgnocastNodeConstructionTest, clock_thread_enabled_creates_clock_callback_group)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  options.use_clock_thread(true);

  auto node = std::make_shared<agnocast::Node>("test_node_clock_thread_on", options);

  // The default callback group plus the dedicated clock callback group.
  EXPECT_EQ(count_callback_groups(node), 2u);
}
