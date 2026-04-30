#include "agnocast/node/agnocast_node.hpp"

#include <gtest/gtest.h>

#include <memory>

class AgnocastNodeConstructionTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// Regression test: agnocast::Node construction with start_parameter_services(true) used to
// SIGSEGV because Node passed `this` to NodeParameters in the initializer list, which then
// re-entered the half-built Node via ParameterService → BasicSubscription. Construction must
// now complete cleanly when parameter services are enabled.
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
