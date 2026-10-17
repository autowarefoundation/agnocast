#include "agnocast/internal/qos_parameters.hpp"

#include <rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include <gtest/gtest.h>

#include <string>

using agnocast::internal::declare_qos_parameters;
using agnocast::internal::QosOverrideEntity;

class QosParametersTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static rclcpp::QoS declare_for(
    rclcpp::Node & node, const rclcpp::QosOverridingOptions & options, const rclcpp::QoS & qos,
    const std::string & topic, const QosOverrideEntity entity)
  {
    return declare_qos_parameters(
      options, node.get_node_parameters_interface(), topic, qos, entity);
  }
};

TEST_F(QosParametersTest, depth_override_replaces_the_constructor_value)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({rclcpp::Parameter("qos_overrides./topic.publisher.depth", 9)});
  rclcpp::Node node("qos_param_depth", node_options);
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Depth});

  // Act
  const rclcpp::QoS actual = declare_for(
    node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);

  // Assert
  EXPECT_EQ(actual.depth(), 9u);
  const auto descriptor = node.describe_parameter("qos_overrides./topic.publisher.depth");
  EXPECT_TRUE(descriptor.read_only);
  EXPECT_EQ(descriptor.description, "qos policy {depth} for publisher {/topic}");
}

TEST_F(QosParametersTest, an_id_is_inserted_into_the_parameter_name)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("qos_overrides./topic.publisher_cam.depth", 4)});
  rclcpp::Node node("qos_param_id", node_options);
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Depth}, nullptr, "cam");

  // Act
  const rclcpp::QoS actual = declare_for(
    node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);

  // Assert
  EXPECT_EQ(actual.depth(), 4u);
  EXPECT_EQ(
    node.describe_parameter("qos_overrides./topic.publisher_cam.depth").description,
    "qos policy {depth} for publisher {/topic} with id {cam}");
}

TEST_F(QosParametersTest, an_already_declared_parameter_is_reused)
{
  // Arrange
  rclcpp::Node node("qos_param_existing");
  node.declare_parameter(
    "qos_overrides./topic.publisher.depth", rclcpp::ParameterValue(static_cast<int64_t>(6)));
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Depth});

  // Act
  const rclcpp::QoS actual = declare_for(
    node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);

  // Assert
  EXPECT_EQ(actual.depth(), 6u);
}

TEST_F(QosParametersTest, a_subscription_ignores_a_lifespan_override)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({rclcpp::Parameter(
    "qos_overrides./topic.subscription.lifespan", static_cast<int64_t>(1000000000))});
  rclcpp::Node node("qos_param_lifespan", node_options);
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Lifespan});

  // Act
  const rclcpp::QoS actual = declare_for(
    node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Subscription);

  // Assert
  EXPECT_EQ(actual.lifespan().nanoseconds(), 0);
  EXPECT_FALSE(node.has_parameter("qos_overrides./topic.subscription.lifespan"));
}

TEST_F(QosParametersTest, a_publisher_applies_a_lifespan_override)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({rclcpp::Parameter(
    "qos_overrides./topic.publisher.lifespan", static_cast<int64_t>(1000000000))});
  rclcpp::Node node("qos_param_pub_lifespan", node_options);
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Lifespan});

  // Act
  const rclcpp::QoS actual = declare_for(
    node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);

  // Assert
  EXPECT_EQ(actual.lifespan().nanoseconds(), 1000000000);
}

TEST_F(QosParametersTest, an_unknown_policy_string_is_rejected)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("qos_overrides./topic.publisher.reliability", "not_a_policy")});
  rclcpp::Node node("qos_param_bad_reliability", node_options);
  const rclcpp::QosOverridingOptions options({rclcpp::QosPolicyKind::Reliability});

  // Act & Assert
  EXPECT_THROW(
    {
      declare_for(
        node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);
    },
    std::invalid_argument);
}

TEST_F(QosParametersTest, a_failing_validation_callback_is_rejected)
{
  // Arrange
  rclcpp::Node node("qos_param_callback");
  const rclcpp::QosOverridingOptions options(
    {rclcpp::QosPolicyKind::Depth}, [](const rclcpp::QoS &) {
      rclcpp::QosCallbackResult result;
      result.successful = false;
      result.reason = "too shallow";
      return result;
    });

  // Act & Assert
  try {
    declare_for(
      node, options, rclcpp::QoS(rclcpp::KeepLast(1)), "/topic", QosOverrideEntity::Publisher);
    FAIL() << "expected InvalidQosOverridesException";
  } catch (const rclcpp::exceptions::InvalidQosOverridesException & error) {
    EXPECT_EQ(std::string(error.what()), "validation callback failed: too shallow");
  }
}
