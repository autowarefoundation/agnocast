#include "agnocast/agnocast_service_event_publisher.hpp"

#if AGNOCAST_HAS_SERVICE_INTROSPECTION

#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

class ServiceEventPublisherTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_service_event_publisher_node");
  }

  void TearDown() override
  {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<agnocast::ServiceEventPublisher> make_event_publisher(
    const std::string & service_type)
  {
    return std::make_unique<agnocast::ServiceEventPublisher>(
      node_.get(), "/test_service", service_type);
  }
};

TEST_F(ServiceEventPublisherTest, ConfigureRejectsANullClockWhenEnabling)
{
  // Arrange
  auto event_publisher = make_event_publisher("std_srvs/srv/SetBool");

  // Act & Assert
  EXPECT_THROW(
    event_publisher->configure(nullptr, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS),
    std::invalid_argument);
}

TEST_F(ServiceEventPublisherTest, ConfigureRejectsANullClockWhenDisabling)
{
  // Arrange
  auto event_publisher = make_event_publisher("std_srvs/srv/SetBool");

  // Act & Assert: rcl checks the clock for every state, so disabling is rejected too.
  EXPECT_THROW(
    event_publisher->configure(nullptr, rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_OFF),
    std::invalid_argument);
}

TEST_F(ServiceEventPublisherTest, ConfigureReportsAServiceTypeWhoseTypesupportCannotBeLoaded)
{
  // Arrange
  auto event_publisher = make_event_publisher("no_such_package/srv/NoSuchService");

  // Act & Assert: the typesupport is loaded here, not at construction.
  EXPECT_THROW(
    event_publisher->configure(
      node_->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_METADATA),
    std::runtime_error);
}

TEST_F(ServiceEventPublisherTest, ConfigureRejectsAQosAgnocastCannotUse)
{
  // Arrange
  auto event_publisher = make_event_publisher("std_srvs/srv/SetBool");

  // Act & Assert
  EXPECT_THROW(
    event_publisher->configure(
      node_->get_clock(), rclcpp::QoS(rclcpp::KeepAll()), RCL_SERVICE_INTROSPECTION_METADATA),
    std::invalid_argument);
}

#endif  // AGNOCAST_HAS_SERVICE_INTROSPECTION
