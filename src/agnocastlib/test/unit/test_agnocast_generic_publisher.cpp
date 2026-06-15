#include "agnocast/agnocast_publisher.hpp"
#include "rclcpp/serialization.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>

// Defined in test_mocked_agnocast.cpp (same compilation unit / test target).
extern int publish_core_mock_called_count;

namespace
{

const std::string kKnownType = "std_msgs/msg/String";
const std::string kUnknownType = "totally_fake_package/msg/DoesNotExist";

std::string g_captured_log;

void log_capture_handler(
  const rcutils_log_location_t * /*location*/, int /*severity*/, const char * name,
  rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args)
{
  if (name == nullptr || std::string(name) != "Agnocast") {
    return;
  }
  char buf[1024];
  va_list args_copy;
  va_copy(args_copy, *args);
  vsnprintf(buf, sizeof(buf), format, args_copy);
  va_end(args_copy);
  g_captured_log += buf;
  g_captured_log += '\n';
}

class LogCapture
{
public:
  LogCapture()
  {
    g_captured_log.clear();
    previous_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(log_capture_handler);
  }
  ~LogCapture() { rcutils_logging_set_output_handler(previous_); }

private:
  rcutils_logging_output_handler_t previous_;
};

}  // namespace

class GenericPublisherUnitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_generic_pub_node");
    pub_ = std::make_unique<agnocast::GenericPublisher>(
      node_.get(), "/test_generic_pub_string", kKnownType, rclcpp::QoS{1});
  }

  void TearDown() override { rclcpp::shutdown(); }

  static rclcpp::SerializedMessage make_valid_serialized_message()
  {
    std_msgs::msg::String msg;
    msg.data = "hello from generic publisher unit test";
    rclcpp::Serialization<std_msgs::msg::String> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&msg, &serialized);
    return serialized;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<agnocast::GenericPublisher> pub_;
};

// ---------------------------------------------------------------------------
// Test 1: unknown topic type → std::runtime_error in constructor
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherUnitTest, typesupport_error_unknown_type)
{
  EXPECT_THROW(
    agnocast::GenericPublisher(node_.get(), "/test_topic", kUnknownType, rclcpp::QoS{1}),
    std::runtime_error);
}

// ---------------------------------------------------------------------------
// Test 2: valid serialized message → publish_core is called once
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherUnitTest, publish_calls_publish_core)
{
  publish_core_mock_called_count = 0;
  pub_->publish(make_valid_serialized_message());

  EXPECT_EQ(publish_core_mock_called_count, 1);
}

// ---------------------------------------------------------------------------
// Test 3: capacity == 0 → publish_core not called; ERROR is logged
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherUnitTest, publish_zero_capacity_skips_publish_core)
{
  publish_core_mock_called_count = 0;
  LogCapture capture;

  rclcpp::SerializedMessage empty_msg;  // capacity() == 0, size() == 0
  pub_->publish(empty_msg);

  EXPECT_EQ(publish_core_mock_called_count, 0);
  EXPECT_NE(g_captured_log.find("capacity of zero"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Test 4: capacity > 0 but size == 0 → publish_core not called; ERROR is logged
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherUnitTest, publish_zero_size_skips_publish_core)
{
  publish_core_mock_called_count = 0;
  LogCapture capture;

  rclcpp::SerializedMessage msg_with_capacity;  // capacity > 0, size == 0
  msg_with_capacity.reserve(64);
  pub_->publish(msg_with_capacity);

  EXPECT_EQ(publish_core_mock_called_count, 0);
  EXPECT_NE(g_captured_log.find("size of zero"), std::string::npos);
}
