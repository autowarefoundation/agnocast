#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/tf2/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_ros/buffer_interface.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
int initialize_subscriber_call_count = 0;
std::vector<std::string> initialized_topic_names;
std::vector<rclcpp::QoS> initialized_qos_values;

void reset_capture_state()
{
  initialize_subscriber_call_count = 0;
  initialized_topic_names.clear();
  initialized_qos_values.clear();
}
}  // namespace

extern "C" int ioctl(int, unsigned long, ...)
{
  return 0;
}

namespace agnocast
{
void validate_ld_preload()
{
}

void release_subscriber_reference(const std::string &, const topic_local_id_t, const int64_t)
{
}

mqd_t open_mq_for_subscription(
  const std::string &, const topic_local_id_t, std::pair<mqd_t, std::string> & mq_subscription)
{
  mq_subscription = std::make_pair(static_cast<mqd_t>(-1), std::string{});
  return -1;
}

void remove_mq(const std::pair<mqd_t, std::string> &)
{
}

union ioctl_add_subscriber_args SubscriptionBase::initialize(
  const rclcpp::QoS & qos, const bool, const bool, const bool, const std::string &)
{
  initialize_subscriber_call_count++;
  initialized_topic_names.push_back(topic_name_);
  initialized_qos_values.push_back(qos);
  union ioctl_add_subscriber_args args {
  };
  args.ret_id = 0;
  return args;
}

BridgeMode get_bridge_mode()
{
  return BridgeMode::Off;
}
}  // namespace agnocast

namespace
{
geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent_frame, const std::string & child_frame, double tx,
  const tf2::TimePoint & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.header.stamp = tf2_ros::toMsg(stamp);
  t.child_frame_id = child_frame;
  t.transform.translation.x = tx;
  t.transform.rotation.w = 1.0;
  return t;
}

// Subscriber-side ipc_shared_ptr does not delete `raw_msg` — the caller must.
agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage> make_subscriber_ipc_ptr(
  tf2_msgs::msg::TFMessage * raw_msg, const std::string & topic_name)
{
  return agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage>(raw_msg, topic_name, 0, 0);
}

constexpr tf2::TimePoint kStamp{std::chrono::seconds(10)};
constexpr tf2::TimePoint kFarFutureStamp{std::chrono::seconds(10) + std::chrono::seconds(100)};
}  // namespace

class TransformListenerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    reset_capture_state();

    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    node_ = std::make_shared<agnocast::Node>("tf_listener_test_node", options);

    buffer_ = std::make_shared<tf2::BufferCore>();
  }

  void TearDown() override { agnocast::shutdown(); }

  agnocast::Node::SharedPtr node_;
  std::shared_ptr<tf2::BufferCore> buffer_;
};

// =========================================
// Constructor (node-based)
// =========================================

TEST_F(
  TransformListenerTest, node_constructor_subscribes_with_volatile_tf_and_transient_local_tf_static)
{
  // Arrange
  const auto expected_dynamic_durability = tf2_ros::DynamicListenerQoS().durability();
  const auto expected_static_durability = tf2_ros::StaticListenerQoS().durability();

  // Act
  agnocast::TransformListener listener(*buffer_, *node_, /*spin_thread=*/false);

  // Assert
  ASSERT_EQ(initialize_subscriber_call_count, 2);
  ASSERT_EQ(initialized_topic_names.size(), 2u);
  bool saw_tf = false;
  bool saw_tf_static = false;
  for (size_t i = 0; i < initialized_topic_names.size(); ++i) {
    if (initialized_topic_names[i] == "/tf") {
      EXPECT_EQ(initialized_qos_values[i].durability(), expected_dynamic_durability);
      saw_tf = true;
    } else if (initialized_topic_names[i] == "/tf_static") {
      EXPECT_EQ(initialized_qos_values[i].durability(), expected_static_durability);
      saw_tf_static = true;
    }
  }
  EXPECT_TRUE(saw_tf);
  EXPECT_TRUE(saw_tf_static);
}

// =========================================
// Constructor (simplified)
// =========================================

TEST_F(TransformListenerTest, simplified_constructor_constructs_without_external_node)
{
  // Act / Assert: smoke test the simplified constructor — the rest of its behavior is
  // covered by the node-based test above, which it delegates to.
  EXPECT_NO_THROW({ agnocast::TransformListener listener(*buffer_, /*spin_thread=*/false); });
}

// =========================================
// subscription_callback
// =========================================

TEST_F(
  TransformListenerTest,
  subscription_callback_forwards_is_static_true_so_frame_answers_arbitrary_times)
{
  // Arrange
  agnocast::TransformListener listener(*buffer_, *node_, /*spin_thread=*/false);
  auto * raw_msg = new tf2_msgs::msg::TFMessage();
  raw_msg->transforms.push_back(make_transform("map", "imu", 2.0, kStamp));

  // Act
  listener.subscription_callback(
    make_subscriber_ipc_ptr(raw_msg, "/tf_static"), /*is_static=*/true);

  // Assert
  std::string err;
  EXPECT_TRUE(buffer_->canTransform("map", "imu", kFarFutureStamp, &err)) << err;

  // Cleanup
  delete raw_msg;
}

TEST_F(
  TransformListenerTest,
  subscription_callback_forwards_is_static_false_so_frame_does_not_answer_arbitrary_times)
{
  // Arrange
  agnocast::TransformListener listener(*buffer_, *node_, /*spin_thread=*/false);
  auto * raw_msg = new tf2_msgs::msg::TFMessage();
  raw_msg->transforms.push_back(make_transform("map", "base_link", 1.0, kStamp));

  // Act
  listener.subscription_callback(make_subscriber_ipc_ptr(raw_msg, "/tf"), /*is_static=*/false);

  // Assert
  EXPECT_FALSE(buffer_->canTransform("map", "base_link", kFarFutureStamp));

  // Cleanup
  delete raw_msg;
}

TEST_F(TransformListenerTest, subscription_callback_handles_empty_message_safely)
{
  // Arrange
  agnocast::TransformListener listener(*buffer_, *node_, /*spin_thread=*/false);
  auto * raw_msg = new tf2_msgs::msg::TFMessage();

  // Act
  EXPECT_NO_THROW(
    listener.subscription_callback(make_subscriber_ipc_ptr(raw_msg, "/tf"), /*is_static=*/false));

  // Assert
  EXPECT_FALSE(buffer_->canTransform("map", "base_link", kStamp));

  // Cleanup
  delete raw_msg;
}

TEST_F(TransformListenerTest, subscription_callback_inserts_all_transforms_from_a_single_message)
{
  // Arrange
  agnocast::TransformListener listener(*buffer_, *node_, /*spin_thread=*/false);
  auto * raw_msg = new tf2_msgs::msg::TFMessage();
  raw_msg->transforms.push_back(make_transform("map", "a", 1.0, kStamp));
  raw_msg->transforms.push_back(make_transform("a", "b", 2.0, kStamp));

  // Act
  listener.subscription_callback(make_subscriber_ipc_ptr(raw_msg, "/tf"), /*is_static=*/false);

  // Assert
  EXPECT_TRUE(buffer_->canTransform("map", "a", kStamp));
  EXPECT_TRUE(buffer_->canTransform("a", "b", kStamp));

  // Cleanup
  delete raw_msg;
}
