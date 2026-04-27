#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/tf2/static_transform_broadcaster.hpp"
#include "agnocast/node/tf2/transform_broadcaster.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

// =========================================
// Mock state — captures publish calls so the tests can assert on the
// transmitted TFMessage contents. This file is linked into a separate test
// binary (test_unit_tf2_broadcasters_agnocastlib) so it does not conflict
// with the generic mocks in test_mocked_agnocast.cpp.
// =========================================

namespace
{
int initialize_publisher_call_count = 0;
std::string last_initialized_topic_name;
rclcpp::QoS last_initialized_qos{10};

int publish_core_call_count = 0;
std::string last_published_topic_name;
tf2_msgs::msg::TFMessage last_published_message;

uint32_t mock_borrowed_publisher_num = 0;

void reset_capture_state()
{
  initialize_publisher_call_count = 0;
  last_initialized_topic_name.clear();
  last_initialized_qos = rclcpp::QoS{10};
  publish_core_call_count = 0;
  last_published_topic_name.clear();
  last_published_message = tf2_msgs::msg::TFMessage{};
  mock_borrowed_publisher_num = 0;
}
}  // namespace

extern "C" uint32_t agnocast_get_borrowed_publisher_num()
{
  return mock_borrowed_publisher_num;
}

namespace agnocast
{
void release_subscriber_reference(const std::string &, const topic_local_id_t, const int64_t)
{
}

void increment_borrowed_publisher_num()
{
  mock_borrowed_publisher_num++;
}

void decrement_borrowed_publisher_num()
{
  if (mock_borrowed_publisher_num > 0) {
    mock_borrowed_publisher_num--;
  }
}

topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string &, const rclcpp::QoS & qos, const bool)
{
  initialize_publisher_call_count++;
  last_initialized_topic_name = topic_name;
  last_initialized_qos = qos;
  return 0;
}

union ioctl_publish_msg_args publish_core(
  const void *, const std::string & topic_name, const topic_local_id_t,
  const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> &)
{
  publish_core_call_count++;
  last_published_topic_name = topic_name;
  // Both broadcasters publish tf2_msgs::msg::TFMessage — safe to type-pun here.
  const auto * msg = reinterpret_cast<const tf2_msgs::msg::TFMessage *>(msg_virtual_address);
  last_published_message = *msg;
  return ioctl_publish_msg_args{};
}

BridgeMode get_bridge_mode()
{
  return BridgeMode::Off;
}
}  // namespace agnocast

// =========================================
// Test fixtures
// =========================================

namespace
{
geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent_frame, const std::string & child_frame, double tx)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;
  t.transform.translation.x = tx;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  return t;
}
}  // namespace

class TransformBroadcasterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    reset_capture_state();
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<agnocast::Node>("tf_broadcaster_test_node", options);
  }

  void TearDown() override { agnocast::shutdown(); }

  agnocast::Node::SharedPtr node_;
};

class StaticTransformBroadcasterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    reset_capture_state();
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<agnocast::Node>("static_tf_broadcaster_test_node", options);
  }

  void TearDown() override { agnocast::shutdown(); }

  agnocast::Node::SharedPtr node_;
};

// =========================================
// TransformBroadcaster
// =========================================

TEST_F(TransformBroadcasterTest, advertises_tf_topic)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  EXPECT_EQ(initialize_publisher_call_count, 1);
  EXPECT_EQ(last_initialized_topic_name, "/tf");
}

TEST_F(TransformBroadcasterTest, send_single_transform_publishes_one)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(make_transform("map", "base_link", 1.0));

  EXPECT_EQ(publish_core_call_count, 1);
  EXPECT_EQ(last_published_topic_name, "/tf");
  ASSERT_EQ(last_published_message.transforms.size(), 1u);
  EXPECT_EQ(last_published_message.transforms[0].header.frame_id, "map");
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 1.0);
}

TEST_F(TransformBroadcasterTest, send_vector_publishes_all_transforms_in_one_message)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  std::vector<geometry_msgs::msg::TransformStamped> transforms{
    make_transform("map", "base_link", 1.0), make_transform("base_link", "imu", 2.0),
    make_transform("base_link", "lidar", 3.0)};
  broadcaster.sendTransform(transforms);

  EXPECT_EQ(publish_core_call_count, 1);
  ASSERT_EQ(last_published_message.transforms.size(), 3u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 1.0);
  EXPECT_EQ(last_published_message.transforms[1].child_frame_id, "imu");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[1].transform.translation.x, 2.0);
  EXPECT_EQ(last_published_message.transforms[2].child_frame_id, "lidar");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[2].transform.translation.x, 3.0);
}

TEST_F(TransformBroadcasterTest, multiple_send_calls_each_publish_independently)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(make_transform("map", "a", 1.0));
  EXPECT_EQ(publish_core_call_count, 1);
  EXPECT_EQ(last_published_message.transforms.size(), 1u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "a");

  broadcaster.sendTransform(make_transform("map", "b", 2.0));
  EXPECT_EQ(publish_core_call_count, 2);
  EXPECT_EQ(last_published_message.transforms.size(), 1u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "b");
}

TEST_F(TransformBroadcasterTest, propagates_custom_qos_depth)
{
  agnocast::TransformBroadcaster broadcaster(*node_, rclcpp::QoS(rclcpp::KeepLast(7)));

  EXPECT_EQ(last_initialized_qos.depth(), 7u);
}

TEST_F(TransformBroadcasterTest, send_empty_vector_still_publishes_empty_message)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{});

  EXPECT_EQ(publish_core_call_count, 1);
  EXPECT_EQ(last_published_topic_name, "/tf");
  EXPECT_EQ(last_published_message.transforms.size(), 0u);
}

TEST_F(TransformBroadcasterTest, borrow_counter_returns_to_zero_after_send)
{
  agnocast::TransformBroadcaster broadcaster(*node_);

  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);
  broadcaster.sendTransform(make_transform("map", "base_link", 1.0));
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);

  broadcaster.sendTransform(make_transform("map", "imu", 2.0));
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);
}

TEST_F(TransformBroadcasterTest, exits_on_keep_all_history_qos)
{
  rclcpp::QoS bad_qos(rclcpp::KeepAll{});
  EXPECT_EXIT(
    agnocast::TransformBroadcaster(*node_, bad_qos), ::testing::ExitedWithCode(EXIT_FAILURE),
    "KeepAll");
}

// =========================================
// StaticTransformBroadcaster
// =========================================

TEST_F(StaticTransformBroadcasterTest, advertises_tf_static_topic)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  EXPECT_EQ(initialize_publisher_call_count, 1);
  EXPECT_EQ(last_initialized_topic_name, "/tf_static");
}

TEST_F(StaticTransformBroadcasterTest, send_single_transform_publishes_one)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(make_transform("map", "base_link", 1.0));

  EXPECT_EQ(publish_core_call_count, 1);
  EXPECT_EQ(last_published_topic_name, "/tf_static");
  ASSERT_EQ(last_published_message.transforms.size(), 1u);
  EXPECT_EQ(last_published_message.transforms[0].header.frame_id, "map");
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 1.0);
}

TEST_F(StaticTransformBroadcasterTest, accumulates_distinct_child_frames)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(make_transform("map", "base_link", 1.0));
  broadcaster.sendTransform(make_transform("base_link", "imu", 2.0));

  EXPECT_EQ(publish_core_call_count, 2);
  // The second publish must contain BOTH transforms — that is the transient_local
  // accumulation contract that lets late-joining subscribers see all static TFs.
  ASSERT_EQ(last_published_message.transforms.size(), 2u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "base_link");
  EXPECT_EQ(last_published_message.transforms[1].child_frame_id, "imu");
}

TEST_F(StaticTransformBroadcasterTest, overwrites_same_child_frame)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  broadcaster.sendTransform(make_transform("map", "sensor", 1.0));
  broadcaster.sendTransform(make_transform("map", "sensor", 9.0));

  EXPECT_EQ(publish_core_call_count, 2);
  ASSERT_EQ(last_published_message.transforms.size(), 1u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "sensor");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 9.0);
}

TEST_F(StaticTransformBroadcasterTest, vector_send_is_equivalent_to_repeated_single_sends)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  std::vector<geometry_msgs::msg::TransformStamped> transforms{
    make_transform("map", "a", 1.0), make_transform("map", "b", 2.0),
    make_transform("map", "a", 7.0)};  // 3rd entry overwrites 1st
  broadcaster.sendTransform(transforms);

  EXPECT_EQ(publish_core_call_count, 1);
  ASSERT_EQ(last_published_message.transforms.size(), 2u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "a");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 7.0);
  EXPECT_EQ(last_published_message.transforms[1].child_frame_id, "b");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[1].transform.translation.x, 2.0);
}

TEST_F(StaticTransformBroadcasterTest, accumulates_distinct_frames_unbounded)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  // Each new child_frame_id grows the accumulated buffer; verify the invariant up to a
  // representative count (no implicit upper bound exists in the broadcaster itself).
  for (int i = 0; i < 50; ++i) {
    broadcaster.sendTransform(make_transform("map", "frame_" + std::to_string(i), i));
  }

  EXPECT_EQ(publish_core_call_count, 50);
  ASSERT_EQ(last_published_message.transforms.size(), 50u);
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(last_published_message.transforms[i].child_frame_id, "frame_" + std::to_string(i));
    EXPECT_DOUBLE_EQ(
      last_published_message.transforms[i].transform.translation.x, static_cast<double>(i));
  }
}

TEST_F(StaticTransformBroadcasterTest, vector_send_overwrites_past_state)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  // Seed past state with "a".
  broadcaster.sendTransform(make_transform("map", "a", 1.0));

  // Vector that both overwrites past state ("a") and appends a new frame ("b").
  std::vector<geometry_msgs::msg::TransformStamped> transforms{
    make_transform("map", "a", 9.0), make_transform("map", "b", 2.0)};
  broadcaster.sendTransform(transforms);

  EXPECT_EQ(publish_core_call_count, 2);
  ASSERT_EQ(last_published_message.transforms.size(), 2u);
  EXPECT_EQ(last_published_message.transforms[0].child_frame_id, "a");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[0].transform.translation.x, 9.0);
  EXPECT_EQ(last_published_message.transforms[1].child_frame_id, "b");
  EXPECT_DOUBLE_EQ(last_published_message.transforms[1].transform.translation.x, 2.0);
}

TEST_F(StaticTransformBroadcasterTest, propagates_custom_qos_depth)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_, rclcpp::QoS(rclcpp::KeepLast(13)));

  EXPECT_EQ(last_initialized_qos.depth(), 13u);
}

TEST_F(StaticTransformBroadcasterTest, borrow_counter_returns_to_zero_after_send)
{
  agnocast::StaticTransformBroadcaster broadcaster(*node_);

  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);
  broadcaster.sendTransform(make_transform("map", "a", 1.0));
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);

  broadcaster.sendTransform(make_transform("map", "b", 2.0));
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0u);
}

TEST_F(StaticTransformBroadcasterTest, exits_on_keep_all_history_qos)
{
  rclcpp::QoS bad_qos(rclcpp::KeepAll{});
  EXPECT_EXIT(
    agnocast::StaticTransformBroadcaster(*node_, bad_qos), ::testing::ExitedWithCode(EXIT_FAILURE),
    "KeepAll");
}
