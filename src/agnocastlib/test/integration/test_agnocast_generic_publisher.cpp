#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_single_threaded_executor.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using StringMsg = std_msgs::msg::String;

namespace
{

// Polls `pred` until it returns true or `timeout` elapses.
template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout = 5000ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return pred();
}

}  // namespace

class GenericPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_generic_pub");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// Round-trip: serialized message is delivered to a typed Subscription.
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherTest, round_trip_to_typed_subscription)
{
  const std::string topic = "/test_generic_pub_to_typed_sub";
  const std::string type = "std_msgs/msg/String";
  const std::string expected_data = "hello from generic publisher";
  rclcpp::QoS qos{1};

  auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cbg;

  std::shared_ptr<const StringMsg> received;
  std::mutex mtx;

  auto sub = agnocast::create_subscription<StringMsg>(
    node_.get(), topic, qos,
    [&received, &mtx](agnocast::ipc_shared_ptr<const StringMsg> msg) {
      auto copy = std::make_shared<StringMsg>(*msg);
      std::lock_guard<std::mutex> lock(mtx);
      received = std::move(copy);
    },
    sub_opts);

  auto pub = agnocast::create_generic_publisher(node_.get(), topic, type, qos);

  StringMsg out_msg;
  out_msg.data = expected_data;
  rclcpp::Serialization<StringMsg> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&out_msg, &serialized);
  pub->publish(serialized);

  const bool delivered = wait_for([&] {
    std::lock_guard<std::mutex> lock(mtx);
    return received != nullptr;
  });

  ASSERT_TRUE(delivered) << "Timed out waiting for typed subscription callback";
  {
    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_EQ(received->data, expected_data);
  }
}

// ---------------------------------------------------------------------------
// Round-trip: serialized message is delivered to a GenericSubscription.
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherTest, round_trip_to_generic_subscription)
{
  const std::string topic = "/test_generic_pub_to_generic_sub";
  const std::string type = "std_msgs/msg/String";
  const std::string expected_data = "hello via generic subscription";
  rclcpp::QoS qos{1};

  auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cbg;

  std::shared_ptr<rclcpp::SerializedMessage> received;
  std::mutex mtx;
  auto sub = agnocast::create_generic_subscription(
    node_.get(), topic, type, qos,
    [&received, &mtx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      std::lock_guard<std::mutex> lock(mtx);
      received = std::move(msg);
    },
    sub_opts);

  auto pub = agnocast::create_generic_publisher(node_.get(), topic, type, qos);

  StringMsg out_msg;
  out_msg.data = expected_data;
  rclcpp::Serialization<StringMsg> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&out_msg, &serialized);
  pub->publish(serialized);

  const bool delivered = wait_for([&] {
    std::lock_guard<std::mutex> lock(mtx);
    return received != nullptr;
  });

  ASSERT_TRUE(delivered) << "Timed out waiting for GenericSubscription callback";

  rclcpp::Serialization<StringMsg> deser;
  StringMsg decoded;
  {
    std::lock_guard<std::mutex> lock(mtx);
    deser.deserialize_message(received.get(), &decoded);
  }
  EXPECT_EQ(decoded.data, expected_data);
}

// ---------------------------------------------------------------------------
// Lifecycle: constructor registers the publisher; topic name and GID are populated.
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherTest, lifecycle_registers_publisher)
{
  const std::string topic = "/test_generic_pub_lifecycle";
  const std::string type = "std_msgs/msg/String";
  rclcpp::QoS qos{1};

  {
    auto pub = agnocast::create_generic_publisher(node_.get(), topic, type, qos);

    EXPECT_STRNE(pub->get_topic_name(), "");

    const rmw_gid_t & gid = pub->get_gid();
    EXPECT_EQ(gid.data[0], static_cast<uint8_t>('A'));
    EXPECT_EQ(gid.data[1], static_cast<uint8_t>('G'));
  }
}

// ---------------------------------------------------------------------------
// Constructor error: unknown topic type throws std::runtime_error before any IPC.
// ---------------------------------------------------------------------------
TEST_F(GenericPublisherTest, typesupport_error_unknown_type)
{
  const std::string unknown_type = "totally_fake_package/msg/DoesNotExist";

  EXPECT_THROW(
    agnocast::GenericPublisher(node_.get(), "/test_topic", unknown_type, rclcpp::QoS{1}),
    std::runtime_error);
}

// =========================================
// agnocast::Node tests
// =========================================

class AgnocastNodeGenericPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_generic_pub_agnocast_node");
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    executor_.reset();
    agnocast::shutdown();
  }

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// Round-trip: GenericPublisher works correctly with agnocast::Node.
// ---------------------------------------------------------------------------
TEST_F(AgnocastNodeGenericPublisherTest, round_trip_serialization)
{
  const std::string topic = "/test_agnocast_node_generic_pub";
  const std::string type = "std_msgs/msg/String";
  const std::string expected_data = "hello from agnocast node generic publisher";
  rclcpp::QoS qos{1};

  auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cbg;

  std::shared_ptr<rclcpp::SerializedMessage> received;
  std::mutex mtx;
  auto sub = agnocast::create_generic_subscription(
    node_.get(), topic, type, qos,
    [&received, &mtx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      std::lock_guard<std::mutex> lock(mtx);
      received = std::move(msg);
    },
    sub_opts);

  auto pub = agnocast::create_generic_publisher(node_.get(), topic, type, qos);

  StringMsg out_msg;
  out_msg.data = expected_data;
  rclcpp::Serialization<StringMsg> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&out_msg, &serialized);
  pub->publish(serialized);

  const bool delivered = wait_for([&] {
    std::lock_guard<std::mutex> lock(mtx);
    return received != nullptr;
  });

  ASSERT_TRUE(delivered) << "Timed out waiting for GenericSubscription callback";

  rclcpp::Serialization<StringMsg> deser;
  StringMsg decoded;
  {
    std::lock_guard<std::mutex> lock(mtx);
    deser.deserialize_message(received.get(), &decoded);
  }
  EXPECT_EQ(decoded.data, expected_data);
}
