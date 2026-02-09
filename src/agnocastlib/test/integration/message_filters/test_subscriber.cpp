#include "agnocast/agnocast.hpp"
#include "agnocast/message_filters/subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace agnocast::message_filters;
using namespace std::placeholders;

using Msg = std_msgs::msg::Int32;
using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

class Helper
{
public:
  Helper() : count_(0) {}

  void cb(const MsgConstPtr &) { ++count_; }

  int32_t count_;
};

struct ConstHelper
{
  void cb(const MsgConstPtr & msg) { msg_ = msg; }

  void reset() { msg_.reset(); }

  MsgConstPtr msg_;
};

class AgnocastSubscriberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);

    // Start spinning in a background thread
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void waitFor(
    std::function<bool()> condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (condition()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;
};

// Equivalent to ROS2 "simple" test
TEST_F(AgnocastSubscriberTest, simple)
{
  Helper h;
  Subscriber<Msg> sub(node_, "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic", rclcpp::QoS(10));

  // Wait for connection
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}

// Equivalent to ROS2 "simple_raw" test (raw node pointer)
TEST_F(AgnocastSubscriberTest, simple_raw)
{
  Helper h;
  Subscriber<Msg> sub(node_.get(), "test_topic_raw");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_raw", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}

// Equivalent to ROS2 "subUnsubSub" test
TEST_F(AgnocastSubscriberTest, subUnsubSub)
{
  Helper h;
  Subscriber<Msg> sub(node_, "test_topic_unsub");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_unsub", rclcpp::QoS(10));

  sub.unsubscribe();
  sub.subscribe();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}

// Equivalent to ROS2 "subUnsubSub_raw" test
TEST_F(AgnocastSubscriberTest, subUnsubSub_raw)
{
  Helper h;
  Subscriber<Msg> sub(node_.get(), "test_topic_unsub_raw");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_unsub_raw", rclcpp::QoS(10));

  sub.unsubscribe();
  sub.subscribe();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}

// Equivalent to ROS2 "switchRawAndShared" test
TEST_F(AgnocastSubscriberTest, switchRawAndShared)
{
  Helper h;
  Subscriber<Msg> sub(node_, "test_topic_switch1");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_switch2", rclcpp::QoS(10));

  sub.unsubscribe();
  sub.subscribe(node_.get(), "test_topic_switch2");

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}

// Equivalent to ROS2 "singleNonConstCallback" (const version for agnocast)
TEST_F(AgnocastSubscriberTest, singleConstCallback)
{
  ConstHelper h;
  Subscriber<Msg> sub(node_, "test_topic_const");
  sub.registerCallback(&ConstHelper::cb, &h);

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_const", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 123;
  pub->publish(std::move(msg));

  waitFor([&h]() { return static_cast<bool>(h.msg_); });

  ASSERT_TRUE(h.msg_);
  EXPECT_EQ(h.msg_->data, 123);

  // Reset ipc_shared_ptr before executor shutdown to avoid cleanup errors
  h.reset();
}

// Equivalent to ROS2 "multipleNonConstCallbacksFilterSubscriber" (const version)
TEST_F(AgnocastSubscriberTest, multipleCallbacks)
{
  ConstHelper h1, h2;
  Subscriber<Msg> sub(node_, "test_topic_multi");
  sub.registerCallback(&ConstHelper::cb, &h1);
  sub.registerCallback(&ConstHelper::cb, &h2);

  auto pub = agnocast::create_publisher<Msg>(node_.get(), "test_topic_multi", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 456;
  pub->publish(std::move(msg));

  waitFor([&h1, &h2]() { return static_cast<bool>(h1.msg_) && static_cast<bool>(h2.msg_); });

  ASSERT_TRUE(h1.msg_);
  ASSERT_TRUE(h2.msg_);
  EXPECT_EQ(h1.msg_->data, 456);
  EXPECT_EQ(h2.msg_->data, 456);
  // In agnocast, both callbacks receive the same pointer (read-only shared memory)
  EXPECT_EQ(h1.msg_.get(), h2.msg_.get());

  // Reset ipc_shared_ptr before executor shutdown to avoid cleanup errors
  h1.reset();
  h2.reset();
}

// ============================================================================
// Tests for agnocast::Node
// ============================================================================

class AgnocastNodeSubscriberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_agnocast_node");
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start spinning in a background thread
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
    // TODO(Koichi98): Call agnocast::shutdown() once available.
    // See https://github.com/tier4/agnocast/issues/1019
  }

  void waitFor(
    std::function<bool()> condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (condition()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(AgnocastNodeSubscriberTest, simple)
{
  Helper h;
  Subscriber<Msg, agnocast::Node> sub(node_, "test_topic_an");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));

  auto pub = node_->create_publisher<Msg>("test_topic_an", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto msg = pub->borrow_loaned_message();
  msg->data = 42;
  pub->publish(std::move(msg));

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_GT(h.count_, 0);
}
