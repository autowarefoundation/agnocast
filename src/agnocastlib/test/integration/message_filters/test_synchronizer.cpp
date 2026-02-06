#include "agnocast/agnocast.hpp"
#include "agnocast/message_filters/subscriber.hpp"
#include "agnocast/message_filters/sync_policies/exact_time.hpp"
#include "agnocast/message_filters/synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace agnocast::message_filters;
using namespace agnocast::message_filters::sync_policies;

using Msg = std_msgs::msg::Header;
using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

namespace message_filters
{
namespace message_traits
{
template <>
struct TimeStamp<std_msgs::msg::Header>
{
  static rclcpp::Time value(const std_msgs::msg::Header & m)
  {
    return rclcpp::Time(m.stamp, RCL_ROS_TIME);
  }
};
}  // namespace message_traits
}  // namespace message_filters

class Helper
{
public:
  Helper() : count_(0) {}

  void cb2(const MsgConstPtr &, const MsgConstPtr &) { ++count_; }

  int32_t count_;
};

class AgnocastSynchronizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_synchronizer_node");
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
    executor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void waitFor(
    std::function<bool()> condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
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

// Test full 9-channel pipeline: Publisher -> Subscriber -> Synchronizer -> ExactTime -> callback
TEST_F(AgnocastSynchronizerTest, exactTimeSync9)
{
  using Policy9 = ExactTime<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg>;

  Subscriber<Msg> sub0(node_, "sync9_topic_0");
  Subscriber<Msg> sub1(node_, "sync9_topic_1");
  Subscriber<Msg> sub2(node_, "sync9_topic_2");
  Subscriber<Msg> sub3(node_, "sync9_topic_3");
  Subscriber<Msg> sub4(node_, "sync9_topic_4");
  Subscriber<Msg> sub5(node_, "sync9_topic_5");
  Subscriber<Msg> sub6(node_, "sync9_topic_6");
  Subscriber<Msg> sub7(node_, "sync9_topic_7");
  Subscriber<Msg> sub8(node_, "sync9_topic_8");

  Synchronizer<Policy9> sync(Policy9(10), sub0, sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8);
  Helper h;
  sync.registerCallback(std::bind(
    [&h](
      const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
      const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
      const MsgConstPtr &) { ++h.count_; },
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
    std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8,
    std::placeholders::_9));

  auto pub0 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_0", rclcpp::QoS(10));
  auto pub1 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_1", rclcpp::QoS(10));
  auto pub2 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_2", rclcpp::QoS(10));
  auto pub3 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_3", rclcpp::QoS(10));
  auto pub4 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_4", rclcpp::QoS(10));
  auto pub5 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_5", rclcpp::QoS(10));
  auto pub6 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_6", rclcpp::QoS(10));
  auto pub7 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_7", rclcpp::QoS(10));
  auto pub8 = agnocast::create_publisher<Msg>(node_.get(), "sync9_topic_8", rclcpp::QoS(10));

  // Wait for publisher/subscriber connections
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Publish with matching timestamp on all 9 topics
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1;
  stamp.nanosec = 0;

  auto publish_with_stamp = [&stamp](auto & pub) {
    auto msg = pub->borrow_loaned_message();
    msg->stamp = stamp;
    pub->publish(std::move(msg));
  };

  publish_with_stamp(pub0);
  publish_with_stamp(pub1);
  publish_with_stamp(pub2);
  publish_with_stamp(pub3);
  publish_with_stamp(pub4);
  publish_with_stamp(pub5);
  publish_with_stamp(pub6);
  publish_with_stamp(pub7);
  publish_with_stamp(pub8);

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_EQ(h.count_, 1);
}

// Test partial arrival: publish on one topic first, then complete
TEST_F(AgnocastSynchronizerTest, exactTimeSyncPartialThenComplete)
{
  using Policy2 = ExactTime<Msg, Msg>;

  Subscriber<Msg> sub0(node_, "sync_partial_topic_0");
  Subscriber<Msg> sub1(node_, "sync_partial_topic_1");

  Synchronizer<Policy2> sync(Policy2(10), sub0, sub1);
  Helper h;
  sync.registerCallback(&Helper::cb2, &h);

  auto pub0 = agnocast::create_publisher<Msg>(node_.get(), "sync_partial_topic_0", rclcpp::QoS(10));
  auto pub1 = agnocast::create_publisher<Msg>(node_.get(), "sync_partial_topic_1", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 10;
  stamp.nanosec = 0;

  // Publish only on topic 0
  {
    auto msg = pub0->borrow_loaned_message();
    msg->stamp = stamp;
    pub0->publish(std::move(msg));
  }

  // Wait to confirm callback does NOT fire with only one message
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(h.count_, 0);

  // Now publish on topic 1 with same timestamp -> should trigger sync
  {
    auto msg = pub1->borrow_loaned_message();
    msg->stamp = stamp;
    pub1->publish(std::move(msg));
  }

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_EQ(h.count_, 1);
}

// Test mismatched timestamps don't sync, then matching timestamps do
TEST_F(AgnocastSynchronizerTest, exactTimeSyncNoMatchThenMatch)
{
  using Policy2 = ExactTime<Msg, Msg>;

  Subscriber<Msg> sub0(node_, "sync_nomatch_topic_0");
  Subscriber<Msg> sub1(node_, "sync_nomatch_topic_1");

  Synchronizer<Policy2> sync(Policy2(10), sub0, sub1);
  Helper h;
  sync.registerCallback(&Helper::cb2, &h);

  auto pub0 = agnocast::create_publisher<Msg>(node_.get(), "sync_nomatch_topic_0", rclcpp::QoS(10));
  auto pub1 = agnocast::create_publisher<Msg>(node_.get(), "sync_nomatch_topic_1", rclcpp::QoS(10));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Publish with different timestamps -> should NOT sync
  {
    builtin_interfaces::msg::Time stamp0;
    stamp0.sec = 100;
    stamp0.nanosec = 0;
    auto msg = pub0->borrow_loaned_message();
    msg->stamp = stamp0;
    pub0->publish(std::move(msg));
  }
  {
    builtin_interfaces::msg::Time stamp1;
    stamp1.sec = 200;
    stamp1.nanosec = 0;
    auto msg = pub1->borrow_loaned_message();
    msg->stamp = stamp1;
    pub1->publish(std::move(msg));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(h.count_, 0);

  // Now publish with matching timestamps -> should sync
  {
    builtin_interfaces::msg::Time stamp_match;
    stamp_match.sec = 300;
    stamp_match.nanosec = 0;

    auto msg0 = pub0->borrow_loaned_message();
    msg0->stamp = stamp_match;
    pub0->publish(std::move(msg0));

    auto msg1 = pub1->borrow_loaned_message();
    msg1->stamp = stamp_match;
    pub1->publish(std::move(msg1));
  }

  waitFor([&h]() { return h.count_ > 0; });

  EXPECT_EQ(h.count_, 1);
}
