#include "agnocast/message_filters/sync_policies/exact_time.hpp"
#include "agnocast/message_filters/synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

using namespace agnocast::message_filters;
using namespace agnocast::message_filters::sync_policies;

struct Header
{
  rclcpp::Time stamp;
};

struct Msg
{
  Header header;
  int data;
};

// Test helper to create ipc_shared_ptr for testing without kernel support
template <typename T>
agnocast::ipc_shared_ptr<T> make_test_ipc_shared_ptr(T * ptr)
{
  return agnocast::ipc_shared_ptr<T>(ptr, "test_topic", 0, -1);
}

namespace message_filters
{
namespace message_traits
{
template <>
struct TimeStamp<Msg>
{
  static rclcpp::Time value(const Msg & m) { return m.header.stamp; }
};
}  // namespace message_traits
}  // namespace message_filters

using MsgPtr = agnocast::ipc_shared_ptr<Msg>;
using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

class Helper
{
public:
  Helper() : count_(0), drop_count_(0) {}

  void cb() { ++count_; }

  void dropcb() { ++drop_count_; }

  int32_t count_;
  int32_t drop_count_;
};

using Policy2 = ExactTime<Msg, Msg>;
using Policy3 = ExactTime<Msg, Msg, Msg>;
using Sync2 = Synchronizer<Policy2>;
using Sync3 = Synchronizer<Policy3>;

// Helper class to manage test messages (prevents memory leaks)
class TestMsgManager
{
public:
  MsgConstPtr createMsg(rclcpp::Time stamp = rclcpp::Time())
  {
    Msg * raw = new Msg();
    raw->header.stamp = stamp;
    msgs_.push_back(raw);
    return make_test_ipc_shared_ptr(raw);
  }

  ~TestMsgManager()
  {
    for (auto * msg : msgs_) {
      delete msg;
    }
  }

private:
  std::vector<Msg *> msgs_;
};

TEST(AgnocastExactTime, multipleTimes)
{
  Sync3 sync(Policy3(2));
  Helper h;
  TestMsgManager mgr;
  sync.registerCallback(std::bind(&Helper::cb, &h));

  auto m = mgr.createMsg(rclcpp::Time());

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);

  auto m2 = mgr.createMsg(rclcpp::Time(100000000));
  sync.add<1>(m2);
  ASSERT_EQ(h.count_, 0);
  sync.add<0>(m2);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m2);
  ASSERT_EQ(h.count_, 1);
}

TEST(AgnocastExactTime, queueSize)
{
  Sync3 sync(Policy3(1));
  Helper h;
  TestMsgManager mgr;
  sync.registerCallback(std::bind(&Helper::cb, &h));

  auto m = mgr.createMsg(rclcpp::Time());

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);

  auto m2 = mgr.createMsg(rclcpp::Time(100000000));
  sync.add<1>(m2);
  ASSERT_EQ(h.count_, 0);

  auto m3 = mgr.createMsg(rclcpp::Time());
  sync.add<1>(m3);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m3);
  ASSERT_EQ(h.count_, 0);
}

TEST(AgnocastExactTime, dropCallback)
{
  Sync2 sync(Policy2(1));
  Helper h;
  TestMsgManager mgr;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  sync.getPolicy()->registerDropCallback(std::bind(&Helper::dropcb, &h));

  auto m = mgr.createMsg(rclcpp::Time());

  sync.add<0>(m);
  ASSERT_EQ(h.drop_count_, 0);

  auto m2 = mgr.createMsg(rclcpp::Time(100000000));
  sync.add<0>(m2);

  ASSERT_EQ(h.drop_count_, 1);
}

struct EventHelper
{
  void callback(const MessageEvent<Msg const> & e1, const MessageEvent<Msg const> & e2)
  {
    e1_ = e1;
    e2_ = e2;
  }

  MessageEvent<Msg const> e1_;
  MessageEvent<Msg const> e2_;
};

TEST(AgnocastExactTime, eventInEventOut)
{
  Sync2 sync(Policy2(2));
  EventHelper h;
  TestMsgManager mgr;
  sync.registerCallback(&EventHelper::callback, &h);

  auto m = mgr.createMsg(rclcpp::Time());
  MessageEvent<Msg const> evt(m, rclcpp::Time(4, 0));

  sync.add<0>(evt);
  sync.add<1>(evt);

  ASSERT_TRUE(h.e1_.getMessage());
  ASSERT_TRUE(h.e2_.getMessage());
  ASSERT_EQ(h.e1_.getReceiptTime(), evt.getReceiptTime());
  ASSERT_EQ(h.e2_.getReceiptTime(), evt.getReceiptTime());
}
