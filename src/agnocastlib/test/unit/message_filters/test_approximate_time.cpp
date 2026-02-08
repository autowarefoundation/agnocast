#include "agnocast/message_filters/sync_policies/approximate_time.hpp"
#include "agnocast/message_filters/synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <vector>

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

using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

// Helper class to manage test messages (prevents memory leaks with ipc_shared_ptr fd=-1)
class TestMsgManager
{
public:
  MsgConstPtr createMsg(rclcpp::Time stamp = rclcpp::Time(), int data = 0)
  {
    Msg * raw = new Msg();
    raw->header.stamp = stamp;
    raw->data = data;
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

class Helper
{
public:
  Helper() : count_(0) {}

  void cb() { ++count_; }

  int32_t count_;
};

using TimePair = std::pair<rclcpp::Time, rclcpp::Time>;
using TimeAndTopic = std::pair<rclcpp::Time, unsigned int>;
struct TimeQuad
{
  TimeQuad(rclcpp::Time p, rclcpp::Time q, rclcpp::Time r, rclcpp::Time s)
  {
    time[0] = p;
    time[1] = q;
    time[2] = r;
    time[3] = s;
  }
  rclcpp::Time time[4];
};

using Policy2 = ApproximateTime<Msg, Msg>;
using Policy4 = ApproximateTime<Msg, Msg, Msg, Msg>;
using Sync2 = Synchronizer<Policy2>;
using Sync4 = Synchronizer<Policy4>;

//----------------------------------------------------------
//                Test Class (for 2 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTest
{
public:
  ApproximateTimeSynchronizerTest(
    const std::vector<TimeAndTopic> & input, const std::vector<TimePair> & output,
    uint32_t queue_size)
  : input_(input), output_(output), output_position_(0), sync_(Policy2(queue_size))
  {
    sync_.registerCallback(&ApproximateTimeSynchronizerTest::callback, this);
  }

  void callback(const MsgConstPtr & p, const MsgConstPtr & q)
  {
    ASSERT_NE(nullptr, p.get());
    ASSERT_NE(nullptr, q.get());
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].first, p->header.stamp);
    EXPECT_EQ(output_[output_position_].second, q->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (unsigned int i = 0; i < input_.size(); i++) {
      if (input_[i].second == 0) {
        auto p = mgr_.createMsg(input_[i].first);
        sync_.add<0>(p);
      } else {
        auto q = mgr_.createMsg(input_[i].first);
        sync_.add<1>(q);
      }
    }
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  TestMsgManager mgr_;
  const std::vector<TimeAndTopic> & input_;
  const std::vector<TimePair> & output_;
  unsigned int output_position_;

public:
  Sync2 sync_;
};

//----------------------------------------------------------
//                Test Class (for 4 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTestQuad
{
public:
  ApproximateTimeSynchronizerTestQuad(
    const std::vector<TimeAndTopic> & input, const std::vector<TimeQuad> & output,
    uint32_t queue_size)
  : input_(input), output_(output), output_position_(0), sync_(Policy4(queue_size))
  {
    sync_.registerCallback(&ApproximateTimeSynchronizerTestQuad::callback, this);
  }

  void callback(
    const MsgConstPtr & p, const MsgConstPtr & q, const MsgConstPtr & r, const MsgConstPtr & s)
  {
    ASSERT_NE(nullptr, p.get());
    ASSERT_NE(nullptr, q.get());
    ASSERT_NE(nullptr, r.get());
    ASSERT_NE(nullptr, s.get());
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].time[0], p->header.stamp);
    EXPECT_EQ(output_[output_position_].time[1], q->header.stamp);
    EXPECT_EQ(output_[output_position_].time[2], r->header.stamp);
    EXPECT_EQ(output_[output_position_].time[3], s->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (unsigned int i = 0; i < input_.size(); i++) {
      auto p = mgr_.createMsg(input_[i].first);
      switch (input_[i].second) {
        case 0:
          sync_.add<0>(p);
          break;
        case 1:
          sync_.add<1>(p);
          break;
        case 2:
          sync_.add<2>(p);
          break;
        case 3:
          sync_.add<3>(p);
          break;
      }
    }
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  TestMsgManager mgr_;
  const std::vector<TimeAndTopic> & input_;
  const std::vector<TimeQuad> & output_;
  unsigned int output_position_;

public:
  Sync4 sync_;
};

//----------------------------------------------------------
//                   Test Suite
//----------------------------------------------------------
TEST(AgnocastApproximateTime, ExactMatch)
{
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));          // a
  input.push_back(TimeAndTopic(t, 1));          // A
  input.push_back(TimeAndTopic(t + s * 3, 0));  // b
  input.push_back(TimeAndTopic(t + s * 3, 1));  // B
  input.push_back(TimeAndTopic(t + s * 6, 0));  // c
  input.push_back(TimeAndTopic(t + s * 6, 1));  // C
  output.push_back(TimePair(t, t));
  output.push_back(TimePair(t + s * 3, t + s * 3));
  output.push_back(TimePair(t + s * 6, t + s * 6));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, PerfectMatch)
{
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   ...a..b.
  //           ...A..B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));          // a
  input.push_back(TimeAndTopic(t + s, 1));      // A
  input.push_back(TimeAndTopic(t + s * 3, 0));  // b
  input.push_back(TimeAndTopic(t + s * 4, 1));  // B
  input.push_back(TimeAndTopic(t + s * 6, 0));  // c
  input.push_back(TimeAndTopic(t + s * 7, 1));  // C
  output.push_back(TimePair(t, t + s));
  output.push_back(TimePair(t + s * 3, t + s * 4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, ImperfectMatch)
{
  // Input A:  a.xb..c.
  // Input B:  .A...B.C
  // Output:   ..a...c.
  //           ..A...B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));          // a
  input.push_back(TimeAndTopic(t + s, 1));      // A
  input.push_back(TimeAndTopic(t + s * 2, 0));  // x
  input.push_back(TimeAndTopic(t + s * 3, 0));  // b
  input.push_back(TimeAndTopic(t + s * 5, 1));  // B
  input.push_back(TimeAndTopic(t + s * 6, 0));  // c
  input.push_back(TimeAndTopic(t + s * 7, 1));  // C
  output.push_back(TimePair(t, t + s));
  output.push_back(TimePair(t + s * 6, t + s * 5));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, Acceleration)
{
  // Time:     0123456789012345678
  // Input A:  a...........b....c.
  // Input B:  .......A.......B..C
  // Output:   ............b.....c
  //           ............A.....C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));           // a
  input.push_back(TimeAndTopic(t + s * 7, 1));   // A
  input.push_back(TimeAndTopic(t + s * 12, 0));  // b
  input.push_back(TimeAndTopic(t + s * 15, 1));  // B
  input.push_back(TimeAndTopic(t + s * 17, 0));  // c
  input.push_back(TimeAndTopic(t + s * 18, 1));  // C
  output.push_back(TimePair(t + s * 12, t + s * 7));
  output.push_back(TimePair(t + s * 17, t + s * 18));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, DroppedMessages)
{
  // Queue size 1 (too small)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   .......b.....d.
  //           .......B.....D.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));           // a
  input.push_back(TimeAndTopic(t + s, 1));       // A
  input.push_back(TimeAndTopic(t + s * 3, 1));   // B
  input.push_back(TimeAndTopic(t + s * 4, 0));   // b
  input.push_back(TimeAndTopic(t + s * 7, 1));   // C
  input.push_back(TimeAndTopic(t + s * 8, 0));   // c
  input.push_back(TimeAndTopic(t + s * 10, 0));  // d
  input.push_back(TimeAndTopic(t + s * 11, 1));  // D
  input.push_back(TimeAndTopic(t + s * 13, 0));  // e
  input.push_back(TimeAndTopic(t + s * 14, 1));  // E
  output.push_back(TimePair(t + s * 4, t + s * 3));
  output.push_back(TimePair(t + s * 10, t + s * 11));

  ApproximateTimeSynchronizerTest sync_test(input, output, 1);
  sync_test.run();

  // Queue size 2 (just enough)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   ....a..b...c.d.
  //           ....A..B...C.D.
  std::vector<TimePair> output2;
  output2.push_back(TimePair(t, t + s));
  output2.push_back(TimePair(t + s * 4, t + s * 3));
  output2.push_back(TimePair(t + s * 8, t + s * 7));
  output2.push_back(TimePair(t + s * 10, t + s * 11));

  ApproximateTimeSynchronizerTest sync_test2(input, output2, 2);
  sync_test2.run();
}

TEST(AgnocastApproximateTime, LongQueue)
{
  // Queue size 5
  // Time:     012345678901234
  // Input A:  abcdefghiklmnp.
  // Input B:  ...j......o....
  // Output:   ..........l....
  //           ..........o....
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);

  input.push_back(TimeAndTopic(t, 0));                            // a
  input.push_back(TimeAndTopic(t + rclcpp::Duration(1, 0), 0));   // b
  input.push_back(TimeAndTopic(t + rclcpp::Duration(2, 0), 0));   // c
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 0));   // d
  input.push_back(TimeAndTopic(t + rclcpp::Duration(4, 0), 0));   // e
  input.push_back(TimeAndTopic(t + rclcpp::Duration(5, 0), 0));   // f
  input.push_back(TimeAndTopic(t + rclcpp::Duration(6, 0), 0));   // g
  input.push_back(TimeAndTopic(t + rclcpp::Duration(7, 0), 0));   // h
  input.push_back(TimeAndTopic(t + rclcpp::Duration(8, 0), 0));   // i
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 1));   // j
  input.push_back(TimeAndTopic(t + rclcpp::Duration(9, 0), 0));   // k
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 0));  // l
  input.push_back(TimeAndTopic(t + rclcpp::Duration(11, 0), 0));  // m
  input.push_back(TimeAndTopic(t + rclcpp::Duration(12, 0), 0));  // n
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 1));  // o
  input.push_back(TimeAndTopic(t + rclcpp::Duration(13, 0), 0));  // p
  output.push_back(TimePair(t + rclcpp::Duration(10, 0), t + rclcpp::Duration(10, 0)));

  ApproximateTimeSynchronizerTest sync_test(input, output, 5);
  sync_test.run();
}

TEST(AgnocastApproximateTime, DoublePublish)
{
  // Input A:  a..b
  // Input B:  .A.B
  // Output:   ...b
  //           ...B
  //              +
  //              a
  //              A
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));                           // a
  input.push_back(TimeAndTopic(t + s, 1));                       // A
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 1));  // B
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 0));  // b
  output.push_back(TimePair(t, t + s));
  output.push_back(TimePair(t + rclcpp::Duration(3, 0), t + rclcpp::Duration(3, 0)));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, FourTopics)
{
  // Time:     012345678901234
  // Input A:  a....e..i.m..n.
  // Input B:  .b....g..j....o
  // Input C:  ..c...h...k....
  // Input D:  ...d.f.....l...
  // Output:   ......a....e..m
  //           ......b....g..j
  //           ......c....h..k
  //           ......d....f..l
  std::vector<TimeAndTopic> input;
  std::vector<TimeQuad> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));                            // a
  input.push_back(TimeAndTopic(t + s, 1));                        // b
  input.push_back(TimeAndTopic(t + rclcpp::Duration(2, 0), 2));   // c
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 3));   // d
  input.push_back(TimeAndTopic(t + rclcpp::Duration(5, 0), 0));   // e
  input.push_back(TimeAndTopic(t + rclcpp::Duration(5, 0), 3));   // f
  input.push_back(TimeAndTopic(t + rclcpp::Duration(6, 0), 1));   // g
  input.push_back(TimeAndTopic(t + rclcpp::Duration(6, 0), 2));   // h
  input.push_back(TimeAndTopic(t + rclcpp::Duration(8, 0), 0));   // i
  input.push_back(TimeAndTopic(t + rclcpp::Duration(9, 0), 1));   // j
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 2));  // k
  input.push_back(TimeAndTopic(t + rclcpp::Duration(11, 0), 3));  // l
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 0));  // m
  input.push_back(TimeAndTopic(t + rclcpp::Duration(13, 0), 0));  // n
  input.push_back(TimeAndTopic(t + rclcpp::Duration(14, 0), 1));  // o
  output.push_back(TimeQuad(t, t + s, t + rclcpp::Duration(2, 0), t + rclcpp::Duration(3, 0)));
  output.push_back(TimeQuad(
    t + rclcpp::Duration(5, 0), t + rclcpp::Duration(6, 0), t + rclcpp::Duration(6, 0),
    t + rclcpp::Duration(5, 0)));
  output.push_back(TimeQuad(
    t + rclcpp::Duration(10, 0), t + rclcpp::Duration(9, 0), t + rclcpp::Duration(10, 0),
    t + rclcpp::Duration(11, 0)));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, EarlyPublish)
{
  // Time:     012345678901234
  // Input A:  a......e
  // Input B:  .b......
  // Input C:  ..c.....
  // Input D:  ...d....
  // Output:   .......a
  //           .......b
  //           .......c
  //           .......d
  std::vector<TimeAndTopic> input;
  std::vector<TimeQuad> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));          // a
  input.push_back(TimeAndTopic(t + s, 1));      // b
  input.push_back(TimeAndTopic(t + s * 2, 2));  // c
  input.push_back(TimeAndTopic(t + s * 3, 3));  // d
  input.push_back(TimeAndTopic(t + s * 7, 0));  // e
  output.push_back(TimeQuad(t, t + s, t + s * 2, t + s * 3));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10);
  sync_test.run();
}

TEST(AgnocastApproximateTime, RateBound)
{
  // Rate bound A: 1.5
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b...
  //           .A..B...
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));          // a
  input.push_back(TimeAndTopic(t + s, 1));      // A
  input.push_back(TimeAndTopic(t + s * 3, 0));  // b
  input.push_back(TimeAndTopic(t + s * 4, 1));  // B
  input.push_back(TimeAndTopic(t + s * 6, 0));  // c
  input.push_back(TimeAndTopic(t + s * 7, 1));  // C
  output.push_back(TimePair(t, t + s));
  output.push_back(TimePair(t + s * 3, t + s * 4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.sync_.setInterMessageLowerBound(0, s * 1.5);
  sync_test.run();

  // Rate bound A: 2
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b..c
  //           .A..B..C

  output.push_back(TimePair(t + s * 6, t + s * 7));

  ApproximateTimeSynchronizerTest sync_test2(input, output, 10);
  sync_test2.sync_.setInterMessageLowerBound(0, s * 2);
  sync_test2.run();
}

//----------------------------------------------------------
//              Agnocast-specific Tests
//----------------------------------------------------------

// Test: setAgePenalty
TEST(AgnocastApproximateTime, AgePenalty)
{
  TestMsgManager mgr;
  Policy2 policy(10);
  policy.setAgePenalty(0.5);
  Sync2 sync(policy);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));

  auto m0 = mgr.createMsg(rclcpp::Time(1, 0));
  auto m1 = mgr.createMsg(rclcpp::Time(1, 0));

  sync.add<0>(m0);
  sync.add<1>(m1);

  ASSERT_EQ(h.count_, 1);
}

// Test: setMaxIntervalDuration rejects too-wide intervals
TEST(AgnocastApproximateTime, MaxIntervalDuration)
{
  TestMsgManager mgr;
  Policy2 policy(10);
  // Set max interval to 1 second
  policy.setMaxIntervalDuration(rclcpp::Duration(1, 0));
  Sync2 sync(policy);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));

  // Messages 5 seconds apart - should NOT sync due to max interval
  auto m0 = mgr.createMsg(rclcpp::Time(0, 0));
  auto m1 = mgr.createMsg(rclcpp::Time(5, 0));

  sync.add<0>(m0);
  sync.add<1>(m1);

  ASSERT_EQ(h.count_, 0);
}

// Test: EventIn/EventOut - verify events preserve receipt time
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

TEST(AgnocastApproximateTime, EventInEventOut)
{
  TestMsgManager mgr;
  Sync2 sync(Policy2(10));
  EventHelper h;
  sync.registerCallback(&EventHelper::callback, &h);

  auto m0 = mgr.createMsg(rclcpp::Time(1, 0));
  auto m1 = mgr.createMsg(rclcpp::Time(1, 0));
  MessageEvent<Msg const> evt0(m0, rclcpp::Time(10, 0));
  MessageEvent<Msg const> evt1(m1, rclcpp::Time(20, 0));

  sync.add<0>(evt0);
  sync.add<1>(evt1);

  ASSERT_TRUE(h.e1_.getMessage());
  ASSERT_TRUE(h.e2_.getMessage());
  ASSERT_EQ(h.e1_.getReceiptTime(), evt0.getReceiptTime());
  ASSERT_EQ(h.e2_.getReceiptTime(), evt1.getReceiptTime());
}

// Test: policy constructor via Synchronizer with policy parameter
TEST(AgnocastApproximateTime, ConstructWithPolicy)
{
  TestMsgManager mgr;
  Policy2 policy(10);
  Sync2 sync(policy);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));

  auto m0 = mgr.createMsg(rclcpp::Time(1, 0));
  auto m1 = mgr.createMsg(rclcpp::Time(1, 0));

  sync.add<0>(m0);
  sync.add<1>(m1);

  ASSERT_EQ(h.count_, 1);
}
