#include "agnocast/message_filters/pass_through.hpp"

#include <gtest/gtest.h>

#include <functional>

using namespace agnocast::message_filters;
using namespace std::placeholders;

struct Msg
{
};

template <typename T>
agnocast::ipc_shared_ptr<T> make_test_ipc_shared_ptr(T * ptr)
{
  return agnocast::ipc_shared_ptr<T>(ptr, "test_topic", 0, 0);
}

using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;
using EventType = MessageEvent<Msg const>;

TEST(AgnocastPassThrough, addMConstPtr)
{
  PassThrough<Msg> pt;
  int count = 0;
  pt.registerCallback([&count](const MsgConstPtr &) { ++count; });

  Msg const * raw = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw);
  pt.add(msg);

  EXPECT_EQ(count, 1);
  delete raw;
}

TEST(AgnocastPassThrough, addEventType)
{
  PassThrough<Msg> pt;
  int count = 0;
  pt.registerCallback(
    std::function<void(const EventType &)>([&count](const EventType &) { ++count; }));

  Msg const * raw = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw);
  pt.add(EventType(msg));

  EXPECT_EQ(count, 1);
  delete raw;
}

TEST(AgnocastPassThrough, connectInput)
{
  PassThrough<Msg> upstream;
  PassThrough<Msg> pt;
  pt.connectInput(upstream);

  int count = 0;
  pt.registerCallback([&count](const MsgConstPtr &) { ++count; });

  Msg const * raw = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw);
  upstream.add(msg);

  EXPECT_EQ(count, 1);
  delete raw;
}

TEST(AgnocastPassThrough, reconnect)
{
  PassThrough<Msg> upstream1;
  PassThrough<Msg> upstream2;
  PassThrough<Msg> pt(upstream1);

  int count = 0;
  pt.registerCallback([&count](const MsgConstPtr &) { ++count; });

  Msg const * raw1 = new Msg const();
  auto msg1 = make_test_ipc_shared_ptr(raw1);
  upstream1.add(msg1);
  EXPECT_EQ(count, 1);

  // Reconnect to upstream2
  pt.connectInput(upstream2);

  // upstream1 should no longer trigger pt
  Msg const * raw2 = new Msg const();
  auto msg2 = make_test_ipc_shared_ptr(raw2);
  upstream1.add(msg2);
  EXPECT_EQ(count, 1);

  // upstream2 should trigger pt
  Msg const * raw3 = new Msg const();
  auto msg3 = make_test_ipc_shared_ptr(raw3);
  upstream2.add(msg3);
  EXPECT_EQ(count, 2);

  delete raw1;
  delete raw2;
  delete raw3;
}

TEST(AgnocastPassThrough, chain)
{
  PassThrough<Msg> pt1;
  PassThrough<Msg> pt2(pt1);
  PassThrough<Msg> pt3(pt2);

  int count = 0;
  pt3.registerCallback([&count](const MsgConstPtr &) { ++count; });

  Msg const * raw = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw);
  pt1.add(msg);

  EXPECT_EQ(count, 1);
  delete raw;
}

TEST(AgnocastPassThrough, multipleCallbacks)
{
  PassThrough<Msg> pt;
  int count1 = 0;
  int count2 = 0;
  pt.registerCallback([&count1](const MsgConstPtr &) { ++count1; });
  pt.registerCallback([&count2](const MsgConstPtr &) { ++count2; });

  Msg const * raw = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw);
  pt.add(msg);

  EXPECT_EQ(count1, 1);
  EXPECT_EQ(count2, 1);
  delete raw;
}
