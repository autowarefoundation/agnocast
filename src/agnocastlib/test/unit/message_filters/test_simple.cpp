#include "agnocast/message_filters/simple_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <memory>

using namespace agnocast::message_filters;
using namespace std::placeholders;

struct Msg
{
};

// Test helper to create ipc_shared_ptr for testing without kernel support
template <typename T>
agnocast::ipc_shared_ptr<T> make_test_ipc_shared_ptr(T * ptr)
{
  // Create ipc_shared_ptr with dummy values for testing
  return agnocast::ipc_shared_ptr<T>(ptr, "test_topic", 0, -1);
}

struct Filter : public SimpleFilter<Msg>
{
  using EventType = MessageEvent<Msg const>;

  void add(const EventType & evt) { signalMessage(evt); }
};

using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

class Helper
{
public:
  Helper() { counts_.fill(0); }

  void cb0(const MsgConstPtr &) { ++counts_[0]; }

  void cb1(MsgConstPtr) { ++counts_[1]; }

  void cb2(const MessageEvent<Msg const> &) { ++counts_[2]; }

  // Note: const M& and M (value copy) are not supported in agnocast
  // as ipc_shared_ptr always points to read-only shared memory

  std::array<int32_t, 30> counts_;
};

TEST(AgnocastSimpleFilter, callbackTypes)
{
  Helper h;
  Filter f;
  f.registerCallback(std::bind(&Helper::cb0, &h, _1));
  f.registerCallback<MsgConstPtr>(std::bind(&Helper::cb1, &h, _1));
  f.registerCallback<const MessageEvent<Msg const> &>(std::bind(&Helper::cb2, &h, _1));

  // Create a test message
  Msg const * raw_msg = new Msg const();
  auto msg = make_test_ipc_shared_ptr(raw_msg);

  f.add(Filter::EventType(msg));

  EXPECT_EQ(h.counts_[0], 1);
  EXPECT_EQ(h.counts_[1], 1);
  EXPECT_EQ(h.counts_[2], 1);

  // Clean up (since we're bypassing normal ipc_shared_ptr lifecycle)
  delete raw_msg;
}

struct OldFilter
{
  ::message_filters::Connection registerCallback(const std::function<void(const MsgConstPtr &)> &)
  {
    return ::message_filters::Connection();
  }
};

TEST(AgnocastSimpleFilter, oldRegisterWithNewFilter)
{
  OldFilter f;
  Helper h;
  f.registerCallback(std::bind(&Helper::cb0, &h, _1));
}
