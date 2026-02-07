#include "agnocast/message_filters/synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <array>

using namespace agnocast::message_filters;
using namespace std::placeholders;
using ::message_filters::NullFilter;
using ::message_filters::NullType;

struct Header
{
  rclcpp::Time stamp;
};

struct Msg
{
  Header header;
  int data;
};

using MsgPtr = agnocast::ipc_shared_ptr<Msg>;
using MsgConstPtr = agnocast::ipc_shared_ptr<Msg const>;

// Test helper to create ipc_shared_ptr for testing without kernel support
template <typename T>
agnocast::ipc_shared_ptr<T> make_test_ipc_shared_ptr(T * ptr)
{
  // Use entry_id=0 (subscriber-side) to avoid triggering publisher-side cleanup logic
  return agnocast::ipc_shared_ptr<T>(ptr, "test_topic", 0, 0);
}

template <
  typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
  typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct NullPolicy : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  using Sync = Synchronizer<NullPolicy>;
  using Super = PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>;
  using Messages = typename Super::Messages;
  using Signal = typename Super::Signal;
  using Events = typename Super::Events;
  using RealTypeCount = typename Super::RealTypeCount;

  NullPolicy()
  {
    for (size_t i = 0; i < RealTypeCount::value; ++i) {
      added_[i] = 0;
    }
  }

  void initParent(Sync *) {}

  template <int i>
  void add(const typename std::tuple_element<i, Events>::type &)
  {
    ++added_.at(i);
  }

  std::array<int32_t, RealTypeCount::value> added_;
};

using Policy2 = NullPolicy<Msg, Msg>;
using Policy3 = NullPolicy<Msg, Msg, Msg>;
using Policy4 = NullPolicy<Msg, Msg, Msg, Msg>;
using Policy5 = NullPolicy<Msg, Msg, Msg, Msg, Msg>;
using Policy6 = NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg>;
using Policy7 = NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg>;
using Policy8 = NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg>;
using Policy9 = NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg>;

TEST(AgnocastSynchronizer, compile2)
{
  NullFilter<Msg> f0, f1;
  Synchronizer<Policy2> sync(f0, f1);
}

TEST(AgnocastSynchronizer, compile3)
{
  NullFilter<Msg> f0, f1, f2;
  Synchronizer<Policy3> sync(f0, f1, f2);
}

TEST(AgnocastSynchronizer, compile4)
{
  NullFilter<Msg> f0, f1, f2, f3;
  Synchronizer<Policy4> sync(f0, f1, f2, f3);
}

TEST(AgnocastSynchronizer, compile5)
{
  NullFilter<Msg> f0, f1, f2, f3, f4;
  Synchronizer<Policy5> sync(f0, f1, f2, f3, f4);
}

TEST(AgnocastSynchronizer, compile6)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5;
  Synchronizer<Policy6> sync(f0, f1, f2, f3, f4, f5);
}

TEST(AgnocastSynchronizer, compile7)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6;
  Synchronizer<Policy7> sync(f0, f1, f2, f3, f4, f5, f6);
}

TEST(AgnocastSynchronizer, compile8)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7;
  Synchronizer<Policy8> sync(f0, f1, f2, f3, f4, f5, f6, f7);
}

TEST(AgnocastSynchronizer, compile9)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7, f8;
  Synchronizer<Policy9> sync(f0, f1, f2, f3, f4, f5, f6, f7, f8);
}

void function2(const MsgConstPtr &, const MsgConstPtr &)
{
}
void function3(const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
{
}
void function4(const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
{
}
void function5(
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
  const MsgConstPtr &)
{
}
void function6(
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
  const MsgConstPtr &, const MsgConstPtr &)
{
}
void function7(
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
{
}
void function8(
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
  const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
{
}
void function9(
  const MsgConstPtr &, MsgConstPtr, const MsgConstPtr &, MsgConstPtr, const MsgConstPtr &,
  MsgConstPtr, const MessageEvent<Msg const> &, const MessageEvent<Msg const> &,
  const MsgConstPtr &)
{
}

TEST(AgnocastSynchronizer, compileFunction2)
{
  Synchronizer<Policy2> sync;
  sync.registerCallback(function2);
}

TEST(AgnocastSynchronizer, compileFunction3)
{
  Synchronizer<Policy3> sync;
  sync.registerCallback(function3);
}

TEST(AgnocastSynchronizer, compileFunction4)
{
  Synchronizer<Policy4> sync;
  sync.registerCallback(function4);
}

TEST(AgnocastSynchronizer, compileFunction5)
{
  Synchronizer<Policy5> sync;
  sync.registerCallback(function5);
}

TEST(AgnocastSynchronizer, compileFunction6)
{
  Synchronizer<Policy6> sync;
  sync.registerCallback(function6);
}

TEST(AgnocastSynchronizer, compileFunction7)
{
  Synchronizer<Policy7> sync;
  sync.registerCallback(function7);
}

TEST(AgnocastSynchronizer, compileFunction8)
{
  Synchronizer<Policy8> sync;
  sync.registerCallback(function8);
}

TEST(AgnocastSynchronizer, compileFunction9)
{
  Synchronizer<Policy9> sync;
  sync.registerCallback(function9);
}

struct MethodHelper
{
  void method2(const MsgConstPtr &, const MsgConstPtr &) {}
  void method3(const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &) {}
  void method4(const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
  {
  }
  void method5(
    const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
    const MsgConstPtr &)
  {
  }
  void method6(
    const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
    const MsgConstPtr &, const MsgConstPtr &)
  {
  }
  void method7(
    const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &,
    const MsgConstPtr &, const MsgConstPtr &, const MsgConstPtr &)
  {
  }
  void method8(
    const MsgConstPtr &, MsgConstPtr, const MsgConstPtr &, MsgConstPtr, const MsgConstPtr &,
    MsgConstPtr, const MessageEvent<Msg const> &, const MessageEvent<Msg const> &)
  {
  }
};

TEST(AgnocastSynchronizer, compileMethod2)
{
  MethodHelper h;
  Synchronizer<Policy2> sync;
  sync.registerCallback(&MethodHelper::method2, &h);
}

TEST(AgnocastSynchronizer, compileMethod3)
{
  MethodHelper h;
  Synchronizer<Policy3> sync;
  sync.registerCallback(&MethodHelper::method3, &h);
}

TEST(AgnocastSynchronizer, compileMethod4)
{
  MethodHelper h;
  Synchronizer<Policy4> sync;
  sync.registerCallback(&MethodHelper::method4, &h);
}

TEST(AgnocastSynchronizer, compileMethod5)
{
  MethodHelper h;
  Synchronizer<Policy5> sync;
  sync.registerCallback(&MethodHelper::method5, &h);
}

TEST(AgnocastSynchronizer, compileMethod6)
{
  MethodHelper h;
  Synchronizer<Policy6> sync;
  sync.registerCallback(&MethodHelper::method6, &h);
}

TEST(AgnocastSynchronizer, compileMethod7)
{
  MethodHelper h;
  Synchronizer<Policy7> sync;
  sync.registerCallback(&MethodHelper::method7, &h);
}

TEST(AgnocastSynchronizer, compileMethod8)
{
  MethodHelper h;
  Synchronizer<Policy8> sync;
  sync.registerCallback(&MethodHelper::method8, &h);
}

// Helper class to manage test messages (prevents memory leaks)
class TestMsgManager
{
public:
  MsgConstPtr createMsg()
  {
    Msg * raw = new Msg();
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

TEST(AgnocastSynchronizer, add2)
{
  Synchronizer<Policy2> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
}

TEST(AgnocastSynchronizer, add3)
{
  Synchronizer<Policy3> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
}

TEST(AgnocastSynchronizer, add4)
{
  Synchronizer<Policy4> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
}

TEST(AgnocastSynchronizer, add5)
{
  Synchronizer<Policy5> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
}

TEST(AgnocastSynchronizer, add6)
{
  Synchronizer<Policy6> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  for (int i = 0; i < 6; ++i) {
    ASSERT_EQ(sync.added_[i], 0);
  }
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
}

TEST(AgnocastSynchronizer, add7)
{
  Synchronizer<Policy7> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  for (int i = 0; i < 7; ++i) {
    ASSERT_EQ(sync.added_[i], 0);
  }
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
}

TEST(AgnocastSynchronizer, add8)
{
  Synchronizer<Policy8> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  for (int i = 0; i < 8; ++i) {
    ASSERT_EQ(sync.added_[i], 0);
  }
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
  sync.add<7>(m);
  ASSERT_EQ(sync.added_[7], 1);
}

TEST(AgnocastSynchronizer, add9)
{
  Synchronizer<Policy9> sync;
  TestMsgManager mgr;
  auto m = mgr.createMsg();

  for (int i = 0; i < 9; ++i) {
    ASSERT_EQ(sync.added_[i], 0);
  }
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
  sync.add<7>(m);
  ASSERT_EQ(sync.added_[7], 1);
  sync.add<8>(m);
  ASSERT_EQ(sync.added_[8], 1);
}
