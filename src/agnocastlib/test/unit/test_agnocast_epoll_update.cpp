#include "agnocast/agnocast_epoll_update_dispatcher.hpp"

#include <gtest/gtest.h>

namespace agnocast
{

namespace
{
void drain(agnocast::EpollUpdateTracker & tracker)
{
  (void)tracker.take_update_request();
}
}  // namespace

TEST(EpollUpdateDispatcherTest, NotifyAllSendsToSingleTracker)
{
  auto tracker = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker);

  EXPECT_FALSE(tracker.take_update_request());

  EpollUpdateDispatcher::get_instance().request_update_all();

  EXPECT_TRUE(tracker.take_update_request());
}

TEST(EpollUpdateDispatcherTest, NotifyDoesNotAffectUnrelatedTrackers)
{
  auto tracker1 = EpollUpdateDispatcher::get_instance().register_tracker();
  auto tracker2 = EpollUpdateDispatcher::get_instance().register_tracker();
  auto tracker3 = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker1);
  drain(tracker2);
  drain(tracker3);

  EXPECT_FALSE(tracker1.take_update_request());
  EXPECT_FALSE(tracker2.take_update_request());
  EXPECT_FALSE(tracker3.take_update_request());

  EpollUpdateDispatcher::get_instance().request_update(tracker2.id());

  EXPECT_FALSE(tracker1.take_update_request());
  EXPECT_TRUE(tracker2.take_update_request());
  EXPECT_FALSE(tracker3.take_update_request());
}

TEST(EpollUpdateTrackerTest, InitialStateIsSet)
{
  // A freshly registered tracker has need_update=true by design so that
  // the executor performs an epoll update on startup.
  auto tracker = EpollUpdateDispatcher::get_instance().register_tracker();
  EXPECT_TRUE(tracker.take_update_request());
}

TEST(EpollUpdateTrackerTest, TakeUpdateRequestClearsFlag)
{
  auto tracker = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker);

  EXPECT_FALSE(tracker.take_update_request());

  EpollUpdateDispatcher::get_instance().request_update(tracker.id());

  EXPECT_TRUE(tracker.take_update_request());
  // Second call must return false — flag was cleared by the first call.
  EXPECT_FALSE(tracker.take_update_request());
}

TEST(EpollUpdateTrackerTest, TakeUpdateRequestIdempotentWhenNotSet)
{
  auto tracker = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker);

  EXPECT_FALSE(tracker.take_update_request());
  EXPECT_FALSE(tracker.take_update_request());
}

TEST(EpollUpdateTrackerTest, TakeUpdateRequestDoesNotAffectOtherTrackers)
{
  auto tracker1 = EpollUpdateDispatcher::get_instance().register_tracker();
  auto tracker2 = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker1);
  drain(tracker2);

  EXPECT_FALSE(tracker1.take_update_request());
  EXPECT_FALSE(tracker2.take_update_request());

  EpollUpdateDispatcher::get_instance().request_update_all();

  // Only tracker1 consumes the flag; tracker2 must remain set.
  EXPECT_TRUE(tracker1.take_update_request());
  EXPECT_TRUE(tracker2.take_update_request());
}

TEST(EpollUpdateTrackerTest, TakeUpdateRequestReturnsAfterMultipleNotifies)
{
  auto tracker = EpollUpdateDispatcher::get_instance().register_tracker();
  drain(tracker);

  // Notify multiple times; flag is boolean so only one read is needed.
  EpollUpdateDispatcher::get_instance().request_update(tracker.id());
  EpollUpdateDispatcher::get_instance().request_update(tracker.id());

  EXPECT_TRUE(tracker.take_update_request());
  EXPECT_FALSE(tracker.take_update_request());
}

}  // namespace agnocast
