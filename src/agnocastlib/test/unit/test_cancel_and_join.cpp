#include "internal/cancel_and_join.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

using namespace std::chrono_literals;

// Force the ordering in #1636: cancel -> spinning.exchange(true) -> join.
// Replacing the retry with a single cancel makes this test hang (use a test timeout).
TEST(CancelAndJoinTest, RetriesCancellationLostBeforeSpinStarts)
{
  std::promise<void> first_cancel;
  auto first_cancelled = first_cancel.get_future();
  std::promise<void> spin_started;
  auto started = spin_started.get_future();
  std::promise<void> completion;
  auto finished = completion.get_future().share();
  std::atomic_bool spinning{false};
  int cancel_count = 0;

  std::thread worker([&]() {
    first_cancelled.wait();
    spinning.exchange(true);
    spin_started.set_value();
    while (spinning.load()) {
      std::this_thread::yield();
    }
    completion.set_value();
  });

  agnocast::detail::cancel_and_join(worker, finished, [&]() {
    if (++cancel_count == 1) {
      spinning.store(false);
      first_cancel.set_value();
    } else {
      started.wait();
      spinning.store(false);
    }
  });

  EXPECT_GE(cancel_count, 2);
  EXPECT_FALSE(worker.joinable());
  EXPECT_EQ(finished.wait_for(0ms), std::future_status::ready);
}

TEST(CancelAndJoinTest, DoesNotCancelAnAlreadyCompletedWorker)
{
  std::promise<void> completion;
  auto finished = completion.get_future().share();
  std::thread worker([&]() { completion.set_value(); });
  finished.wait();
  int cancel_count = 0;

  agnocast::detail::cancel_and_join(worker, finished, [&]() { ++cancel_count; });

  EXPECT_EQ(cancel_count, 0);
  EXPECT_FALSE(worker.joinable());
}

TEST(CancelAndJoinTest, CancelsARunningWorkerAndJoins)
{
  std::promise<void> entered;
  auto started = entered.get_future();
  std::promise<void> completion;
  auto finished = completion.get_future().share();
  std::atomic_bool stop{false};
  std::atomic_bool callback_finished{false};
  std::thread worker([&]() {
    entered.set_value();
    while (!stop.load()) {
      std::this_thread::yield();
    }
    callback_finished.store(true);
    completion.set_value();
  });
  started.wait();

  agnocast::detail::cancel_and_join(worker, finished, [&]() { stop.store(true); });

  EXPECT_TRUE(callback_finished.load());
  EXPECT_FALSE(worker.joinable());
}

TEST(CancelAndJoinTest, WaitsUntilTheWorkerHasFinished)
{
  std::promise<void> completion;
  auto finished = completion.get_future().share();
  std::promise<void> callback_release;
  auto released = callback_release.get_future();
  std::atomic_bool callback_finished{false};
  int cancel_count = 0;
  std::thread worker([&]() {
    released.wait();
    callback_finished.store(true);
    completion.set_value();
  });

  agnocast::detail::cancel_and_join(worker, finished, [&]() {
    if (++cancel_count == 3) {
      callback_release.set_value();
    }
  });

  EXPECT_GE(cancel_count, 3);
  EXPECT_TRUE(callback_finished.load());
  EXPECT_FALSE(worker.joinable());
}

TEST(CancelAndJoinTest, DoesNothingForANonJoinableThread)
{
  std::thread worker;
  std::shared_future<void> no_completion;
  int cancel_count = 0;

  agnocast::detail::cancel_and_join(worker, no_completion, [&]() { ++cancel_count; });

  EXPECT_EQ(cancel_count, 0);
}
