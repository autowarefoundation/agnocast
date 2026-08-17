#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_only_callback_isolated_executor.hpp"
#include "agnocast/node/agnocast_only_multi_threaded_executor.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace
{

using namespace std::chrono_literals;

void reset_context_for_test()
{
  std::lock_guard<std::mutex> lock(agnocast::g_context_mtx);
  agnocast::g_context = agnocast::Context{};
}

void wait_until_or_fail(
  const std::function<bool()> & predicate, const std::chrono::milliseconds timeout,
  const std::string & failure_message)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return;
    }
    std::this_thread::sleep_for(2ms);
  }
  FAIL() << failure_message;
}

template <typename ExecutorT>
void expect_cancel_stops_spin_without_shutdown(const std::string & executor_name)
{
  agnocast::init(0, nullptr);
  auto executor = std::make_shared<ExecutorT>();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });

  std::this_thread::sleep_for(250ms);
  executor->cancel();

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    executor_name + " spin() did not return after cancel().");

  EXPECT_TRUE(agnocast::ok()) << executor_name
                              << " cancel() should not change agnocast::ok() without shutdown().";

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  agnocast::shutdown();
}

// Verifies that a cancel() issued *before* spin() ever starts still makes spin() return.
template <typename ExecutorT>
void expect_spin_returns_when_cancelled_before_spin(const std::string & executor_name)
{
  agnocast::init(0, nullptr);
  auto executor = std::make_shared<ExecutorT>();

  // cancel() happens before spin() is ever called.
  executor->cancel();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    executor_name + " spin() did not return after a cancel() that preceded it.");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  agnocast::shutdown();
}

}  // namespace

class InitOkShutdownTest : public ::testing::Test
{
protected:
  void SetUp() override { reset_context_for_test(); }

  void TearDown() override
  {
    agnocast::shutdown();
    reset_context_for_test();
  }
};

TEST_F(InitOkShutdownTest, OkIsTrueOnlyBetweenInitAndShutdown)
{
  EXPECT_FALSE(agnocast::ok());
  agnocast::init(0, nullptr);
  EXPECT_TRUE(agnocast::ok());
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok());
}

TEST_F(InitOkShutdownTest, InitShutdownCanBeRepeated)
{
  constexpr int kNrIterations = 5;

  for (int i = 0; i < kNrIterations; ++i) {
    EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should be false before init()";
    agnocast::init(0, nullptr);
    EXPECT_TRUE(agnocast::ok()) << "agnocast::ok() should be true after init()";
    agnocast::shutdown();
    EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should be false after shutdown()";
  }
}

TEST_F(InitOkShutdownTest, ShutdownStopsAgnocastOnlySingleThreadedExecutorSpin)
{
  agnocast::init(0, nullptr);
  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });
  std::this_thread::sleep_for(250ms);
  agnocast::shutdown();

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    "executor spin() did not return after agnocast::shutdown().");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

TEST_F(InitOkShutdownTest, ShutdownStopsAgnocastOnlyMultiThreadedExecutorSpin)
{
  agnocast::init(0, nullptr);
  auto executor = std::make_shared<agnocast::AgnocastOnlyMultiThreadedExecutor>();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });
  std::this_thread::sleep_for(250ms);
  agnocast::shutdown();

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    "executor spin() did not return after agnocast::shutdown().");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

TEST_F(InitOkShutdownTest, ShutdownStopsAgnocastOnlyCallbackIsolatedExecutorSpin)
{
  agnocast::init(0, nullptr);
  auto executor = std::make_shared<agnocast::AgnocastOnlyCallbackIsolatedExecutor>();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });
  std::this_thread::sleep_for(250ms);
  agnocast::shutdown();

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    "executor spin() did not return after agnocast::shutdown().");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

TEST_F(InitOkShutdownTest, CancelStopsAgnocastOnlySingleThreadedExecutorSpinWithoutShutdown)
{
  expect_cancel_stops_spin_without_shutdown<agnocast::AgnocastOnlySingleThreadedExecutor>(
    "AgnocastOnlySingleThreadedExecutor");
}

TEST_F(InitOkShutdownTest, CancelStopsAgnocastOnlyMultiThreadedExecutorSpinWithoutShutdown)
{
  expect_cancel_stops_spin_without_shutdown<agnocast::AgnocastOnlyMultiThreadedExecutor>(
    "AgnocastOnlyMultiThreadedExecutor");
}

TEST_F(InitOkShutdownTest, CancelStopsAgnocastOnlyCallbackIsolatedExecutorSpinWithoutShutdown)
{
  expect_cancel_stops_spin_without_shutdown<agnocast::AgnocastOnlyCallbackIsolatedExecutor>(
    "AgnocastOnlyCallbackIsolatedExecutor");
}

TEST_F(InitOkShutdownTest, CancelBeforeSpinStopsAgnocastOnlySingleThreadedExecutorSpin)
{
  expect_spin_returns_when_cancelled_before_spin<agnocast::AgnocastOnlySingleThreadedExecutor>(
    "AgnocastOnlySingleThreadedExecutor");
}

TEST_F(InitOkShutdownTest, CancelBeforeSpinStopsAgnocastOnlyMultiThreadedExecutorSpin)
{
  expect_spin_returns_when_cancelled_before_spin<agnocast::AgnocastOnlyMultiThreadedExecutor>(
    "AgnocastOnlyMultiThreadedExecutor");
}

TEST_F(InitOkShutdownTest, CancelBeforeSpinStopsAgnocastOnlyCallbackIsolatedExecutorSpin)
{
  expect_spin_returns_when_cancelled_before_spin<agnocast::AgnocastOnlyCallbackIsolatedExecutor>(
    "AgnocastOnlyCallbackIsolatedExecutor");
}

TEST_F(InitOkShutdownTest, CustomLoopUsingOkCanExitCleanly)
{
  agnocast::init(0, nullptr);

  std::atomic_bool entered_loop{false};
  std::atomic_bool loop_exited{false};
  std::thread loop_thread([&]() {
    while (agnocast::ok()) {
      entered_loop.store(true);
      std::this_thread::sleep_for(10ms);
    }
    loop_exited.store(true);
  });

  wait_until_or_fail(
    [&]() { return entered_loop.load(); }, 1s,
    "custom loop did not observe agnocast::ok()==true after init().");

  agnocast::shutdown();

  wait_until_or_fail(
    [&]() { return loop_exited.load(); }, 2s,
    "custom loop did not exit after agnocast::shutdown() changed ok() state.");

  if (loop_thread.joinable()) {
    loop_thread.join();
  }
}

TEST_F(InitOkShutdownTest, SigintAndSigtermStopAgnocastOnlySingleThreadedExecutorSpin)
{
  auto run_signal_case = [](int signum) {
    agnocast::init(0, nullptr);

    auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    std::atomic_bool spin_exited{false};

    std::thread spin_thread([&]() {
      executor->spin();
      spin_exited.store(true);
    });

    ASSERT_EQ(kill(getpid(), signum), 0);

    const char * signal_name = signum == SIGINT ? "SIGINT" : "SIGTERM";
    wait_until_or_fail(
      [&]() { return spin_exited.load(); }, 2s,
      std::string("executor spin() did not stop after ") + signal_name + ".");

    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    agnocast::shutdown();
  };

  run_signal_case(SIGINT);
  run_signal_case(SIGTERM);
}

TEST_F(InitOkShutdownTest, SigintAndSigtermStopAgnocastOnlyMultiThreadedExecutorSpin)
{
  auto run_signal_case = [](int signum) {
    agnocast::init(0, nullptr);

    auto executor = std::make_shared<agnocast::AgnocastOnlyMultiThreadedExecutor>();
    std::atomic_bool spin_exited{false};

    std::thread spin_thread([&]() {
      executor->spin();
      spin_exited.store(true);
    });

    ASSERT_EQ(kill(getpid(), signum), 0);

    const char * signal_name = signum == SIGINT ? "SIGINT" : "SIGTERM";
    wait_until_or_fail(
      [&]() { return spin_exited.load(); }, 2s,
      std::string("executor spin() did not stop after ") + signal_name + ".");

    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    agnocast::shutdown();
  };

  run_signal_case(SIGINT);
  run_signal_case(SIGTERM);
}

TEST_F(InitOkShutdownTest, SigintAndSigtermStopAgnocastOnlyCallbackIsolatedExecutorSpin)
{
  auto run_signal_case = [](int signum) {
    agnocast::init(0, nullptr);

    auto executor = std::make_shared<agnocast::AgnocastOnlyCallbackIsolatedExecutor>();
    std::atomic_bool spin_exited{false};

    std::thread spin_thread([&]() {
      executor->spin();
      spin_exited.store(true);
    });

    ASSERT_EQ(kill(getpid(), signum), 0);

    const char * signal_name = signum == SIGINT ? "SIGINT" : "SIGTERM";
    wait_until_or_fail(
      [&]() { return spin_exited.load(); }, 2s,
      std::string("executor spin() did not stop after ") + signal_name + ".");

    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    agnocast::shutdown();
  };

  run_signal_case(SIGINT);
  run_signal_case(SIGTERM);
}

TEST_F(InitOkShutdownTest, InitAndShutdownAreIdempotent)
{
  // Calling shutdown() before init() should do nothing and not cause any errors.
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok())
    << "agnocast::ok() should still be false after shutdown() without init()";

  // Calling init() multiple times should not cause any errors and should keep the context
  // initialized.
  agnocast::init(0, nullptr);
  EXPECT_TRUE(agnocast::ok()) << "agnocast::ok() should be true after first init()";
  agnocast::init(0, nullptr);
  EXPECT_TRUE(agnocast::ok()) << "agnocast::ok() should still be true after second init()";

  // Calling shutdown() multiple times should not cause any errors and should keep the context
  // shutdown.
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should be false after first shutdown()";
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should still be false after second shutdown()";
}
