#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_callback_isolated_executor.hpp"
#include "agnocast/node/agnocast_only_multi_threaded_executor.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

#include <rclcpp/context.hpp>
#include <rclcpp/node_options.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <stdexcept>
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

TEST_F(InitOkShutdownTest, ShutdownIsIdempotent)
{
  // Calling shutdown() before init() should do nothing and not cause any errors.
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok())
    << "agnocast::ok() should still be false after shutdown() without init()";

  agnocast::init(0, nullptr);
  ASSERT_TRUE(agnocast::ok());

  // Calling shutdown() multiple times should not cause any errors and should keep the context
  // shutdown.
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should be false after first shutdown()";
  agnocast::shutdown();
  EXPECT_FALSE(agnocast::ok()) << "agnocast::ok() should still be false after second shutdown()";
}

// init() is not idempotent, unlike shutdown(). rclcpp draws the same line:
// rclcpp::Context::init() throws ContextAlreadyInitialized while shutdown() returns false.
TEST_F(InitOkShutdownTest, InitOnAnAlreadyInitializedContextThrows)
{
  agnocast::init(0, nullptr);
  ASSERT_TRUE(agnocast::ok());

  EXPECT_THROW(agnocast::init(0, nullptr), std::runtime_error);
  EXPECT_TRUE(agnocast::ok()) << "a rejected init() must leave the context as it was";
}

TEST_F(InitOkShutdownTest, InitAfterAShutdownStartsAFreshCycle)
{
  agnocast::init(0, nullptr);
  agnocast::shutdown();
  ASSERT_FALSE(agnocast::ok());

  EXPECT_NO_THROW(agnocast::init(0, nullptr));
  EXPECT_TRUE(agnocast::ok());
}

// A component container's main() calls rclcpp::init() but never agnocast::init(). An
// agnocast::Node loaded there still creates Agnocast-only executors internally (the
// use_sim_time clock thread, the tf2 listener thread), so they must come up on their own.
TEST_F(InitOkShutdownTest, AgnocastOnlyExecutorInitializesContextWhenInitWasNotCalled)
{
  ASSERT_FALSE(agnocast::ok()) << "precondition: agnocast::init() has not been called";

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();

  EXPECT_TRUE(agnocast::ok())
    << "constructing an Agnocast-only executor should bring the context up by itself";

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });

  std::this_thread::sleep_for(250ms);
  EXPECT_FALSE(spin_exited.load()) << "spin() should keep running without an explicit init()";

  agnocast::shutdown();
  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s,
    "executor spin() did not return after agnocast::shutdown().");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

// The lazy initialization above must not fight the shutdown path: objects built during
// teardown (AgnocastOnlyCallbackIsolatedExecutor::spin() creates an agnocast::Node for its
// client publisher, for instance) would otherwise revive the context and hang every spin
// loop that waits for agnocast::ok() to turn false.
TEST_F(InitOkShutdownTest, LazyInitializationDoesNotReviveShutdownContext)
{
  agnocast::init(0, nullptr);
  ASSERT_TRUE(agnocast::ok());

  agnocast::shutdown();
  ASSERT_FALSE(agnocast::ok());

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();

  EXPECT_FALSE(agnocast::ok())
    << "creating an executor after shutdown() must not bring the context back up";

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s, "spin() did not return with a shutdown context.");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

// A lazy initialization records no command line, and an init() that follows one could only
// honour its arguments by reconfiguring the rcl logging rclcpp already owns. init() goes first
// or not at all.
TEST_F(InitOkShutdownTest, InitAfterALazyInitializationThrows)
{
  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
  ASSERT_TRUE(agnocast::ok());

  EXPECT_THROW(agnocast::init(0, nullptr), std::runtime_error);

  std::lock_guard<std::mutex> lock(agnocast::g_context_mtx);
  EXPECT_EQ(nullptr, agnocast::g_context.get_parsed_arguments())
    << "lazy initialization must not fabricate global arguments";
}

// A Node has to bring the context up too, not just an Agnocast-only executor: everything hung
// off a node reads agnocast::ok(), and most of it never constructs such an executor.
TEST_F(InitOkShutdownTest, NodeInitializesContextWhenInitWasNotCalled)
{
  ASSERT_FALSE(agnocast::ok()) << "precondition: agnocast::init() has not been called";

  const auto node = std::make_shared<agnocast::Node>("lazy_init_node");

  EXPECT_TRUE(agnocast::ok()) << "constructing a Node should bring the context up by itself";
}

// Without this, the Agnocast-only executors a Node spawns keep spinning and every loop polling
// agnocast::ok() keeps waiting for a process that is already on its way out. Uses a context of
// its own so that the test leaves the global default one -- and the signal handlers
// rclcpp::shutdown() would uninstall with it -- alone.
TEST_F(InitOkShutdownTest, RclcppShutdownStopsALazilyInitializedContext)
{
  auto rclcpp_context = std::make_shared<rclcpp::Context>();
  rclcpp_context->init(0, nullptr);

  rclcpp::NodeOptions options;
  options.context(rclcpp_context);
  const auto node = std::make_shared<agnocast::Node>("hosted_node", options);
  ASSERT_TRUE(agnocast::ok());

  rclcpp_context->shutdown("test");

  EXPECT_FALSE(agnocast::ok())
    << "agnocast::ok() must follow the rclcpp context that governs the process";
}

// An AgnocastOnly process hands its nodes the global default context as well, but never
// initializes it.
TEST_F(InitOkShutdownTest, AnUninitializedRclcppContextIsNotTakenAsTheAuthority)
{
  agnocast::init(0, nullptr);

  const auto node = std::make_shared<agnocast::Node>("agnocast_only_node");

  EXPECT_TRUE(agnocast::ok())
    << "the global default context is not initialized here, so it must not govern";
}

TEST_F(InitOkShutdownTest, SigintStopsAgnocastOnlyExecutorWhenInitWasNotCalled)
{
  ASSERT_FALSE(agnocast::ok()) << "precondition: agnocast::init() has not been called";

  auto executor = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();

  std::atomic_bool spin_exited{false};
  std::thread spin_thread([&]() {
    executor->spin();
    spin_exited.store(true);
  });
  std::this_thread::sleep_for(250ms);

  ASSERT_EQ(kill(getpid(), SIGINT), 0);

  wait_until_or_fail(
    [&]() { return spin_exited.load(); }, 2s, "executor spin() did not stop after SIGINT.");

  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}
