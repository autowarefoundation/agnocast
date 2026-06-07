#include "node/agnocast_signal_handler.hpp"

#include <gtest/gtest.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <thread>
#include <vector>

namespace
{

constexpr int TIMEOUT_MS_DEFAULT = 1000;

bool read_event_fd_with_timeout(int fd, int timeout_ms, uint64_t & value)
{
  const int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd < 0) {
    return false;
  }

  epoll_event event{};
  event.events = EPOLLIN;
  event.data.fd = fd;
  if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event) != 0) {
    close(epoll_fd);
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  bool read_succeeded = false;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    epoll_event triggered_event{};
    const int wait_result =
      epoll_wait(epoll_fd, &triggered_event, 1, static_cast<int>(remaining.count()));
    if (wait_result == 0) {
      break;
    }
    if (wait_result == -1) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }

    bool should_break = false;
    while (true) {
      const ssize_t bytes_read = read(fd, &value, sizeof(value));
      if (bytes_read == static_cast<ssize_t>(sizeof(value))) {
        read_succeeded = true;
        break;
      }
      if (bytes_read == -1 && errno == EINTR) {
        continue;
      }
      if (bytes_read == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      should_break = true;
      break;
    }
    if (should_break) {
      break;
    }

    if (read_succeeded) {
      break;
    }
  }

  close(epoll_fd);
  return read_succeeded;
}

bool event_fd_has_notification(int fd, int timeout_ms = 200)
{
  uint64_t value = 0;
  return read_event_fd_with_timeout(fd, timeout_ms, value);
}

struct sigaction get_current_sigaction(int signum)
{
  struct sigaction action
  {
  };
  EXPECT_EQ(sigaction(signum, nullptr, &action), 0);
  return action;
}

template <typename Pred>
bool wait_for_condition(Pred pred, int timeout_ms = TIMEOUT_MS_DEFAULT)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!pred() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  return pred();
}

static_assert(
  std::atomic<int>::is_always_lock_free,
  "std::atomic<int> is not lock-free on this target architecture!");

// Global atomics for signal argument capture in old-handler stub functions.
std::atomic<int> g_sa_handler_signum{-1};

void old_sa_handler_stub(int signum)
{
  g_sa_handler_signum.store(signum);
}

std::atomic<int> g_sa_sigaction_signum{-1};
std::atomic<int> g_sa_sigaction_si_signo{-1};
std::atomic<int> g_sa_sigaction_si_code{-1};
std::atomic<int> g_sa_sigaction_si_pid{-1};

void old_sa_sigaction_stub(int signum, siginfo_t * siginfo, void * /* context */)
{
  g_sa_sigaction_signum.store(signum);
  if (siginfo != nullptr) {
    g_sa_sigaction_si_signo.store(siginfo->si_signo);
    g_sa_sigaction_si_code.store(siginfo->si_code);
    g_sa_sigaction_si_pid.store(static_cast<int>(siginfo->si_pid));
  }
}

bool sigactions_equal(const struct sigaction & a, const struct sigaction & b)
{
  if ((a.sa_flags & SA_SIGINFO) != (b.sa_flags & SA_SIGINFO)) {
    return false;
  }
  if (a.sa_flags & SA_SIGINFO) {
    return a.sa_sigaction == b.sa_sigaction;
  }
  return a.sa_handler == b.sa_handler;
}

}  // namespace

class SignalHandlerTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    agnocast::SignalHandler::uninstall();
    for (int fd : owned_eventfds_) {
      close(fd);
    }
  }

  int create_event_fd()
  {
    const int fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (fd < 0) {
      throw std::runtime_error("failed to create eventfd");
    }
    owned_eventfds_.push_back(fd);
    return fd;
  }

  static void send_sigint() { ASSERT_EQ(kill(getpid(), SIGINT), 0); }
  static void send_sigterm() { ASSERT_EQ(kill(getpid(), SIGTERM), 0); }

private:
  std::vector<int> owned_eventfds_;
};

TEST_F(SignalHandlerTest, InstallAndUninstallRestoreSigintAndSigtermHandlers)
{
  const auto sigint_before = get_current_sigaction(SIGINT);
  const auto sigterm_before = get_current_sigaction(SIGTERM);

  agnocast::SignalHandler::install();

  const auto sigint_installed = get_current_sigaction(SIGINT);
  const auto sigterm_installed = get_current_sigaction(SIGTERM);
  EXPECT_FALSE(sigactions_equal(sigint_installed, sigint_before));
  EXPECT_FALSE(sigactions_equal(sigterm_installed, sigterm_before));

  agnocast::SignalHandler::uninstall();

  const auto sigint_after = get_current_sigaction(SIGINT);
  const auto sigterm_after = get_current_sigaction(SIGTERM);
  EXPECT_TRUE(sigactions_equal(sigint_after, sigint_before));
  EXPECT_TRUE(sigactions_equal(sigterm_after, sigterm_before));
}

TEST_F(SignalHandlerTest, InstallAndUninstallAreIdempotent)
{
  const auto sigint_before = get_current_sigaction(SIGINT);
  const auto sigterm_before = get_current_sigaction(SIGTERM);

  agnocast::SignalHandler::install();
  const auto sigint_after_first_install = get_current_sigaction(SIGINT);
  const auto sigterm_after_first_install = get_current_sigaction(SIGTERM);
  EXPECT_FALSE(sigactions_equal(sigint_after_first_install, sigint_before));
  EXPECT_FALSE(sigactions_equal(sigterm_after_first_install, sigterm_before));

  agnocast::SignalHandler::install();
  const auto sigint_after_second_install = get_current_sigaction(SIGINT);
  const auto sigterm_after_second_install = get_current_sigaction(SIGTERM);
  EXPECT_TRUE(sigactions_equal(sigint_after_second_install, sigint_after_first_install));
  EXPECT_TRUE(sigactions_equal(sigterm_after_second_install, sigterm_after_first_install));

  agnocast::SignalHandler::uninstall();
  const auto sigint_after_first_uninstall = get_current_sigaction(SIGINT);
  const auto sigterm_after_first_uninstall = get_current_sigaction(SIGTERM);
  EXPECT_TRUE(sigactions_equal(sigint_after_first_uninstall, sigint_before));
  EXPECT_TRUE(sigactions_equal(sigterm_after_first_uninstall, sigterm_before));

  agnocast::SignalHandler::uninstall();
  const auto sigint_after_second_uninstall = get_current_sigaction(SIGINT);
  const auto sigterm_after_second_uninstall = get_current_sigaction(SIGTERM);
  EXPECT_TRUE(sigactions_equal(sigint_after_second_uninstall, sigint_after_first_uninstall));
  EXPECT_TRUE(sigactions_equal(sigterm_after_second_uninstall, sigterm_after_first_uninstall));
}

TEST_F(SignalHandlerTest, SigintNotifiesRegisteredEventfdViaWorkerThread)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  send_sigint();

  uint64_t value = 0;
  ASSERT_TRUE(read_event_fd_with_timeout(fd, 1000, value));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, SigintNotifiesRegisteredEventfdViaWorkerThreadRepeated)
{
  constexpr int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    agnocast::SignalHandler::install();

    const int fd = create_event_fd();
    EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

    send_sigint();

    uint64_t value = 0;
    ASSERT_TRUE(read_event_fd_with_timeout(fd, 1000, value));

    agnocast::SignalHandler::uninstall();
  }
}

TEST_F(SignalHandlerTest, SigtermNotifiesRegisteredEventfdViaWorkerThread)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  send_sigterm();

  uint64_t value = 0;
  ASSERT_TRUE(read_event_fd_with_timeout(fd, 1000, value));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, SigintNotifiesAllRegisteredEventfds)
{
  agnocast::SignalHandler::install();

  const int fd1 = create_event_fd();
  const int fd2 = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd1));
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd2));

  send_sigint();

  uint64_t value1 = 0;
  uint64_t value2 = 0;
  ASSERT_TRUE(read_event_fd_with_timeout(fd1, 1000, value1));
  ASSERT_TRUE(read_event_fd_with_timeout(fd2, 1000, value2));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, DuplicateEventfdRegistrationIsIgnored)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  send_sigint();
  EXPECT_TRUE(event_fd_has_notification(fd));

  agnocast::SignalHandler::unregister_shutdown_event(fd);

  send_sigint();
  EXPECT_FALSE(event_fd_has_notification(fd));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, UnregisteredEventfdIsNotNotifiedBySigint)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));
  agnocast::SignalHandler::unregister_shutdown_event(fd);

  send_sigint();

  EXPECT_FALSE(event_fd_has_notification(fd));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, SigintWakesUpWaitingThread)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  std::atomic<bool> notified{false};
  std::thread waiter([&]() {
    uint64_t value = 0;
    notified.store(read_event_fd_with_timeout(fd, TIMEOUT_MS_DEFAULT, value));
  });

  send_sigint();
  waiter.join();

  EXPECT_TRUE(notified.load());

  agnocast::SignalHandler::uninstall();
}

// Background: When using signalfd, all threads in the process must block the signal via
// pthread_sigmask so that the signal is routed to the signalfd rather than delivered directly.
// This requires either calling pthread_sigmask(SIG_BLOCK, ...) in every thread, or calling it in
// the main thread before spawning any threads (child threads inherit the signal mask).
// A library cannot impose either requirement on its callers, so this test verifies that
// the signal handler works correctly even when threads are spawned before install() is called.
// This test is not strictly necessary for the current sigaction + eventfd implementation,
// but is kept as a regression guard against future internal implementation changes.
TEST_F(SignalHandlerTest, SigintNotifiesEventfdEvenWhenThreadsSpawnedBeforeInstall)
{
  // Spawn several threads before calling SignalHandler::install()
  constexpr int pre_spawn_thread_count = 30;
  std::atomic<bool> keep_running{true};
  std::vector<std::thread> pre_spawn_threads;
  pre_spawn_threads.reserve(pre_spawn_thread_count);
  for (int i = 0; i < pre_spawn_thread_count; ++i) {
    pre_spawn_threads.emplace_back([&keep_running]() {
      while (keep_running.load()) {
        std::this_thread::yield();
      }
    });
  }

  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  send_sigint();

  uint64_t value = 0;
  ASSERT_TRUE(read_event_fd_with_timeout(fd, TIMEOUT_MS_DEFAULT, value));

  agnocast::SignalHandler::uninstall();

  keep_running.store(false);
  for (auto & t : pre_spawn_threads) {
    t.join();
  }
}

TEST_F(SignalHandlerTest, ManyEventfdsAreNotifiedBySigint)
{
  agnocast::SignalHandler::install();

  constexpr int eventfd_count = 200;
  std::vector<int> fds;
  fds.reserve(eventfd_count);
  for (int i = 0; i < eventfd_count; ++i) {
    const int fd = create_event_fd();
    fds.push_back(fd);
    EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));
  }

  send_sigint();

  for (int fd : fds) {
    uint64_t value = 0;
    ASSERT_TRUE(read_event_fd_with_timeout(fd, TIMEOUT_MS_DEFAULT, value));
  }

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, RegisterShutdownEventFailsWhenNotInstalled)
{
  const int fd1 = create_event_fd();
  EXPECT_FALSE(agnocast::SignalHandler::register_shutdown_event(fd1));

  agnocast::SignalHandler::install();
  agnocast::SignalHandler::uninstall();

  const int fd2 = create_event_fd();
  EXPECT_FALSE(agnocast::SignalHandler::register_shutdown_event(fd2));
}

TEST_F(SignalHandlerTest, UnregisterShutdownEventIsIdempotent)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  agnocast::SignalHandler::unregister_shutdown_event(fd);
  agnocast::SignalHandler::unregister_shutdown_event(fd);

  send_sigint();
  EXPECT_FALSE(event_fd_has_notification(fd));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, RegisteredEventfdsAreClearedAfterUninstallAndReinstall)
{
  agnocast::SignalHandler::install();

  const int old_fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(old_fd));

  agnocast::SignalHandler::uninstall();

  agnocast::SignalHandler::install();

  const int new_fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(new_fd));

  // old_fd must not be notified because registrations are cleared by uninstall().
  send_sigint();
  EXPECT_FALSE(event_fd_has_notification(old_fd));
  EXPECT_TRUE(event_fd_has_notification(new_fd));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, UnregisterShutdownEventIsSafeAfterUninstall)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  agnocast::SignalHandler::uninstall();

  // This should not crash or have any observable effect.
  agnocast::SignalHandler::unregister_shutdown_event(fd);
}

TEST_F(SignalHandlerTest, NotifyAllExecutorsNotifiesRegisteredEventfdsWhenInstalled)
{
  agnocast::SignalHandler::install();

  const int fd1 = create_event_fd();
  const int fd2 = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd1));
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd2));

  agnocast::SignalHandler::notify_all_executors();

  uint64_t value1 = 0;
  uint64_t value2 = 0;
  EXPECT_TRUE(read_event_fd_with_timeout(fd1, TIMEOUT_MS_DEFAULT, value1));
  EXPECT_TRUE(read_event_fd_with_timeout(fd2, TIMEOUT_MS_DEFAULT, value2));

  agnocast::SignalHandler::uninstall();
}

TEST_F(SignalHandlerTest, NotifyAllExecutorsFailsWhenNotInstalled)
{
  const int fd = create_event_fd();
  EXPECT_FALSE(agnocast::SignalHandler::register_shutdown_event(fd));
  agnocast::SignalHandler::notify_all_executors();
  EXPECT_FALSE(event_fd_has_notification(fd));
}

TEST_F(SignalHandlerTest, NotifyAllExecutorsFailsAfterUninstall)
{
  agnocast::SignalHandler::install();

  const int fd = create_event_fd();
  EXPECT_TRUE(agnocast::SignalHandler::register_shutdown_event(fd));

  agnocast::SignalHandler::uninstall();

  agnocast::SignalHandler::notify_all_executors();
  EXPECT_FALSE(event_fd_has_notification(fd));
}

TEST_F(SignalHandlerTest, OldSaHandlerIsCalledWithCorrectSigintArgument)
{
  g_sa_handler_signum.store(-1);

  struct sigaction old_before
  {
  };
  struct sigaction sa
  {
  };
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sa.sa_handler = old_sa_handler_stub;
  ASSERT_EQ(sigaction(SIGINT, &sa, &old_before), 0);

  agnocast::SignalHandler::install();
  send_sigint();

  EXPECT_TRUE(wait_for_condition([] { return g_sa_handler_signum.load() != -1; }));
  EXPECT_EQ(g_sa_handler_signum.load(), SIGINT);

  agnocast::SignalHandler::uninstall();
  ASSERT_EQ(sigaction(SIGINT, &old_before, nullptr), 0);
}

TEST_F(SignalHandlerTest, OldSaHandlerIsCalledWithCorrectSigtermArgument)
{
  g_sa_handler_signum.store(-1);

  struct sigaction old_before
  {
  };
  struct sigaction sa
  {
  };
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sa.sa_handler = old_sa_handler_stub;
  ASSERT_EQ(sigaction(SIGTERM, &sa, &old_before), 0);

  agnocast::SignalHandler::install();
  send_sigterm();

  EXPECT_TRUE(wait_for_condition([] { return g_sa_handler_signum.load() != -1; }));
  EXPECT_EQ(g_sa_handler_signum.load(), SIGTERM);

  agnocast::SignalHandler::uninstall();
  ASSERT_EQ(sigaction(SIGTERM, &old_before, nullptr), 0);
}

TEST_F(SignalHandlerTest, OldSaSigactionHandlerIsCalledWithCorrectArguments)
{
  g_sa_sigaction_signum.store(-1);
  g_sa_sigaction_si_signo.store(-1);
  g_sa_sigaction_si_code.store(-1);
  g_sa_sigaction_si_pid.store(-1);

  struct sigaction old_before
  {
  };
  struct sigaction sa
  {
  };
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = old_sa_sigaction_stub;
  ASSERT_EQ(sigaction(SIGINT, &sa, &old_before), 0);

  agnocast::SignalHandler::install();
  send_sigint();

  EXPECT_TRUE(wait_for_condition([] { return g_sa_sigaction_signum.load() != -1; }));
  EXPECT_EQ(g_sa_sigaction_signum.load(), SIGINT);
  EXPECT_EQ(g_sa_sigaction_si_signo.load(), SIGINT);
  EXPECT_EQ(g_sa_sigaction_si_code.load(), SI_USER);
  EXPECT_EQ(g_sa_sigaction_si_pid.load(), static_cast<int>(getpid()));

  agnocast::SignalHandler::uninstall();
  ASSERT_EQ(sigaction(SIGINT, &old_before, nullptr), 0);
}
