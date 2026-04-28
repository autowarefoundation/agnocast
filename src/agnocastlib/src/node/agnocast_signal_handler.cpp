#include "agnocast_signal_handler.hpp"

#include "rclcpp/rclcpp.hpp"

#include <sys/eventfd.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <thread>

namespace agnocast
{

namespace
{
rclcpp::Logger logger = rclcpp::get_logger("agnocast_signal_handler");
}

// Check async-signal safety at compile time.
// std::atomic<T> may use internal locks on some platforms.
// Signal handlers must not use locks.
// So we require std::atomic<int> to be lock-free on this target.
static_assert(
  std::atomic<int>::is_always_lock_free,
  "std::atomic<int> is not lock-free on this target architecture!");

static_assert(
  std::atomic<bool>::is_always_lock_free,
  "std::atomic<bool> is not lock-free on this target architecture!");

SignalHandler::State SignalHandler::state_{SignalHandler::State::NotInstalled};
std::mutex SignalHandler::mutex_;
std::set<int> SignalHandler::eventfds_{};
std::atomic<bool> SignalHandler::stop_requested_{false};
std::atomic<bool> SignalHandler::signal_received_{false};
std::atomic<int> SignalHandler::signal_eventfd_{-1};
// Keep this as a raw pointer on purpose.
// This prevents thread destruction during process teardown.
// Destroying an unjoined std::thread calls std::terminate (abort).
std::thread * SignalHandler::signal_thread_ = nullptr;
std::atomic<int> SignalHandler::handler_inflight_count_{0};
struct sigaction SignalHandler::old_sigint_action_
{
};
struct sigaction SignalHandler::old_sigterm_action_
{
};

void SignalHandler::install()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::NotInstalled) {
    return;
  }

  eventfds_.clear();

  const int signal_eventfd = eventfd(0, EFD_CLOEXEC);
  if (signal_eventfd < 0) {
    RCLCPP_ERROR(logger, "Failed to create signal eventfd: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }
  signal_eventfd_.store(signal_eventfd);

  stop_requested_.store(false);
  signal_received_.store(false);

  signal_thread_ = new std::thread(&SignalHandler::signal_processing_loop);  // NOLINT

  struct sigaction sa
  {
  };
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sa.sa_handler = &SignalHandler::signal_handler;

  if (sigaction(SIGINT, &sa, &old_sigint_action_) != 0) {
    RCLCPP_ERROR(logger, "Failed to install SIGINT handler: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (sigaction(SIGTERM, &sa, &old_sigterm_action_) != 0) {
    RCLCPP_ERROR(logger, "Failed to install SIGTERM handler: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  state_ = State::Installed;
}

void SignalHandler::uninstall()
{
  std::thread * signal_thread = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Installed) {
      return;
    }
    state_ = State::Uninstalling;

    bool restore_failed = false;
    if (sigaction(SIGINT, &old_sigint_action_, nullptr) != 0) {
      RCLCPP_ERROR(logger, "Failed to restore SIGINT handler: %s", strerror(errno));
      restore_failed = true;
    }
    if (sigaction(SIGTERM, &old_sigterm_action_, nullptr) != 0) {
      RCLCPP_ERROR(logger, "Failed to restore SIGTERM handler: %s", strerror(errno));
      restore_failed = true;
    }
    if (restore_failed) {
      RCLCPP_ERROR(
        logger,
        "Failed to restore previous signal handlers; aborting uninstall to avoid inconsistent "
        "SIGINT/SIGTERM handling");
      exit(EXIT_FAILURE);
    }

    // shutdown signal processing loop
    stop_requested_.store(true);
    notify_signal_eventfd();

    signal_thread = signal_thread_;
    signal_thread_ = nullptr;
  }

  // wait for signal_thread_ to finish
  if (signal_thread == nullptr) {
    RCLCPP_ERROR(logger, "Signal handler thread was not running");
    exit(EXIT_FAILURE);
  }
  // join signal_thread_
  if (signal_thread->joinable()) {
    signal_thread->join();
  }
  delete signal_thread;  // NOLINT

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Invalidate signal_eventfd_ atomically before closing it.
    // The OS may have dispatched a signal to our handler before sigaction was
    // restored but the handler's first statement (handler_inflight_count_.fetch_add)
    // had not yet executed ("tiny gap").  Setting signal_eventfd_ to -1 here ensures
    // any such late-starting handler will load -1 in notify_signal_eventfd() and
    // skip the write, preventing a write to a closed—and potentially reused—fd.
    int signal_eventfd = signal_eventfd_.exchange(-1);
    if (signal_eventfd == -1) {
      RCLCPP_ERROR(logger, "Signal eventfd was already closed");
      exit(EXIT_FAILURE);
    }

    // Wait for handlers that were already in flight before or during the exchange
    // above. Those handlers may have already loaded the valid fd before it was
    // invalidated and must complete their write before we close it.
    while (handler_inflight_count_.load() > 0) {
      std::this_thread::yield();
    }

    if (close(signal_eventfd) != 0) {
      RCLCPP_ERROR(logger, "Failed to close signal eventfd: %s", strerror(errno));
      exit(EXIT_FAILURE);
    }

    state_ = State::NotInstalled;
  }
}

bool SignalHandler::register_shutdown_event(int eventfd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Installed) {
    RCLCPP_WARN(
      logger, "Cannot register shutdown eventfd %d: signal handler is not installed", eventfd);
    return false;
  }

  eventfds_.insert(eventfd);
  return true;
}

void SignalHandler::unregister_shutdown_event(int eventfd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == State::NotInstalled) {
    return;
  }

  eventfds_.erase(eventfd);
}

void SignalHandler::signal_processing_loop()
{
  while (true) {
    if (signal_received_.exchange(false)) {
      notify_all_executors();
    }

    if (stop_requested_.load()) {
      break;
    }

    wait_for_signal_eventfd();
  }
}

void SignalHandler::notify_signal_eventfd()
{
  const int fd = signal_eventfd_.load();
  if (fd != -1) {
    while (true) {
      uint64_t val = 1;
      auto ret = write(fd, &val, sizeof(val));
      if (ret == -1 && errno == EINTR) {
        continue;
      }
      break;
    }
  }
}

void SignalHandler::wait_for_signal_eventfd()
{
  const int fd = signal_eventfd_.load();
  if (fd != -1) {
    uint64_t count = 0;
    while (true) {
      const auto ret = read(fd, &count, sizeof(count));
      if (ret == static_cast<ssize_t>(sizeof(count))) {
        break;
      }
      if (ret == -1 && errno == EINTR) {
        continue;
      }
      if (ret == -1) {
        RCLCPP_ERROR(logger, "Failed to read signal eventfd: %s", std::strerror(errno));
      } else {
        RCLCPP_ERROR(
          logger, "Short read from signal eventfd: got %zd bytes, expected %zu", ret,
          sizeof(count));
      }
      break;
    }
  }
}

void SignalHandler::signal_handler(int signum)
{
  (void)signum;

  int saved_errno = errno;

  handler_inflight_count_.fetch_add(1);
  signal_received_.store(true);
  notify_signal_eventfd();
  handler_inflight_count_.fetch_sub(1);

  errno = saved_errno;
}

void SignalHandler::notify_all_executors()
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t val = 1;
  for (auto it = eventfds_.begin(); it != eventfds_.end();) {
    const int fd = *it;
    bool erase_fd = false;
    while (true) {
      const auto ret = write(fd, &val, sizeof(val));
      if (ret != -1) {
        break;
      }
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(logger, "Failed to notify shutdown eventfd %d: %s", fd, std::strerror(errno));
      if (errno == EBADF || errno == EINVAL) {
        erase_fd = true;
      }
      break;
    }

    if (erase_fd) {
      it = eventfds_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace agnocast
