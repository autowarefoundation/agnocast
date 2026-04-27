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

bool SignalHandler::installed_{false};
std::mutex SignalHandler::mutex_;
std::array<std::atomic<int>, SignalHandler::MAX_EXECUTORS_NUM> SignalHandler::eventfds_{};
std::atomic<size_t> SignalHandler::eventfd_count_{0};
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
  if (installed_) {
    return;
  }

  // Initialize eventfds array with -1 (empty marker)
  for (auto & fd : eventfds_) {
    fd.store(-1);
  }
  eventfd_count_.store(0);

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

  installed_ = true;
}

void SignalHandler::uninstall()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!installed_) {
    return;
  }

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

  if (signal_thread_ == nullptr) {
    RCLCPP_ERROR(logger, "Signal handler thread was not running");
    exit(EXIT_FAILURE);
  }

  // join signal_thread_
  if (signal_thread_->joinable()) {
    signal_thread_->join();
  }
  delete signal_thread_;  // NOLINT
  signal_thread_ = nullptr;

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

  installed_ = false;
}

void SignalHandler::register_shutdown_event(int eventfd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!installed_) {
    return;
  }

  size_t count = eventfd_count_.load();
  for (size_t i = 0; i < count; ++i) {
    if (eventfds_[i].load() == -1) {
      eventfds_[i].store(eventfd);
      return;
    }
  }

  if (count >= MAX_EXECUTORS_NUM) {
    RCLCPP_ERROR(logger, "Maximum number of executors (%zu) exceeded", MAX_EXECUTORS_NUM);
    return;
  }
  eventfds_[count].store(eventfd);
  eventfd_count_.store(count + 1);
}

void SignalHandler::unregister_shutdown_event(int eventfd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!installed_) {
    return;
  }

  size_t count = eventfd_count_.load();
  for (size_t i = 0; i < count; ++i) {
    if (eventfds_[i].load() == eventfd) {
      eventfds_[i].store(-1);
      return;
    }
  }
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
  uint64_t val = 1;
  for (size_t i = 0; i < MAX_EXECUTORS_NUM; ++i) {
    int fd = eventfds_[i].load();
    if (fd != -1) {
      [[maybe_unused]] auto ret = write(fd, &val, sizeof(val));
    }
  }
}

}  // namespace agnocast
