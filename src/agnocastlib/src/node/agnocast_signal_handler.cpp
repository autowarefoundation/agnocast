#include "agnocast_signal_handler.hpp"

#include "rclcpp/rclcpp.hpp"

#include <unistd.h>

#include <csignal>
#include <cstring>

namespace agnocast
{

namespace
{
rclcpp::Logger logger = rclcpp::get_logger("agnocast_signal_handler");
}

std::atomic<bool> SignalHandler::installed_{false};
std::mutex SignalHandler::eventfds_mutex_;
std::array<std::atomic<int>, SignalHandler::MAX_EXECUTORS_NUM> SignalHandler::eventfds_{};
std::atomic<size_t> SignalHandler::eventfd_count_{0};

void SignalHandler::install()
{
  if (installed_.exchange(true)) {
    return;
  }

  // Initialize eventfds array with -1 (empty marker)
  for (auto & fd : eventfds_) {
    fd.store(-1);
  }

  struct sigaction sa
  {
  };
  sigemptyset(&sa.sa_mask);
  sa.sa_handler = &SignalHandler::signal_handler;

  if (sigaction(SIGINT, &sa, nullptr) != 0) {
    RCLCPP_ERROR(logger, "Failed to install SIGINT handler: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (sigaction(SIGTERM, &sa, nullptr) != 0) {
    RCLCPP_ERROR(logger, "Failed to install SIGTERM handler: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void SignalHandler::register_shutdown_event(int eventfd)
{
  std::lock_guard<std::mutex> lock(eventfds_mutex_);
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
  std::lock_guard<std::mutex> lock(eventfds_mutex_);
  size_t count = eventfd_count_.load();
  for (size_t i = 0; i < count; ++i) {
    if (eventfds_[i].load() == eventfd) {
      eventfds_[i].store(-1);
      return;
    }
  }
}

void SignalHandler::signal_handler(int signum)
{
  (void)signum;
  notify_all_executors();
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
