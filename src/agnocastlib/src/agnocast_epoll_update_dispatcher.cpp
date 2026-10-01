#include "agnocast/agnocast_epoll_update_dispatcher.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <sys/eventfd.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace agnocast
{

namespace
{

void signal_notify_fd(int notify_fd)
{
  if (notify_fd < 0) {
    return;
  }
  const uint64_t one = 1;
  const ssize_t ret = write(notify_fd, &one, sizeof(one));
  // EAGAIN: counter already saturated; still readable, so wakeup is already pending.
  if (ret < 0 && errno != EAGAIN) {
    RCLCPP_WARN(logger, "Failed to write to epoll update notify_fd: %s", strerror(errno));
  }
}

void drain_notify_fd(int notify_fd)
{
  if (notify_fd < 0) {
    return;
  }
  uint64_t value = 0;
  while (read(notify_fd, &value, sizeof(value)) >= 0) {
  }
}

}  // namespace

EpollUpdateTracker EpollUpdateDispatcher::register_tracker()
{
  uint64_t new_id = next_tracker_id_.fetch_add(1, std::memory_order_relaxed);

  auto context = std::make_shared<TrackerContext>();
  context->notify_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (context->notify_fd < 0) {
    throw std::runtime_error(
      std::string("eventfd creation failed for EpollUpdateTracker: ") + strerror(errno));
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    trackers_.emplace(new_id, context);
  }

  return {new_id, context};
}

void EpollUpdateDispatcher::request_update_all()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [id, context] : trackers_) {
    context->need_update.store(true, std::memory_order_release);
    signal_notify_fd(context->notify_fd);
  }
}

void EpollUpdateDispatcher::request_update(uint64_t tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = trackers_.find(tracker_id);
  if (it != trackers_.end()) {
    it->second->need_update.store(true, std::memory_order_release);
    signal_notify_fd(it->second->notify_fd);
  }
}

void EpollUpdateDispatcher::unregister_tracker(uint64_t tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.erase(tracker_id);
}

EpollUpdateTracker::~EpollUpdateTracker()
{
  if (id_ != 0) {
    EpollUpdateDispatcher::get_instance().unregister_tracker(id_);
  }
  if (context_ && context_->notify_fd >= 0) {
    close(context_->notify_fd);
    context_->notify_fd = -1;
  }
}

bool EpollUpdateTracker::take_update_request()
{
  if (!context_) {
    return false;
  }
  const bool needed = context_->need_update.exchange(false, std::memory_order_acquire);
  drain_notify_fd(context_->notify_fd);
  return needed;
}

int EpollUpdateTracker::notify_fd() const
{
  if (!context_) {
    return -1;
  }
  return context_->notify_fd;
}

}  // namespace agnocast
