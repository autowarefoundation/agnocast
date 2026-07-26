#include "agnocast/agnocast_epoll_update_dispatcher.hpp"

#include <atomic>
#include <cstdint>

namespace agnocast
{

EpollUpdateTracker EpollUpdateDispatcher::register_tracker()
{
  uint64_t new_id = next_tracker_id_.fetch_add(1, std::memory_order_relaxed);

  auto context = std::make_shared<TrackerContext>();

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
  }
}

void EpollUpdateDispatcher::request_update(uint64_t tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = trackers_.find(tracker_id);
  if (it != trackers_.end()) {
    it->second->need_update.store(true, std::memory_order_release);
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
}

bool EpollUpdateTracker::take_update_request()
{
  if (!context_) {
    return false;
  }
  return context_->need_update.exchange(false, std::memory_order_acquire);
}

}  // namespace agnocast
