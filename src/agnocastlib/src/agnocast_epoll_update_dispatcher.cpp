#include "agnocast/agnocast_epoll_update_dispatcher.hpp"

#include <atomic>

namespace agnocast
{

EpollUpdateTracker EpollUpdateDispatcher::create_tracker()
{
  int new_id = next_tracker_id_.fetch_add(1, std::memory_order_relaxed);

  auto context = std::make_shared<TrackerContext>();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    trackers_.emplace(new_id, context);
  }

  return {new_id, context};
}

void EpollUpdateDispatcher::notify_all()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [id, context] : trackers_) {
    context->need_update.store(true, std::memory_order_release);
  }
}

void EpollUpdateDispatcher::notify(int tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = trackers_.find(tracker_id);
  if (it != trackers_.end()) {
    it->second->need_update.store(true, std::memory_order_release);
  }
}

void EpollUpdateDispatcher::unregister(int tracker_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.erase(tracker_id);
}

EpollUpdateTracker::EpollUpdateTracker(EpollUpdateTracker && other) noexcept
: id_(other.id_), context_(std::move(other.context_))
{
  other.id_ = 0;
}

EpollUpdateTracker & EpollUpdateTracker::operator=(EpollUpdateTracker && other) noexcept
{
  if (this != &other) {
    if (id_ != 0) {
      EpollUpdateDispatcher::get_instance().unregister(id_);
    }
    id_ = other.id_;
    context_ = std::move(other.context_);

    other.id_ = 0;
  }
  return *this;
}

EpollUpdateTracker::~EpollUpdateTracker()
{
  if (id_ != 0) {
    EpollUpdateDispatcher::get_instance().unregister(id_);
  }
}

bool EpollUpdateTracker::need_update() const
{
  if (!context_) {
    return false;
  }
  return context_->need_update.exchange(false, std::memory_order_acquire);
}

}  // namespace agnocast
