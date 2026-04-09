#pragma once

#include "sys/epoll.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace agnocast
{

class EpollUpdateTracker;

struct TrackerContext
{
  std::atomic<bool> need_update{true};
};

class EpollUpdateDispatcher
{
public:
  static EpollUpdateDispatcher & get_instance()
  {
    static EpollUpdateDispatcher instance;
    return instance;
  }

  void notify_all();

  EpollUpdateTracker create_tracker();

private:
  EpollUpdateDispatcher() = default;
  friend class EpollUpdateTracker;

  void unregister(int tracker_id);

  std::atomic<int> next_tracker_id_{1};

  std::mutex mutex_;
  std::unordered_map<int, std::shared_ptr<TrackerContext>> trackers_;
};

class EpollUpdateTracker
{
public:
  EpollUpdateTracker(const EpollUpdateTracker &) = delete;
  EpollUpdateTracker & operator=(const EpollUpdateTracker &) = delete;

  EpollUpdateTracker(EpollUpdateTracker && other) noexcept;
  EpollUpdateTracker & operator=(EpollUpdateTracker && other) noexcept;

  ~EpollUpdateTracker();

  [[nodiscard]] bool need_update() const;

private:
  friend class EpollUpdateDispatcher;

  EpollUpdateTracker(int tracker_id, std::shared_ptr<TrackerContext> context)
  : id_(tracker_id), context_(std::move(context))
  {
  }

  int id_;
  std::shared_ptr<TrackerContext> context_;
};

}  // namespace agnocast
