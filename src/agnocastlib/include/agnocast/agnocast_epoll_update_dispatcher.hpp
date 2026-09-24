#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace agnocast
{

class EpollUpdateTracker;

struct TrackerContext
{
  std::atomic<bool> need_update{true};
  // eventfd used to wake Executor threads blocked in epoll_wait(). Owned by EpollUpdateTracker.
  int notify_fd{-1};
};

class EpollUpdateDispatcher
{
public:
  static EpollUpdateDispatcher & get_instance()
  {
    static EpollUpdateDispatcher instance;
    return instance;
  }

  void request_update_all();

  void request_update(uint64_t tracker_id);

  EpollUpdateTracker register_tracker();

private:
  EpollUpdateDispatcher() = default;
  friend class EpollUpdateTracker;

  void unregister_tracker(uint64_t tracker_id);

  std::atomic<uint64_t> next_tracker_id_{1};

  std::mutex mutex_;
  std::unordered_map<uint64_t, std::shared_ptr<TrackerContext>> trackers_;
};

class EpollUpdateTracker
{
public:
  EpollUpdateTracker(const EpollUpdateTracker &) = delete;
  EpollUpdateTracker & operator=(const EpollUpdateTracker &) = delete;
  EpollUpdateTracker(EpollUpdateTracker && other) = delete;
  EpollUpdateTracker & operator=(EpollUpdateTracker && other) = delete;
  ~EpollUpdateTracker();

  /**
   * @brief Consume a pending update request and drain the notify eventfd.
   *
   * Draining is required so a level-triggered epoll_wait does not spin after wakeup.
   */
  [[nodiscard]] bool take_update_request();

  [[nodiscard]] uint64_t id() const { return id_; }

  /// eventfd that becomes readable when an update is requested. Register with EpollManager.
  [[nodiscard]] int notify_fd() const;

private:
  friend class EpollUpdateDispatcher;

  EpollUpdateTracker(uint64_t tracker_id, std::shared_ptr<TrackerContext> context)
  : id_(tracker_id), context_(std::move(context))
  {
  }

  uint64_t id_;
  std::shared_ptr<TrackerContext> context_;
};

}  // namespace agnocast
