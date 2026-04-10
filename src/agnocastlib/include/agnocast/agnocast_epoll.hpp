#pragma once

#include "agnocast/agnocast_epoll_event.hpp"

#include <rclcpp/callback_group.hpp>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace agnocast
{

using CallbackGroupValidator = std::function<bool(const rclcpp::CallbackGroup::SharedPtr &)>;

class EpollEventSource
{
public:
  EpollEventSource() = default;

  virtual ~EpollEventSource() = default;

  EpollEventSource(const EpollEventSource &) = delete;
  EpollEventSource & operator=(const EpollEventSource &) = delete;

  EpollEventSource(EpollEventSource &&) = delete;
  EpollEventSource & operator=(EpollEventSource &&) = delete;

  [[nodiscard]] virtual EpollEventType get_type() const = 0;

  virtual void prepare_epoll(
    Epoll & epoll, const CallbackGroupValidator & validate_callback_group) = 0;

  virtual bool handle(EpollEventLocalID event_local_id) = 0;
};

// Shutdown event - only used by AgnocastOnlyExecutor
class ShutdownEventSource : public EpollEventSource
{
public:
  ShutdownEventSource() = default;

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Shutdown; }

  void prepare_epoll(Epoll & epoll, const CallbackGroupValidator & validate_callback_group) override
  {
    (void)epoll;
    (void)validate_callback_group;
  }

  bool handle(EpollEventLocalID /*event_local_id*/) override { return true; }
};

using EventSourceArray =
  std::array<std::unique_ptr<EpollEventSource>, static_cast<size_t>(EpollEventType::NrEventType)>;

class EpollManager
{
public:
  explicit EpollManager(EventSourceArray sources);

  int add_event(int fd, EpollEventType type, EpollEventLocalID local_id)
  {
    return epoll_.add_source(fd, type, local_id);
  }

  void prepare_epoll(const CallbackGroupValidator & validate_callback_group);

  /// @return true if shutdown event detected, false otherwise
  bool wait_and_handle_epoll_event(int timeout_ms);

private:
  Epoll epoll_;
  EventSourceArray sources_{nullptr};
};

}  // namespace agnocast
