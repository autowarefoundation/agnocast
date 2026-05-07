#pragma once

#include "agnocast/agnocast_epoll_event.hpp"

#include <rclcpp/callback_group.hpp>

#include <array>
#include <functional>
#include <memory>

namespace agnocast
{

using CallbackGroupValidator = std::function<bool(const rclcpp::CallbackGroup::SharedPtr &)>;

class EpollEventHandler
{
public:
  EpollEventHandler() = default;

  virtual ~EpollEventHandler() = default;

  EpollEventHandler(const EpollEventHandler &) = delete;
  EpollEventHandler & operator=(const EpollEventHandler &) = delete;

  EpollEventHandler(EpollEventHandler &&) = delete;
  EpollEventHandler & operator=(EpollEventHandler &&) = delete;

  [[nodiscard]] virtual EpollEventType get_type() const = 0;

  virtual void prepare_epoll(
    int epoll_fd, const CallbackGroupValidator & validate_callback_group) = 0;

  virtual void handle(EpollEventLocalID event_local_id) = 0;
};

// Shutdown event - only used by AgnocastOnlyExecutor
class ShutdownEventHandler : public EpollEventHandler
{
public:
  ShutdownEventHandler() = default;

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Shutdown; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override
  {
    (void)epoll_fd;
    (void)validate_callback_group;
  }

  void handle(EpollEventLocalID /*event_local_id*/) override {}
};

class DummyEventHandler : public EpollEventHandler
{
public:
  DummyEventHandler() = default;

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Dummy; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override
  {
    (void)epoll_fd;
    (void)validate_callback_group;
  }

  void handle(EpollEventLocalID event_local_id) override;
};

using EventHandlerArray =
  std::array<std::unique_ptr<EpollEventHandler>, static_cast<size_t>(EpollEventType::NrEventType)>;

class EpollManager final
{
public:
  explicit EpollManager(EventHandlerArray sources);
  ~EpollManager();
  EpollManager(const EpollManager &) = delete;
  EpollManager & operator=(const EpollManager &) = delete;
  EpollManager(EpollManager &&) = delete;
  EpollManager & operator=(EpollManager &&) = delete;

  [[nodiscard]] bool add_event(int fd, EpollEventType type, EpollEventLocalID local_id) const;

  void prepare_epoll(const CallbackGroupValidator & validate_callback_group);

  void wait_and_handle_epoll_event(int timeout_ms);

private:
  int epoll_fd_{-1};
  EventHandlerArray sources_{nullptr};
};

}  // namespace agnocast
