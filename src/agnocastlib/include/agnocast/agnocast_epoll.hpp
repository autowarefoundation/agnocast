#pragma once

#include "agnocast/agnocast_epoll_event.hpp"

#include <rclcpp/callback_group.hpp>

#include <array>
#include <functional>
#include <memory>
#include <unordered_map>

namespace agnocast
{

using CallbackGroupValidator = std::function<bool(const rclcpp::CallbackGroup::SharedPtr &)>;

/**
 * @brief Records which fds an EpollEventHandler has added to its epoll instance, keyed by the
 * local ID of the entity that owns each fd.
 *
 * Every Executor owns its own handlers, so a registry describes the registrations of one epoll
 * instance. In prepare_epoll(), a handler calls sync() for every existing entity and then
 * forget_unless() to drop the entities that no longer exist.
 */
class EpollRegistry
{
public:
  /**
   * @brief Adds or removes the registration of an entity so that it matches should_monitor.
   *
   * A registration whose fd differs from the given fd is forgotten without EPOLL_CTL_DEL: that
   * fd has been closed, which already removed it from epoll, and its number may have been reused
   * by another entity. An entity that closes its fd must therefore set it to -1 rather than
   * replace it with a new one, since the new fd may get the same number and be mistaken for the
   * registered one.
   *
   * @param epoll_fd The file descriptor for the epoll instance.
   * @param type The category of the event.
   * @param local_id The local ID of the entity.
   * @param fd The entity's current fd, or -1 if it has none. Must be open while this runs.
   * @param should_monitor Whether this epoll instance should monitor the entity.
   *
   * @return true if the entity was newly added to epoll.
   */
  bool sync(
    int epoll_fd, EpollEventType type, EpollEventLocalID local_id, int fd, bool should_monitor);

  /**
   * @brief Forgets the registrations of entities for which is_alive returns false, without
   * EPOLL_CTL_DEL. Their owners close their fds, which removes them from epoll.
   */
  template <typename IsAlive>
  void forget_unless(IsAlive is_alive)
  {
    for (auto it = fds_.begin(); it != fds_.end();) {
      if (is_alive(it->first)) {
        ++it;
      } else {
        it = fds_.erase(it);
      }
    }
  }

private:
  std::unordered_map<EpollEventLocalID, int> fds_;
};

/**
 * @brief Abstract base class implemented for each type of event (e.g., Timer, Subscription,
 * Shutdown events). Please refer to EpollEventType for details on event classification. Derived
 * classes inheriting from this class must implement the processing logic specific to each event.
 */
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

  /**
   * @brief Brings the events of this type monitored by epoll in line with the current entities:
   * adds the entities newly under this Executor's management and removes the ones no longer under
   * it.
   *
   * @param epoll_fd The file descriptor for the epoll instance.
   * @param validate_callback_group A function that takes a callback group and
   * determines whether the entities belonging to that group are under
   * its management.
   */
  virtual void prepare_epoll(
    int epoll_fd, const CallbackGroupValidator & validate_callback_group) = 0;

  /**
   * @brief Invoked when an event belonging to this event type occurs.
   *
   * @param event_local_id The local ID used to identify the specific event.
   */
  virtual void handle(EpollEventLocalID event_local_id) = 0;
};

/**
 * @brief Shutdown handler used by AgnocastOnlyExecutor to receive shutdown notifications via an
 * eventfd.
 */
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

/**
 * @brief Dummy handler used to fill unused slots in the EventHandlerArray.
 * Logs a warning if an event notification is unexpectedly received.
 * Since EpollManager requires every slot in the EventHandlerArray to be populated with a valid
 * handler, any slot corresponding to an unused event must be filled with a DummyEventHandler.
 */
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

/**
 * @brief A facade class that manages event waiting via epoll, primarily used by the Executor.
 */
class EpollManager final
{
public:
  /**
   * @param sources An array of event handlers mapped by event type.
   * Every slot in this array must contain a valid handler instance.
   */
  explicit EpollManager(EventHandlerArray sources);
  ~EpollManager();
  EpollManager(const EpollManager &) = delete;
  EpollManager & operator=(const EpollManager &) = delete;
  EpollManager(EpollManager &&) = delete;
  EpollManager & operator=(EpollManager &&) = delete;

  /**
   * @brief Directly adds a new event to be monitored by epoll.
   *
   * @param fd The file descriptor associated with the event.
   * @param type The category of the event.
   * @param local_id The unique local ID specific to this event.
   *
   * @return true on success, or false on failure.
   */
  [[nodiscard]] bool add_event(int fd, EpollEventType type, EpollEventLocalID local_id) const;

  /**
   * @brief Updates the set of events monitored by epoll (e.g., adding or removing events).
   *
   * This function works together with EpollUpdateTracker. When the Executor is
   * notified that the target events have changed, it calls this function.
   *
   * @param validate_callback_group A function that takes a callback group and
   * determines whether the entities belonging to that group are under its management.
   */
  void prepare_epoll(const CallbackGroupValidator & validate_callback_group);

  /**
   * @brief Waits for events to occur and executes their corresponding handlers.
   *
   * @param timeout_ms The maximum time to wait in milliseconds.
   */
  void wait_and_handle_epoll_event(int timeout_ms);

private:
  int epoll_fd_{-1};
  EventHandlerArray sources_{nullptr};
};

}  // namespace agnocast
