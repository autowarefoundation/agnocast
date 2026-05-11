#pragma once

#include <cstdint>
#include <utility>

namespace agnocast
{

/**
 * @brief Categorizes the event types monitored by the Executor using epoll.
 *
 * Events monitored by epoll are identified by a 64-bit integer, which is a
 * combination of this event type and an event-specific local ID.
 *
 * @see EpollEventHandler
 */
enum class EpollEventType : uint32_t {
  Subscription = 0,
  Timer,
  Clock,
  Shutdown,
  NrEventType,

  Dummy = 0xFFFFFFFF,
};

/**
 * @brief Represents a unique local identifier specific to an event type.
 */
using EpollEventLocalID = uint32_t;

constexpr uint32_t EPOLL_DATA_TYPE_SHIFT = 32;
constexpr uint32_t EPOLL_DATA_ID_BITMASK = 0xFFFFFFFF;

inline uint64_t pack_epoll_data(EpollEventType type, EpollEventLocalID id)
{
  return (static_cast<uint64_t>(type) << EPOLL_DATA_TYPE_SHIFT) | id;
}

inline std::pair<EpollEventType, EpollEventLocalID> unpack_epoll_data(uint64_t data)
{
  auto type = static_cast<EpollEventType>(data >> EPOLL_DATA_TYPE_SHIFT);
  auto id = static_cast<uint32_t>(data & EPOLL_DATA_ID_BITMASK);
  return {type, id};
}

}  // namespace agnocast
