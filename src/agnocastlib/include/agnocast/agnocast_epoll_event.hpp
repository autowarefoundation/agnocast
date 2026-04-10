#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace agnocast
{

enum class EpollEventType : uint32_t {
  Subscription = 0,
  Timer = 1,
  Shutdown = 2,
  NrEventType,
};

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

using EpollResult = std::pair<EpollEventType, EpollEventLocalID>;

class Epoll
{
  static constexpr int EPOLL_WAIT_MAX_EVENTS = 1;
  int epoll_fd_;

public:
  Epoll();

  Epoll(const Epoll &) = delete;
  Epoll & operator=(const Epoll &) = delete;

  Epoll(Epoll && other) noexcept : epoll_fd_(other.epoll_fd_) { other.epoll_fd_ = -1; }
  Epoll & operator=(Epoll && other) noexcept;

  ~Epoll();

  [[nodiscard]] int add_source(int fd, EpollEventType type, EpollEventLocalID local_id) const;

  void remove(int fd) const;

  int wait(std::vector<EpollResult> & results, int timeout_ms) const;

  [[nodiscard]] int fd() const { return epoll_fd_; }

  [[nodiscard]] bool is_valid() const { return epoll_fd_ >= 0; }
};

}  // namespace agnocast
