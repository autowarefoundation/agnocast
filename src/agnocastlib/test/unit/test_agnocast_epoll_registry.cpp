// White-box tests for how epoll event handlers keep their epoll registrations in sync with the
// entities an Executor manages.

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_timer_info.hpp"

#include <gtest/gtest.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

using agnocast::CallbackGroupValidator;
using agnocast::EpollEventLocalID;
using agnocast::EpollEventType;
using agnocast::EpollRegistry;

namespace
{

// EPOLL_CTL_MOD fails with ENOENT for an fd that is not in the interest list.
bool is_registered(int epoll_fd, int fd)
{
  struct epoll_event ev = {};
  ev.events = EPOLLIN;
  return epoll_ctl(epoll_fd, EPOLL_CTL_MOD, fd, &ev) == 0;
}

int make_eventfd()
{
  return eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
}

class EpollFdTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    ASSERT_GE(epoll_fd_, 0);
  }

  void TearDown() override { close(epoll_fd_); }

  int epoll_fd_ = -1;
};

}  // namespace

// =============================================================================
// EpollRegistry
// =============================================================================

using EpollRegistryTest = EpollFdTest;

TEST_F(EpollRegistryTest, sync_adds_a_monitored_fd_with_packed_data)
{
  // Arrange
  EpollRegistry registry;
  const int fd = make_eventfd();
  ASSERT_GE(fd, 0);

  // Act
  const bool added = registry.sync(epoll_fd_, EpollEventType::Subscription, 7, fd, true);

  // Assert
  EXPECT_TRUE(added);
  const uint64_t one = 1;
  ASSERT_EQ(write(fd, &one, sizeof(one)), static_cast<ssize_t>(sizeof(one)));
  struct epoll_event ev = {};
  ASSERT_EQ(epoll_wait(epoll_fd_, &ev, 1, 0), 1);
  EXPECT_EQ(ev.data.u64, agnocast::pack_epoll_data(EpollEventType::Subscription, 7));

  close(fd);
}

TEST_F(EpollRegistryTest, sync_does_not_add_an_already_registered_fd_again)
{
  // Arrange
  EpollRegistry registry;
  const int fd = make_eventfd();
  ASSERT_GE(fd, 0);
  ASSERT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true));

  // Act — a second EPOLL_CTL_ADD would fail with EEXIST and abort the process.
  const bool added = registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true);

  // Assert
  EXPECT_FALSE(added);
  EXPECT_TRUE(is_registered(epoll_fd_, fd));

  close(fd);
}

TEST_F(EpollRegistryTest, sync_removes_an_fd_that_is_no_longer_monitored)
{
  // Arrange
  EpollRegistry registry;
  const int fd = make_eventfd();
  ASSERT_GE(fd, 0);
  ASSERT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true));

  // Act
  registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, false);

  // Assert — the fd stays open but is no longer monitored.
  EXPECT_FALSE(is_registered(epoll_fd_, fd));

  // Re-monitoring adds it back.
  EXPECT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true));
  EXPECT_TRUE(is_registered(epoll_fd_, fd));

  close(fd);
}

TEST_F(EpollRegistryTest, sync_forgets_an_entity_whose_fd_was_closed_without_a_new_one)
{
  // Arrange
  EpollRegistry registry;
  const int fd = make_eventfd();
  ASSERT_GE(fd, 0);
  ASSERT_TRUE(registry.sync(epoll_fd_, EpollEventType::Timer, 3, fd, true));
  close(fd);

  // Act — the fd number may already belong to another entity, so it must not be deleted.
  const int reused_fd = make_eventfd();
  ASSERT_GE(reused_fd, 0);
  ASSERT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 9, reused_fd, true));
  registry.sync(epoll_fd_, EpollEventType::Timer, 3, -1, true);

  // Assert
  EXPECT_TRUE(is_registered(epoll_fd_, reused_fd));

  close(reused_fd);
}

TEST_F(EpollRegistryTest, forget_unless_drops_registrations_without_deleting_them)
{
  // Arrange
  EpollRegistry registry;
  const int fd = make_eventfd();
  ASSERT_GE(fd, 0);
  ASSERT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true));

  // Act
  registry.forget_unless([](EpollEventLocalID) { return false; });

  // Assert — the owner closes the fd, which removes it from epoll; the registry only forgets.
  EXPECT_TRUE(is_registered(epoll_fd_, fd));
  ASSERT_EQ(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr), 0);
  EXPECT_TRUE(registry.sync(epoll_fd_, EpollEventType::Subscription, 1, fd, true));

  close(fd);
}
