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

CallbackGroupValidator manages_all()
{
  return [](const rclcpp::CallbackGroup::SharedPtr &) { return true; };
}

CallbackGroupValidator manages_none()
{
  return [](const rclcpp::CallbackGroup::SharedPtr &) { return false; };
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

// =============================================================================
// SubscriptionEventHandler::prepare_epoll
// =============================================================================

class SubscriptionPrepareEpollTest : public EpollFdTest
{
protected:
  void TearDown() override
  {
    {
      std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
      agnocast::id2_callback_info.clear();
    }
    for (const int fd : fds_) {
      close(fd);
    }
    EpollFdTest::TearDown();
  }

  int add_callback_info(uint32_t callback_info_id)
  {
    const int fd = make_eventfd();
    EXPECT_GE(fd, 0);
    fds_.push_back(fd);

    agnocast::CallbackInfo info;
    info.topic_name = "/test";
    info.is_transient_local = false;
    info.notify_eventfd = fd;

    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    agnocast::id2_callback_info[callback_info_id] = std::move(info);
    return fd;
  }

  agnocast::SubscriptionEventHandler make_handler()
  {
    return agnocast::SubscriptionEventHandler(getpid(), &ready_mutex_, &ready_executables_);
  }

  std::vector<int> fds_;
  std::mutex ready_mutex_;
  std::vector<agnocast::AgnocastExecutable> ready_executables_;
};

TEST_F(SubscriptionPrepareEpollTest, removes_a_subscription_that_is_no_longer_managed)
{
  // Arrange
  const int fd = add_callback_info(1);
  auto handler = make_handler();
  handler.prepare_epoll(epoll_fd_, manages_all());
  ASSERT_TRUE(is_registered(epoll_fd_, fd));

  // Act — e.g. the callback group was removed from the Executor.
  handler.prepare_epoll(epoll_fd_, manages_none());

  // Assert
  EXPECT_FALSE(is_registered(epoll_fd_, fd));
}

TEST_F(SubscriptionPrepareEpollTest, adds_a_subscription_again_once_it_is_managed_again)
{
  // Arrange
  const int fd = add_callback_info(1);
  auto handler = make_handler();
  handler.prepare_epoll(epoll_fd_, manages_all());
  handler.prepare_epoll(epoll_fd_, manages_none());

  // Act
  handler.prepare_epoll(epoll_fd_, manages_all());

  // Assert
  EXPECT_TRUE(is_registered(epoll_fd_, fd));
}

TEST_F(SubscriptionPrepareEpollTest, each_handler_registers_into_its_own_epoll)
{
  // Arrange — two Executors, e.g. a callback group moved from one to the other.
  const int fd = add_callback_info(1);
  auto first = make_handler();
  auto second = make_handler();
  const int second_epoll_fd = epoll_create1(EPOLL_CLOEXEC);
  ASSERT_GE(second_epoll_fd, 0);
  first.prepare_epoll(epoll_fd_, manages_all());

  // Act
  first.prepare_epoll(epoll_fd_, manages_none());
  second.prepare_epoll(second_epoll_fd, manages_all());

  // Assert
  EXPECT_FALSE(is_registered(epoll_fd_, fd));
  EXPECT_TRUE(is_registered(second_epoll_fd, fd));

  close(second_epoll_fd);
}

TEST_F(SubscriptionPrepareEpollTest, a_destroyed_subscription_does_not_block_a_reused_fd)
{
  // Arrange — mirror ~Subscription(): erase the entry, then close its eventfd.
  auto handler = make_handler();
  const int old_fd = add_callback_info(1);
  handler.prepare_epoll(epoll_fd_, manages_all());
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_callback_info_mtx);
    agnocast::id2_callback_info.erase(1);
  }
  close(old_fd);
  fds_.clear();

  // Act — the new eventfd usually gets the same number as the closed one.
  const int new_fd = add_callback_info(2);
  handler.prepare_epoll(epoll_fd_, manages_all());

  // Assert
  EXPECT_TRUE(is_registered(epoll_fd_, new_fd));
}
