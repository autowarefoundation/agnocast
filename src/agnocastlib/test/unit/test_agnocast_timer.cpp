// White-box tests for the timer subsystem internals — rewrite or delete on internal
// refactor. The public-API spec lives in test_agnocast_timer_api.cpp.

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_timer_info.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include <gtest/gtest.h>
#include <poll.h>
#include <rcl/time.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

// Forward declarations for internal helpers (defined in agnocast_timer_info.cpp
// but not exposed via the public header; declared here so tests can call them
// directly).
namespace agnocast
{
void handle_pre_time_jump(TimerInfo & timer_info);
void handle_post_time_jump(TimerInfo & timer_info, const rcl_time_jump_t & jump);
}  // namespace agnocast

class TestTimer : public ::testing::Test
{
protected:
  void TearDown() override
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info.clear();
  }

  static constexpr int64_t kPeriodNs = 100'000'000;  // 100ms

  // Creates a Clock (RCL_ROS_TIME) with ROS time override enabled and set to `time_ns`.
  rclcpp::Clock::SharedPtr make_ros_clock_at(int64_t time_ns) const
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rcl_clock_t * rcl_clock = clock->get_clock_handle();
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    EXPECT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    EXPECT_EQ(rcl_set_ros_time_override(rcl_clock, time_ns), RCL_RET_OK);
    return clock;
  }

  // Builds a TimerInfo with the minimum fields required by the time-jump handlers.
  std::shared_ptr<agnocast::TimerInfo> make_timer_info(
    rclcpp::Clock::SharedPtr clock, int64_t now_ns, int64_t period_ns = kPeriodNs) const
  {
    auto info = std::make_shared<agnocast::TimerInfo>();
    info->timer_id = 1;
    info->period_ns.store(period_ns, std::memory_order_relaxed);
    info->clock = std::move(clock);
    info->last_call_time_ns.store(now_ns, std::memory_order_relaxed);
    info->next_call_time_ns.store(now_ns + period_ns, std::memory_order_relaxed);
    info->time_credit.store(0, std::memory_order_relaxed);
    return info;
  }

  // Reads up to one u64 event from `fd`. Returns true if an event was consumed.
  // poll() guards against blocking when the fd was not opened with EFD_NONBLOCK.
  static bool consume_eventfd(int fd)
  {
    pollfd pfd{fd, POLLIN, 0};
    if (poll(&pfd, 1, /*timeout_ms=*/0) <= 0) {
      return false;
    }
    uint64_t value = 0;
    const ssize_t ret = read(fd, &value, sizeof(value));
    return ret == sizeof(value) && value > 0;
  }
};

// handle_pre_time_jump — saves remaining period as time_credit before a jump
// =============================================================================

TEST_F(TestTimer, handle_pre_time_jump_saves_remaining_period_as_time_credit)
{
  // Arrange
  const int64_t now_ns = 500'000'000;  // 0.5s on the new epoch
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  // Pin next_call so the credit is deterministic: now + 30ms.
  const int64_t next_call_ns = now_ns + 30'000'000;
  info->next_call_time_ns.store(next_call_ns, std::memory_order_relaxed);

  // Act
  agnocast::handle_pre_time_jump(*info);

  // Assert
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), next_call_ns - now_ns);
}

TEST_F(TestTimer, handle_pre_time_jump_is_noop_when_clock_uninitialized)
{
  // Arrange — ROS clock at t=0 (uninitialized).
  auto clock = make_ros_clock_at(0);
  auto info = make_timer_info(clock, /*now_ns=*/0);
  // Had the function not bailed out, it would have stored
  // (next_call_time_ns - now_ns) into time_credit. Pick a sentinel that
  // differs from that value so a false positive cannot hide a regression.
  const int64_t would_be_credit = info->next_call_time_ns.load(std::memory_order_relaxed);
  const int64_t sentinel = would_be_credit + 1;
  info->time_credit.store(sentinel, std::memory_order_relaxed);

  // Act
  agnocast::handle_pre_time_jump(*info);

  // Assert — time_credit is preserved (function did not perform its store).
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), sentinel);
}

TEST_F(TestTimer, handle_pre_time_jump_swallows_exception_from_clock_now)
{
  // Arrange — RCL_CLOCK_UNINITIALIZED has no get_now hook, so clock->now()
  // throws rclcpp::exceptions::RCLError (a std::exception).
  auto clock = std::make_shared<rclcpp::Clock>(RCL_CLOCK_UNINITIALIZED);
  auto info = make_timer_info(clock, /*now_ns=*/0);
  const int64_t would_be_credit = info->next_call_time_ns.load(std::memory_order_relaxed);
  const int64_t sentinel = would_be_credit + 1;
  info->time_credit.store(sentinel, std::memory_order_relaxed);

  // Act / Assert — exception must not escape.
  EXPECT_NO_THROW(agnocast::handle_pre_time_jump(*info));
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), sentinel);
}

// handle_post_time_jump (RCL_ROS_TIME_ACTIVATED) — closes timer_fd and applies time_credit
// =============================================================================

TEST_F(TestTimer, handle_post_time_jump_ros_time_activation_closes_timer_fd)
{
  // Arrange
  auto clock = make_ros_clock_at(1'000'000'000);  // 1s
  auto info = make_timer_info(clock, /*now_ns=*/1'000'000'000);
  // Open a real eventfd as a stand-in for timerfd so we can observe the close.
  info->timer_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->timer_fd, -1);
}

TEST_F(TestTimer, handle_post_time_jump_ros_time_activation_consumes_and_applies_time_credit)
{
  // Arrange — 30ms of credit was saved in the pre-jump callback.
  const int64_t now_ns = 2'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  const int64_t credit = 30'000'000;
  info->time_credit.store(credit, std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — credit is consumed (set to 0) and applied to the call-time anchors.
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns - credit);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns - credit + kPeriodNs);
}

TEST_F(TestTimer, handle_post_time_jump_ros_time_activation_leaves_anchors_when_no_credit)
{
  // Arrange — credit is zero (pre-jump never ran or stored 0).
  const int64_t now_ns = 2'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
}

TEST_F(TestTimer, handle_post_time_jump_ros_time_activation_skips_credit_when_clock_uninitialized)
{
  // Arrange — clock at 0 (uninitialized) with credit pre-saved.
  auto clock = make_ros_clock_at(0);
  auto info = make_timer_info(clock, /*now_ns=*/0);
  info->timer_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);
  const int64_t credit = 30'000'000;
  info->time_credit.store(credit, std::memory_order_relaxed);
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_ACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert — timer_fd is closed but credit is preserved and anchors are untouched.
  EXPECT_EQ(info->timer_fd, -1);
  EXPECT_EQ(info->time_credit.load(std::memory_order_relaxed), credit);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
}

// handle_post_time_jump (RCL_ROS_TIME_DEACTIVATED) — currently a no-op (warns only)
// =============================================================================

TEST_F(TestTimer, handle_post_time_jump_ros_time_deactivation_does_not_mutate_state)
{
  // Arrange
  const int64_t now_ns = 3'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  info->timer_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);
  const int fd_before = info->timer_fd;
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_DEACTIVATED;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->timer_fd, fd_before);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);

  close(info->timer_fd);
  info->timer_fd = -1;
}

// handle_post_time_jump (forward jump) — writes clock_eventfd when timer is ready
// =============================================================================

TEST_F(TestTimer, handle_post_time_jump_forward_jump_writes_clock_eventfd_when_ready)
{
  // Arrange — now has advanced past next_call_time, so the timer is ready.
  const int64_t now_ns = 1'200'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns - 200'000'000);
  info->next_call_time_ns.store(now_ns - 100'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    /*timer_id=*/0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::function<void()>{[]() {}});
  info->timer = timer;
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_TRUE(consume_eventfd(info->clock_eventfd));
}

TEST_F(TestTimer, handle_post_time_jump_forward_jump_does_not_write_when_canceled)
{
  // Arrange — timer would be ready (now past next_call_time), but it is canceled.
  const int64_t now_ns = 1'200'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/now_ns - 200'000'000);
  info->next_call_time_ns.store(now_ns - 100'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    /*timer_id=*/0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::function<void()>{[]() {}});
  info->timer = timer;
  timer->cancel();
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_FALSE(consume_eventfd(info->clock_eventfd));
}

TEST_F(TestTimer, handle_post_time_jump_forward_jump_does_not_write_when_not_ready)
{
  // Arrange — next_call_time is still in the future after the jump.
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  info->next_call_time_ns.store(now_ns + 100'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_FALSE(consume_eventfd(info->clock_eventfd));
}

// handle_post_time_jump (backward jump) — resets anchors when now < last_call_time
// =============================================================================

TEST_F(TestTimer, handle_post_time_jump_backward_jump_resets_anchors_when_past_last_call)
{
  // Arrange — now is earlier than the recorded last_call_time.
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = 200'000'000;  // 800ms backwards
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + kPeriodNs);
}

TEST_F(TestTimer, handle_post_time_jump_leaves_anchors_when_jump_falls_within_one_period)
{
  // Arrange — now is between last_call and next_call: neither branch fires.
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = 1'050'000'000;  // 50ms after last_call
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  info->next_call_time_ns.store(now_ns + 50'000'000, std::memory_order_relaxed);
  info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  ASSERT_GE(info->clock_eventfd, 0);
  rcl_time_jump_t jump = {};
  jump.clock_change = RCL_ROS_TIME_NO_CHANGE;

  // Act
  agnocast::handle_post_time_jump(*info, jump);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), last_call_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + 50'000'000);
  EXPECT_FALSE(consume_eventfd(info->clock_eventfd));
}

// =============================================================================
// handle_timer_event — periodic dispatch with missed-cycle catch-up
// =============================================================================

TEST_F(TestTimer, handle_timer_event_is_noop_when_timer_weakptr_is_expired)
{
  // Arrange — info->timer is left as default-constructed (empty weak_ptr).
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns);
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);

  // Act
  agnocast::handle_timer_event(*info);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
}

TEST_F(TestTimer, handle_timer_event_is_noop_when_timer_is_canceled)
{
  // Arrange — timer is ready (now past next_call_time) but was canceled.
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns - kPeriodNs);
  int call_count = 0;
  std::function<void()> cb = [&call_count]() { ++call_count; };
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    /*timer_id=*/0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::move(cb));
  info->timer = timer;
  timer->cancel();
  const int64_t snapshot_last = info->last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);

  // Act
  agnocast::handle_timer_event(*info);

  // Assert — canceled timer must not invoke callback or mutate anchors.
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), snapshot_last);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
  EXPECT_EQ(call_count, 0);
}

TEST_F(TestTimer, handle_timer_event_advances_next_call_by_one_period_and_invokes_callback)
{
  // Arrange — timer last fired one period ago and is scheduled to fire at now.
  // The candidate (stored + period) lands in the future, so catch-up is not entered.
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, now_ns - kPeriodNs);
  int call_count = 0;
  std::function<void()> cb = [&call_count]() { ++call_count; };
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    /*timer_id=*/0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::move(cb));
  info->timer = timer;

  // Act
  agnocast::handle_timer_event(*info);

  // Assert
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + kPeriodNs);
  EXPECT_EQ(call_count, 1);
}

TEST_F(TestTimer, handle_timer_event_just_past_two_period_boundary_advances_extra_period)
{
  // Arrange — simulate "executor is 2 periods + 1ns late": catch-up must advance
  // next_call by 3 periods (1ns past the boundary triggers an extra period).
  const int64_t stored_next = 1'000'000'000;
  const int64_t now_ns = stored_next + 2 * kPeriodNs + 1;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, stored_next - kPeriodNs);
  std::function<void()> cb = []() {};
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::move(cb));
  info->timer = timer;

  // Act
  agnocast::handle_timer_event(*info);

  // Assert
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), stored_next + 3 * kPeriodNs);
}

TEST_F(TestTimer, handle_timer_event_catches_up_many_periods_at_once)
{
  // Arrange — simulate "executor is 5 periods late". Catch-up must advance by 5
  // periods AND callback must fire exactly once (drop-missed semantics, not burst).
  const int64_t stored_next = 1'000'000'000;
  const int64_t now_ns = stored_next + 5 * kPeriodNs;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, stored_next - kPeriodNs);
  int call_count = 0;
  std::function<void()> cb = [&call_count]() { ++call_count; };
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    0u, std::chrono::nanoseconds{kPeriodNs}, clock, std::move(cb));
  info->timer = timer;

  // Act
  agnocast::handle_timer_event(*info);

  // Assert — exactly 5 periods advance, ending at now (no overshoot).
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(call_count, 1) << "drop-missed: must fire once even when many periods late";
}

TEST_F(TestTimer, handle_timer_event_with_zero_period_sets_next_call_to_now)
{
  // Arrange — period=0 timer with stored next_call far in the past. Callback must
  // fire exactly once (no infinite-loop in the period=0 catch-up branch).
  const int64_t stored_next = 100;
  const int64_t now_ns = 1'000'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, stored_next, /*period_ns=*/0);
  int call_count = 0;
  std::function<void()> cb = [&call_count]() { ++call_count; };
  auto timer = std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
    0u, std::chrono::nanoseconds{0}, clock, std::move(cb));
  info->timer = timer;

  // Act
  agnocast::handle_timer_event(*info);

  // Assert
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(call_count, 1) << "period=0 must fire exactly once per call, not loop";
}

// =============================================================================
// TimerInfo::set_period — semantics defined in TimerBase::set_period docstring.
// =============================================================================

TEST_F(TestTimer, set_period_updates_period_only_without_modifying_call_time_anchors)
{
  // Arrange — last_call != now catches an anchor-to-now regression (set_period must not
  // behave like reset).
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = 1'200'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  const std::chrono::nanoseconds new_period{500'000'000};

  // Act
  info->set_period(new_period);

  // Assert — period swapped; call-time anchors untouched.
  EXPECT_EQ(info->period_ns.load(std::memory_order_relaxed), new_period.count());
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), last_call_ns);
}

TEST_F(TestTimer, set_period_arms_timer_fd_to_fire_at_existing_next_call_then_every_new_period)
{
  // Arrange — pin now 30ms before next_call so remaining = 30ms. Expect it_value=remaining
  // (preserves the existing firing time) and it_interval=new_period (subsequent firings).
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = last_call_ns + 70'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  const int64_t expected_remaining_ns =
    info->next_call_time_ns.load(std::memory_order_relaxed) - now_ns;
  ASSERT_EQ(expected_remaining_ns, 30'000'000);
  info->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);
  const std::chrono::nanoseconds new_period{250'000'000};

  // Act
  info->set_period(new_period);

  // Assert
  struct itimerspec spec = {};
  ASSERT_EQ(timerfd_gettime(info->timer_fd, &spec), 0);
  // it_value: kernel reports remaining time. Slack because nanoseconds elapse between
  // set_period and timerfd_gettime.
  const int64_t got_value_ns = spec.it_value.tv_sec * 1'000'000'000LL + spec.it_value.tv_nsec;
  EXPECT_GT(got_value_ns, 0);
  EXPECT_LE(got_value_ns, expected_remaining_ns);
  const int64_t got_interval_ns =
    spec.it_interval.tv_sec * 1'000'000'000LL + spec.it_interval.tv_nsec;
  EXPECT_EQ(got_interval_ns, new_period.count());
}

TEST_F(TestTimer, set_period_arms_timer_fd_to_fire_immediately_when_already_overdue)
{
  // Arrange — now > next_call → remaining is negative. set_period must clamp it_value to
  // 1ns (0 disarms the timerfd; negative is rejected with EINVAL). The 1ns expiry fires
  // essentially immediately; verify by polling the timerfd for readability rather than
  // by reading it_value (which the kernel auto-overwrites with the next interval).
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = last_call_ns + 500'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  ASSERT_LT(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns);
  info->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);

  // Act
  info->set_period(std::chrono::nanoseconds{250'000'000});

  // Assert
  pollfd pfd{info->timer_fd, POLLIN, 0};
  ASSERT_GT(poll(&pfd, 1, /*timeout_ms=*/100), 0) << "timerfd should be readable within 100ms";
  EXPECT_TRUE(pfd.revents & POLLIN);
  uint64_t expirations = 0;
  ASSERT_EQ(
    read(info->timer_fd, &expirations, sizeof(expirations)),
    static_cast<ssize_t>(sizeof(expirations)));
  EXPECT_GE(expirations, 1u);
}

TEST_F(TestTimer, set_period_with_zero_period_arms_timer_fd_with_one_nanosecond_interval)
{
  // Arrange — same 1ns interval workaround as arm_timer_fd: zero would disarm the timerfd.
  const int64_t last_call_ns = 1'000'000'000;
  const int64_t now_ns = last_call_ns + 30'000'000;
  auto clock = make_ros_clock_at(now_ns);
  auto info = make_timer_info(clock, /*now_ns=*/last_call_ns);
  info->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  ASSERT_GE(info->timer_fd, 0);

  // Act
  info->set_period(std::chrono::nanoseconds{0});

  // Assert
  struct itimerspec spec = {};
  ASSERT_EQ(timerfd_gettime(info->timer_fd, &spec), 0);
  EXPECT_EQ(spec.it_interval.tv_sec, 0);
  EXPECT_EQ(spec.it_interval.tv_nsec, 1);
}

TEST_F(TestTimer, set_period_throws_when_underlying_timer_info_is_invalid)
{
  // Arrange — rcl's RCL_RET_TIMER_INVALID equivalent: the weak_ptr to TimerInfo fails to
  // lock. Reproduced here by force-erasing the global registry to drop the only strong ref.
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  auto node = std::make_shared<agnocast::Node>("test_timer_set_period_invalid", options);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info.clear();
  }

  // Act / Assert
  EXPECT_THROW(timer->set_period(std::chrono::nanoseconds{50'000'000}), std::runtime_error);
}

TEST_F(TestTimer, set_period_skips_timer_fd_re_arm_when_fd_is_minus_one)
{
  // Arrange — sim-time path: no timerfd. period is updated; next_call is preserved.
  auto clock = make_ros_clock_at(1'200'000'000);
  auto info = make_timer_info(clock, /*now_ns=*/1'000'000'000);
  ASSERT_EQ(info->timer_fd, -1);
  const int64_t snapshot_next = info->next_call_time_ns.load(std::memory_order_relaxed);
  const std::chrono::nanoseconds new_period{250'000'000};

  // Act / Assert
  EXPECT_NO_THROW(info->set_period(new_period));
  EXPECT_EQ(info->period_ns.load(std::memory_order_relaxed), new_period.count());
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), snapshot_next);
}

// =============================================================================
// register_timer_info — initializes TimerInfo and inserts into the registry
// =============================================================================

class TestRegisterTimerInfo : public ::testing::Test
{
protected:
  void TearDown() override
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    agnocast::id2_timer_info.clear();
  }

  rclcpp::CallbackGroup::SharedPtr default_callback_group()
  {
    if (!node_) {
      rclcpp::NodeOptions options;
      options.start_parameter_services(false);
      node_ = std::make_shared<agnocast::Node>("test_register_timer_info", options);
    }
    return node_->get_node_base_interface()->get_default_callback_group();
  }

  std::shared_ptr<agnocast::TimerBase> make_generic_timer(
    uint32_t timer_id, std::chrono::nanoseconds period, rclcpp::Clock::SharedPtr clock)
  {
    return std::make_shared<agnocast::GenericTimer<std::function<void()>>>(
      timer_id, period, std::move(clock), std::function<void()>{[]() {}});
  }

  std::shared_ptr<agnocast::TimerInfo> find_registered(uint32_t timer_id) const
  {
    std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
    auto it = agnocast::id2_timer_info.find(timer_id);
    return it != agnocast::id2_timer_info.end() ? it->second : nullptr;
  }

  static constexpr int64_t kPeriodNs = 50'000'000;  // 50ms

  std::shared_ptr<agnocast::Node> node_;
};

TEST_F(TestRegisterTimerInfo, steady_time_clock_creates_timer_fd_only)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = std::chrono::nanoseconds{kPeriodNs};
  const uint32_t timer_id = 1;
  auto timer = make_generic_timer(timer_id, period, clock);

  // Act
  agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

  // Assert
  auto info = find_registered(timer_id);
  ASSERT_NE(info, nullptr);
  EXPECT_GE(info->timer_fd, 0);
  EXPECT_EQ(info->clock_eventfd, -1);
}

TEST_F(TestRegisterTimerInfo, ros_time_clock_inactive_creates_both_timer_fd_and_clock_eventfd)
{
  // Arrange — RCL_ROS_TIME clock without override enabled (sim time inactive).
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_FALSE(clock->ros_time_is_active());
  const auto period = std::chrono::nanoseconds{kPeriodNs};
  const uint32_t timer_id = 2;
  auto timer = make_generic_timer(timer_id, period, clock);

  // Act
  agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

  // Assert
  auto info = find_registered(timer_id);
  ASSERT_NE(info, nullptr);
  EXPECT_GE(info->timer_fd, 0);
  EXPECT_GE(info->clock_eventfd, 0);
}

TEST_F(TestRegisterTimerInfo, ros_time_clock_active_skips_timer_fd)
{
  // Arrange — RCL_ROS_TIME clock with override enabled (sim time active).
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, 1'000'000'000LL), RCL_RET_OK);
  }
  ASSERT_TRUE(clock->ros_time_is_active());
  const auto period = std::chrono::nanoseconds{kPeriodNs};
  const uint32_t timer_id = 3;
  auto timer = make_generic_timer(timer_id, period, clock);

  // Act
  agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

  // Assert
  auto info = find_registered(timer_id);
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->timer_fd, -1);
  EXPECT_GE(info->clock_eventfd, 0);
}

TEST_F(TestRegisterTimerInfo, populates_timer_info_from_arguments_and_clock)
{
  // Arrange — pin ROS time so the anchor values can be asserted exactly.
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock->get_clock_handle();
  const int64_t now_ns = 5'000'000'000LL;
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(rcl_clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(rcl_clock, now_ns), RCL_RET_OK);
  }
  const auto period = std::chrono::nanoseconds{kPeriodNs};
  const uint32_t timer_id = 42;
  auto timer = make_generic_timer(timer_id, period, clock);
  auto cb_group = default_callback_group();

  // Act
  agnocast::register_timer_info(timer_id, timer, period, cb_group, clock);

  // Assert
  auto info = find_registered(timer_id);
  ASSERT_NE(info, nullptr);
  // Argument forwarding: each argument is stored verbatim.
  EXPECT_EQ(info->timer_id, timer_id);
  EXPECT_EQ(info->timer.lock(), timer);
  EXPECT_EQ(info->period_ns.load(std::memory_order_relaxed), period.count());
  EXPECT_EQ(info->callback_group, cb_group);
  // Call-time anchors initialized from clock->now().
  EXPECT_EQ(info->last_call_time_ns.load(std::memory_order_relaxed), now_ns);
  EXPECT_EQ(info->next_call_time_ns.load(std::memory_order_relaxed), now_ns + kPeriodNs);
}

TEST_F(TestRegisterTimerInfo, sets_epoll_update_flags_per_created_fds)
{
  // Arrange — STEADY_TIME so only timer_fd is created (no clock_eventfd).
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = std::chrono::nanoseconds{kPeriodNs};
  const uint32_t timer_id = 5;
  auto timer = make_generic_timer(timer_id, period, clock);

  // Act
  agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

  // Assert
  auto info = find_registered(timer_id);
  ASSERT_NE(info, nullptr);
  EXPECT_TRUE(info->need_epoll_update);
  EXPECT_TRUE(info->timer_fd_need_update);
  EXPECT_FALSE(info->clock_eventfd_need_update);
}

TEST_F(TestRegisterTimerInfo, attaches_jump_handler_only_for_ros_time)
{
  // STEADY_TIME — no jump handler.
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    const auto period = std::chrono::nanoseconds{kPeriodNs};
    const uint32_t timer_id = 6;
    auto timer = make_generic_timer(timer_id, period, clock);

    agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

    auto info = find_registered(timer_id);
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(info->jump_handler, nullptr);
  }

  // ROS_TIME — jump handler installed (drives handle_pre/post_time_jump).
  {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    const auto period = std::chrono::nanoseconds{kPeriodNs};
    const uint32_t timer_id = 7;
    auto timer = make_generic_timer(timer_id, period, clock);

    agnocast::register_timer_info(timer_id, timer, period, default_callback_group(), clock);

    auto info = find_registered(timer_id);
    ASSERT_NE(info, nullptr);
    EXPECT_NE(info->jump_handler, nullptr);
  }
}

// =============================================================================
// create_timer (free function) — wrapper over register_timer_info; tested here
// because the default-callback-group fallback is observable only via the
// internal registry (no public API exposes the callback group).
// =============================================================================

TEST_F(TestRegisterTimerInfo, create_timer_uses_default_callback_group_when_nullptr)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  auto expected_group = default_callback_group();

  // Act — pass nullptr explicitly; create_timer must fall back to the node's default.
  auto timer = agnocast::create_timer(node_.get(), clock, period, []() {}, nullptr);

  // Assert — locate the registered TimerInfo by timer instance and verify the group.
  ASSERT_NE(timer, nullptr);
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  bool found = false;
  for (const auto & [id, info] : agnocast::id2_timer_info) {
    if (info->callback_group == expected_group && info->timer.lock() == timer) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "create_timer with nullptr group must use the node's default";
}

// =============================================================================
// allocate_timer_id — overflow guard against the reserved epoll-flag bits region
// =============================================================================

TEST(AllocateTimerIdTest, throws_when_id_reaches_reserved_range)
{
  // Arrange — pin next_timer_id at the boundary so the next allocation overflows.
  const uint32_t original = agnocast::next_timer_id.load();
  agnocast::next_timer_id.store(agnocast::MAX_TIMER_ID);

  // Act & Assert
  EXPECT_THROW(agnocast::allocate_timer_id(), std::runtime_error);

  // Cleanup
  agnocast::next_timer_id.store(original);
}

TEST(AllocateTimerIdTest, succeeds_at_the_highest_valid_id)
{
  // Arrange — last allocatable ID is MAX_TIMER_ID - 1.
  const uint32_t original = agnocast::next_timer_id.load();
  agnocast::next_timer_id.store(agnocast::MAX_TIMER_ID - 1);

  // Act & Assert
  EXPECT_EQ(agnocast::allocate_timer_id(), agnocast::MAX_TIMER_ID - 1);

  // Cleanup
  agnocast::next_timer_id.store(original);
}
