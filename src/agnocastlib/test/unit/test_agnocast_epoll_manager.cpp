#include "agnocast/agnocast_epoll.hpp"

#include <gtest/gtest.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

namespace
{

using agnocast::CallbackGroupValidator;
using agnocast::DummyEventHandler;
using agnocast::EpollEventHandler;
using agnocast::EpollEventLocalID;
using agnocast::EpollEventType;
using agnocast::EpollManager;
using agnocast::EventHandlerArray;

constexpr int kDispatchTimeoutMs = 200;
constexpr int kShortTimeoutMs = 20;

class RecordingEventHandler : public EpollEventHandler
{
public:
  explicit RecordingEventHandler(const EpollEventType type) : type_(type) {}

  [[nodiscard]] EpollEventType get_type() const override { return type_; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override
  {
    ++prepare_count_;
    last_prepared_epoll_fd_ = epoll_fd;
    validator_call_result_ = validate_callback_group(nullptr);
  }

  void handle(EpollEventLocalID event_local_id) override
  {
    ++handle_count_;
    last_local_id_ = event_local_id;
  }

  [[nodiscard]] int prepare_count() const { return prepare_count_.load(); }
  [[nodiscard]] int handle_count() const { return handle_count_.load(); }
  [[nodiscard]] EpollEventLocalID last_local_id() const { return last_local_id_.load(); }
  [[nodiscard]] int last_prepared_epoll_fd() const { return last_prepared_epoll_fd_.load(); }
  [[nodiscard]] bool validator_call_result() const { return validator_call_result_.load(); }

private:
  EpollEventType type_;
  std::atomic<int> prepare_count_{0};
  std::atomic<int> handle_count_{0};
  std::atomic<EpollEventLocalID> last_local_id_{0};
  std::atomic<int> last_prepared_epoll_fd_{-1};
  std::atomic<bool> validator_call_result_{false};
};

struct SourcesBundle
{
  EventHandlerArray sources{};
  std::array<RecordingEventHandler *, static_cast<size_t>(EpollEventType::NrEventType)> handlers{};
};

SourcesBundle make_recording_sources()
{
  SourcesBundle bundle;
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    auto handler = std::make_unique<RecordingEventHandler>(static_cast<EpollEventType>(type));
    bundle.handlers[type] = handler.get();
    bundle.sources[type] = std::move(handler);
  }
  return bundle;
}

}  // namespace

TEST(EpollManagerUnitTest, AddEventAndWaitDispatchesToMatchingHandler)
{
  auto bundle = make_recording_sources();
  EpollManager manager(std::move(bundle.sources));

  const int event_fd = eventfd(0, EFD_CLOEXEC);
  ASSERT_GE(event_fd, 0);

  constexpr auto kType = EpollEventType::Timer;
  constexpr EpollEventLocalID kLocalId = 1234;
  ASSERT_TRUE(manager.add_event(event_fd, kType, kLocalId));

  const uint64_t value = 1;
  ASSERT_EQ(write(event_fd, &value, sizeof(value)), static_cast<ssize_t>(sizeof(value)));

  manager.wait_and_handle_epoll_event(kDispatchTimeoutMs);

  const auto * timer_handler = bundle.handlers[static_cast<uint32_t>(EpollEventType::Timer)];
  ASSERT_NE(timer_handler, nullptr);
  EXPECT_EQ(timer_handler->handle_count(), 1);
  EXPECT_EQ(timer_handler->last_local_id(), kLocalId);

  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    if (type == static_cast<uint32_t>(kType)) {
      continue;
    }
    EXPECT_EQ(bundle.handlers[type]->handle_count(), 0);
  }

  EXPECT_EQ(close(event_fd), 0);
}

TEST(EpollManagerUnitTest, WaitTimeoutDoesNotCallAnyHandler)
{
  auto bundle = make_recording_sources();
  EpollManager manager(std::move(bundle.sources));

  manager.wait_and_handle_epoll_event(kShortTimeoutMs);

  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    EXPECT_EQ(bundle.handlers[type]->handle_count(), 0);
  }
}

TEST(EpollManagerUnitTest, PrepareEpollInvokesAllHandlersAndValidator)
{
  auto bundle = make_recording_sources();
  EpollManager manager(std::move(bundle.sources));
  manager.prepare_epoll([](const rclcpp::CallbackGroup::SharedPtr &) { return true; });

  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    EXPECT_EQ(bundle.handlers[type]->prepare_count(), 1);
    EXPECT_GE(bundle.handlers[type]->last_prepared_epoll_fd(), 0);
    EXPECT_TRUE(bundle.handlers[type]->validator_call_result());
  }
}

TEST(EpollManagerUnitTest, AddEventReturnsFalseWhenAddingDuplicateFd)
{
  auto bundle = make_recording_sources();
  EpollManager manager(std::move(bundle.sources));

  const int event_fd = eventfd(0, EFD_CLOEXEC);
  ASSERT_GE(event_fd, 0);

  ASSERT_TRUE(manager.add_event(event_fd, EpollEventType::Subscription, 1));
  EXPECT_FALSE(manager.add_event(event_fd, EpollEventType::Subscription, 2));

  EXPECT_EQ(close(event_fd), 0);
}

TEST(EpollManagerUnitTest, AddEventReturnsFalseWhenAddingInvalidFd)
{
  auto bundle = make_recording_sources();
  EpollManager manager(std::move(bundle.sources));

  const int invalid_fd = -1;
  EXPECT_FALSE(manager.add_event(invalid_fd, EpollEventType::Subscription, 0));
}

TEST(EpollManagerUnitTest, ConstructorThrowsWhenAnySourceIsNull)
{
  EventHandlerArray sources{};
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    sources[type] = std::make_unique<RecordingEventHandler>(static_cast<EpollEventType>(type));
  }
  sources[static_cast<uint32_t>(EpollEventType::Clock)] = nullptr;

  EXPECT_THROW(static_cast<void>(EpollManager(std::move(sources))), std::invalid_argument);
}

TEST(EpollManagerUnitTest, ConstructorThrowsWhenSourceTypeAndSlotMismatch)
{
  EventHandlerArray sources{};
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    sources[type] = std::make_unique<RecordingEventHandler>(static_cast<EpollEventType>(type));
  }
  sources[static_cast<uint32_t>(EpollEventType::Subscription)] =
    std::make_unique<RecordingEventHandler>(EpollEventType::Clock);

  EXPECT_THROW(static_cast<void>(EpollManager(std::move(sources))), std::invalid_argument);
}

TEST(EpollManagerUnitTest, ConstructorAcceptsDummyHandlers)
{
  EventHandlerArray sources{};
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); ++type) {
    sources[type] = std::make_unique<DummyEventHandler>();
  }

  EXPECT_NO_THROW(static_cast<void>(EpollManager(std::move(sources))));
}
