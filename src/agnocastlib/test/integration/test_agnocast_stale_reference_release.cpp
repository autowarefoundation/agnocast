// Integration test for releasing a message reference after its Subscription is gone.
//
// This happens when user code declares an ipc_shared_ptr member before the Subscription member that
// filled it: members are destroyed in reverse declaration order, so the pointer outlives the
// Subscription. ~SubscriptionBase issues AGNOCAST_REMOVE_SUBSCRIBER_CMD, which makes the kernel
// module clear this subscriber's bit on every entry of the topic, so the later release finds no bit
// to clear.
//
// That used to make the ioctl fail with EINVAL, which agnocastlib treated as fatal
// (close(agnocast_fd) + exit(EXIT_FAILURE)) -- killing an application whose only mistake was
// holding a message longer than its subscription. Clearing the bit is idempotent and the entry is
// reclaimed elsewhere, so the kernel module now reports success for an already-released reference.
// Reaching the end of each test proves the process survived.

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

using StringMsg = std_msgs::msg::String;

class StaleReferenceReleaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_stale_reference_release");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  // Publishes one message on `topic` and returns the reference a TakeSubscription took for it,
  // having already destroyed both the subscription and the publisher.
  agnocast::ipc_shared_ptr<const StringMsg> take_and_drop_subscription(
    const std::string & topic, const std::string & data)
  {
    const rclcpp::QoS qos{1};
    auto sub = std::make_shared<agnocast::TakeSubscription<StringMsg>>(node_.get(), topic, qos);
    auto pub = agnocast::create_publisher<StringMsg>(node_.get(), topic, qos);

    auto loaned = pub->borrow_loaned_message();
    loaned->data = data;
    pub->publish(std::move(loaned));

    return sub->take();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

// ---------------------------------------------------------------------------
// The reference outlives its Subscription. Before the fix, reset() terminated the process here.
// ---------------------------------------------------------------------------
TEST_F(StaleReferenceReleaseTest, release_after_subscription_destroyed_does_not_terminate)
{
  const std::string expected_data = "held past the subscription";

  auto held = take_and_drop_subscription("/test_stale_reference_release_single", expected_data);
  ASSERT_TRUE(static_cast<bool>(held)) << "take() returned an empty message";

  // The message is still readable: only the kernel's bookkeeping bit is gone, the shared-memory
  // entry itself is kept mapped (~SubscriptionBase does not unmap).
  EXPECT_EQ(held->data, expected_data);

  held.reset();
  SUCCEED() << "Process survived releasing a stale reference";
}

// ---------------------------------------------------------------------------
// Repeated stale releases across several topics must all stay non-fatal.
// ---------------------------------------------------------------------------
TEST_F(StaleReferenceReleaseTest, repeated_stale_releases_stay_non_fatal)
{
  for (int i = 0; i < 3; ++i) {
    const std::string topic = "/test_stale_reference_release_repeat_" + std::to_string(i);
    auto held = take_and_drop_subscription(topic, "msg " + std::to_string(i));
    ASSERT_TRUE(static_cast<bool>(held)) << "take() returned an empty message at i=" << i;
    held.reset();
  }

  SUCCEED() << "Process survived repeated stale releases";
}

// ---------------------------------------------------------------------------
// Copies share one control block, so only the last destruction reaches the kernel. Dropping the
// copies in the reverse order must not terminate either.
// ---------------------------------------------------------------------------
TEST_F(StaleReferenceReleaseTest, stale_release_from_last_of_several_copies)
{
  const std::string expected_data = "shared by three copies";

  auto first = take_and_drop_subscription("/test_stale_reference_release_copies", expected_data);
  ASSERT_TRUE(static_cast<bool>(first)) << "take() returned an empty message";

  auto second = first;
  auto third = second;

  first.reset();
  second.reset();
  EXPECT_EQ(third->data, expected_data);

  // Only this last reset() talks to the kernel.
  third.reset();
  SUCCEED() << "Process survived the last copy releasing a stale reference";
}
