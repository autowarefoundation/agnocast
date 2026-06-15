// Tests for get_erased_generic_callback and register_generic_callback.
//
// These tests verify the serialization path introduced in Commit 1 of
// the GenericSubscription feature. They run entirely in userspace without the
// kernel module: ipc_shared_ptr is constructed subscriber-side (entry_id != -1),
// so the reset() path calls the mocked release_subscriber_reference() instead
// of delete.
//
// The mock for release_subscriber_reference is defined in
// test_mocked_agnocast.cpp which is compiled into the same test target.
//
// Note: rmw_serialize / rmw_deserialize are real RMW calls that require a live
// RMW implementation.  The tests therefore call rclcpp::init / rclcpp::shutdown
// to ensure the RMW layer is initialized (following the same pattern used by
// other unit tests in this target).

#include "agnocast/agnocast_callback_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

using agnocast::get_erased_generic_callback;
using agnocast::RawMessagePtr;
using agnocast::TypeErasedCallback;

template <typename T>
agnocast::ipc_shared_ptr<std::byte> make_subscriber_raw_ptr(
  T * msg, const std::string & topic_name = "test_topic", agnocast::topic_local_id_t sub_id = 0,
  int64_t entry_id = 1)
{
  return agnocast::ipc_shared_ptr<std::byte>(
    reinterpret_cast<std::byte *>(msg), topic_name, sub_id, entry_id);
}

class GenericCallbackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ts_ = agnocast::TypeSupportBundle{
      nullptr, rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>()};
  }

  void TearDown() override { rclcpp::shutdown(); }

  agnocast::TypeSupportBundle ts_;
};

// ---------------------------------------------------------------------------
// Test 1: normal path — rmw_serialize round-trip via String
// ---------------------------------------------------------------------------
TEST_F(GenericCallbackTest, serialize_round_trips_std_msgs_string)
{
  // Arrange
  const std::string original_data = "hello agnocast";
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = original_data;

  auto raw_ptr = make_subscriber_raw_ptr(msg.get());
  RawMessagePtr envelope{std::move(raw_ptr)};

  std::shared_ptr<rclcpp::SerializedMessage> received;
  auto user_cb = [&received](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
    received = std::move(serialized);
  };

  TypeErasedCallback erased = get_erased_generic_callback(user_cb, ts_);

  // Act
  erased(std::move(envelope));

  // Assert — user callback was invoked with a non-null SerializedMessage
  ASSERT_NE(received, nullptr);
  ASSERT_GT(received->get_rcl_serialized_message().buffer_length, 0u);

  // Round-trip: deserialize back to String and compare
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  std_msgs::msg::String decoded;
  serializer.deserialize_message(received.get(), &decoded);
  EXPECT_EQ(decoded.data, original_data);
}

// ---------------------------------------------------------------------------
// Test 2: null raw pointer — callback must be skipped (not called)
// ---------------------------------------------------------------------------
TEST_F(GenericCallbackTest, null_raw_pointer_skips_callback)
{
  // Arrange: construct an empty ipc_shared_ptr<std::byte> (null)
  agnocast::ipc_shared_ptr<std::byte> null_ptr;
  RawMessagePtr envelope{std::move(null_ptr)};

  bool callback_called = false;
  auto user_cb = [&callback_called](std::shared_ptr<rclcpp::SerializedMessage>) {
    callback_called = true;
  };

  TypeErasedCallback erased = get_erased_generic_callback(user_cb, ts_);

  // Act
  erased(std::move(envelope));

  // Assert — callback must NOT have been called
  EXPECT_FALSE(callback_called);
}

// ---------------------------------------------------------------------------
// Test 3: wrong envelope type — must exit with failure
// ---------------------------------------------------------------------------
TEST_F(GenericCallbackTest, wrong_envelope_type_exits)
{
  // Arrange: pass a TypedMessagePtr<int> to a generic callback
  int dummy = 42;
  agnocast::TypedMessagePtr<int> wrong_envelope{
    agnocast::ipc_shared_ptr<int>(&dummy, "test_topic", 0, /*entry_id=*/1)};

  auto user_cb = [](std::shared_ptr<rclcpp::SerializedMessage>) {};

  TypeErasedCallback erased = get_erased_generic_callback(user_cb, ts_);

  // Act & Assert
  EXPECT_EXIT(
    erased(std::move(wrong_envelope)), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Agnocast internal implementation error: bad allocation when callback is called");
}
