#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <message_filters/message_event.h>

#include <memory>
#include <type_traits>

namespace agnocast
{
namespace message_filters
{

/// Wrapper around a message pointer that carries metadata for message filter pipelines.
AGNOCAST_PUBLIC
template <typename M>
class MessageEvent
{
  static_assert(
    std::is_const<M>::value,
    "agnocast::message_filters::MessageEvent only supports const message types. "
    "Use MessageEvent<YourMessage const> instead of MessageEvent<YourMessage>.");

public:
  using ConstMessage = M;
  using Message = typename std::remove_const<M>::type;
  using ConstMessagePtr = ipc_shared_ptr<ConstMessage>;

  /// Default-construct an empty MessageEvent.
  AGNOCAST_PUBLIC MessageEvent() = default;

  /// Copy constructor.
  AGNOCAST_PUBLIC MessageEvent(const MessageEvent & rhs) = default;

  /// Construct from a message pointer, recording the current time as receipt time.
  AGNOCAST_PUBLIC explicit MessageEvent(const ConstMessagePtr & message)
  : message_(message), receipt_time_(rclcpp::Clock().now())
  {
  }

  /// Construct from a message pointer and an explicit receipt time.
  AGNOCAST_PUBLIC MessageEvent(const ConstMessagePtr & message, rclcpp::Time receipt_time)
  : message_(message), receipt_time_(receipt_time)
  {
  }

  /// Copy assignment operator.
  AGNOCAST_PUBLIC MessageEvent & operator=(const MessageEvent & rhs) = default;

  /**
   * \brief Retrieve the message.
   * Returns ipc_shared_ptr<M const> pointing to shared memory.
   */
  AGNOCAST_PUBLIC const ConstMessagePtr & getMessage() const { return message_; }

  /**
   * \brief Retrieve a const version of the message (same as getMessage() in agnocast)
   */
  AGNOCAST_PUBLIC const ConstMessagePtr & getConstMessage() const { return message_; }

  /**
   * \brief Returns the time at which this message was received
   */
  AGNOCAST_PUBLIC rclcpp::Time getReceiptTime() const { return receipt_time_; }

  /// Less-than comparison, ordered by pointer then receipt time.
  AGNOCAST_PUBLIC bool operator<(const MessageEvent & rhs) const
  {
    if (message_.get() != rhs.message_.get()) {
      return message_.get() < rhs.message_.get();
    }

    return receipt_time_ < rhs.receipt_time_;
  }

  /// Equality comparison.
  AGNOCAST_PUBLIC bool operator==(const MessageEvent & rhs) const
  {
    return message_.get() == rhs.message_.get() && receipt_time_ == rhs.receipt_time_;
  }

  /// Inequality comparison.
  AGNOCAST_PUBLIC bool operator!=(const MessageEvent & rhs) const { return !(*this == rhs); }

private:
  ConstMessagePtr message_;
  rclcpp::Time receipt_time_;
};

}  // namespace message_filters
}  // namespace agnocast
