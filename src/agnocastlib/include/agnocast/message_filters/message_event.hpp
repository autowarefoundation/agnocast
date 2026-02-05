#pragma once

#include "agnocast/agnocast_smart_pointer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <message_filters/message_event.h>

#include <memory>
#include <type_traits>

namespace agnocast
{
namespace message_filters
{

/**
 * \brief Event type for subscriptions in agnocast message_filters.
 *
 * In agnocast, ipc_shared_ptr always points to read-only shared memory,
 * so MessageEvent only supports const messages (M const).
 * Non-const M is not supported - use MessageEvent<M const> instead.
 */
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

  MessageEvent() = default;

  MessageEvent(const MessageEvent & rhs) = default;

  explicit MessageEvent(const ConstMessagePtr & message)
  : message_(message), receipt_time_(rclcpp::Clock().now())
  {
  }

  MessageEvent(const ConstMessagePtr & message, rclcpp::Time receipt_time)
  : message_(message), receipt_time_(receipt_time)
  {
  }

  MessageEvent & operator=(const MessageEvent & rhs) = default;

  /**
   * \brief Retrieve the message.
   * Returns ipc_shared_ptr<M const> pointing to shared memory.
   */
  const ConstMessagePtr & getMessage() const { return message_; }

  /**
   * \brief Retrieve a const version of the message (same as getMessage() in agnocast)
   */
  const ConstMessagePtr & getConstMessage() const { return message_; }

  /**
   * \brief Returns the time at which this message was received
   */
  rclcpp::Time getReceiptTime() const { return receipt_time_; }

  bool operator<(const MessageEvent & rhs) const
  {
    if (message_.get() != rhs.message_.get()) {
      return message_.get() < rhs.message_.get();
    }

    return receipt_time_ < rhs.receipt_time_;
  }

  bool operator==(const MessageEvent & rhs) const
  {
    return message_.get() == rhs.message_.get() && receipt_time_ == rhs.receipt_time_;
  }

  bool operator!=(const MessageEvent & rhs) const { return !(*this == rhs); }

private:
  ConstMessagePtr message_;
  rclcpp::Time receipt_time_;
};

}  // namespace message_filters
}  // namespace agnocast
