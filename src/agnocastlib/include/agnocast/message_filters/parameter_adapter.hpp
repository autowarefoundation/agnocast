#pragma once

#include "agnocast/message_filters/message_event.hpp"

#include <message_filters/null_types.h>

#include <memory>
#include <type_traits>

namespace agnocast
{
namespace message_filters
{

/**
 * \brief Adapts a function parameter type into the message type, event type and parameter.
 *
 * In agnocast, all messages are const (ipc_shared_ptr points to read-only shared memory).
 * Supported callback parameter types:
 *   - const ipc_shared_ptr<M const>&
 *   - ipc_shared_ptr<M const>
 *   - const MessageEvent<M const>&
 */
template <typename M>
struct ParameterAdapter;

// const ipc_shared_ptr<M const>&
template <typename M>
struct ParameterAdapter<const ipc_shared_ptr<M const> &>
{
  using Message = typename std::remove_reference<typename std::remove_const<M>::type>::type;
  using Event = MessageEvent<Message const>;
  using Parameter = const ipc_shared_ptr<Message const> &;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event) { return event.getMessage(); }
};

// ipc_shared_ptr<M const>
template <typename M>
struct ParameterAdapter<ipc_shared_ptr<M const>>
{
  using Message = typename std::remove_reference<typename std::remove_const<M>::type>::type;
  using Event = MessageEvent<Message const>;
  using Parameter = ipc_shared_ptr<Message const>;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event) { return event.getMessage(); }
};

// const MessageEvent<M const>&
template <typename M>
struct ParameterAdapter<const MessageEvent<M const> &>
{
  using Message = typename std::remove_reference<typename std::remove_const<M>::type>::type;
  using Event = MessageEvent<Message const>;
  using Parameter = const MessageEvent<Message const> &;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event) { return event; }
};

// Specialization for std::shared_ptr<NullType const> (used internally for unused message slots)
template <>
struct ParameterAdapter<const std::shared_ptr<::message_filters::NullType const> &>
{
  using Message = ::message_filters::NullType;
  using Event = MessageEvent<Message const>;
  using Parameter = const std::shared_ptr<Message const> &;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    static std::shared_ptr<Message const> null_ptr;
    (void)event;
    return null_ptr;
  }
};

}  // namespace message_filters
}  // namespace agnocast
