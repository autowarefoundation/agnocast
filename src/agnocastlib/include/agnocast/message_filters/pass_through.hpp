#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/message_filters/simple_filter.hpp"

#include <functional>

namespace agnocast
{
namespace message_filters
{

/**
 * \brief Simple passthrough filter.  What comes in goes out immediately.
 */
AGNOCAST_PUBLIC
template <class M>
class PassThrough : public SimpleFilter<M>
{
public:
  using MConstPtr = ipc_shared_ptr<M const>;
  using EventType = MessageEvent<M const>;

  /// Construct an unconnected PassThrough filter.
  AGNOCAST_PUBLIC PassThrough() {}

  /// Construct and connect to an upstream filter.
  /// @param f Upstream filter to connect.
  template <typename F>
  AGNOCAST_PUBLIC explicit PassThrough(F & f)
  {
    connectInput(f);
  }

  /// Connect an upstream filter so its output is forwarded through this PassThrough.
  /// @param f Upstream filter.
  template <class F>
  AGNOCAST_PUBLIC void connectInput(F & f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(
      std::bind(&PassThrough::cb, this, std::placeholders::_1)));
  }

  /// Feed a message to all downstream filters.
  /// @param msg Message to forward.
  AGNOCAST_PUBLIC void add(const MConstPtr & msg) { add(EventType(msg)); }

  /// Feed a message event to all downstream filters.
  /// @param evt Event to forward.
  AGNOCAST_PUBLIC void add(const EventType & evt) { this->signalMessage(evt); }

private:
  void cb(const EventType & evt) { add(evt); }

  Connection incoming_connection_;
};

}  // namespace message_filters
}  // namespace agnocast
