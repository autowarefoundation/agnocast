#pragma once

#include "agnocast/message_filters/simple_filter.hpp"

#include <functional>

namespace agnocast
{
namespace message_filters
{

/**
 * \brief Simple passthrough filter.  What comes in goes out immediately.
 */
template <class M>
class PassThrough : public SimpleFilter<M>
{
public:
  using MConstPtr = ipc_shared_ptr<M const>;
  using EventType = MessageEvent<M const>;

  PassThrough() {}

  template <typename F>
  explicit PassThrough(F & f)
  {
    connectInput(f);
  }

  template <class F>
  void connectInput(F & f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(
      std::bind(&PassThrough::cb, this, std::placeholders::_1)));
  }

  void add(const MConstPtr & msg) { add(EventType(msg)); }

  void add(const EventType & evt) { this->signalMessage(evt); }

private:
  void cb(const EventType & evt) { add(evt); }

  Connection incoming_connection_;
};

}  // namespace message_filters
}  // namespace agnocast
