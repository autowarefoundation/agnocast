#pragma once

#include "agnocast/message_filters/message_event.hpp"
#include "agnocast/message_filters/signal1.hpp"

#include <message_filters/connection.h>

#include <functional>
#include <memory>
#include <string>

namespace agnocast
{
namespace message_filters
{

using ::message_filters::Connection;
using ::message_filters::noncopyable;

/**
 * \brief Convenience base-class for simple filters which output a single message
 *
 * SimpleFilter provides some of the tricky callback registering functionality, so that
 * simple filters do not have to duplicate it.  It also provides getName()/setName() for debugging
 * purposes.
 */
template <class M>
class SimpleFilter : public noncopyable
{
public:
  using MConstPtr = ipc_shared_ptr<M const>;
  using Callback = std::function<void(const MConstPtr &)>;
  using EventType = MessageEvent<M const>;
  using EventCallback = std::function<void(const EventType &)>;

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template <typename C>
  Connection registerCallback(const C & callback)
  {
    typename CallbackHelper1<M>::Ptr helper = signal_.addCallback(Callback(callback));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template <typename P>
  Connection registerCallback(const std::function<void(P)> & callback)
  {
    return Connection(std::bind(&Signal::removeCallback, &signal_, signal_.addCallback(callback)));
  }

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template <typename P>
  Connection registerCallback(void (*callback)(P))
  {
    typename CallbackHelper1<M>::Ptr helper =
      signal_.template addCallback<P>(std::bind(callback, std::placeholders::_1));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template <typename T, typename P>
  Connection registerCallback(void (T::*callback)(P), T * t)
  {
    typename CallbackHelper1<M>::Ptr helper =
      signal_.template addCallback<P>(std::bind(callback, t, std::placeholders::_1));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Set the name of this filter.  For debugging use.
   */
  void setName(const std::string & name) { name_ = name; }

  /**
   * \brief Get the name of this filter.  For debugging use.
   */
  const std::string & getName() const { return name_; }

protected:
  /**
   * \brief Call all registered callbacks, passing them the specified message
   */
  void signalMessage(const MConstPtr & msg)
  {
    MessageEvent<M const> event(msg);
    signal_.call(event);
  }

  /**
   * \brief Call all registered callbacks, passing them the specified message
   */
  void signalMessage(const MessageEvent<M const> & event) { signal_.call(event); }

private:
  using Signal = Signal1<M>;

  Signal signal_;
  std::string name_;
};

}  // namespace message_filters
}  // namespace agnocast
