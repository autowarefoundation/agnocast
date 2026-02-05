#pragma once

#include "agnocast/message_filters/message_event.hpp"
#include "agnocast/message_filters/parameter_adapter.hpp"

#include <message_filters/connection.h>
#include <message_filters/null_types.h>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{
namespace message_filters
{

using ::message_filters::Connection;
using ::message_filters::NullType;

template <
  typename M0, typename M1, typename M2, typename M3, typename M4, typename M5, typename M6,
  typename M7, typename M8>
class CallbackHelper9
{
public:
  using M0Event = MessageEvent<M0 const>;
  using M1Event = MessageEvent<M1 const>;
  using M2Event = MessageEvent<M2 const>;
  using M3Event = MessageEvent<M3 const>;
  using M4Event = MessageEvent<M4 const>;
  using M5Event = MessageEvent<M5 const>;
  using M6Event = MessageEvent<M6 const>;
  using M7Event = MessageEvent<M7 const>;
  using M8Event = MessageEvent<M8 const>;

  virtual ~CallbackHelper9() {}

  virtual void call(
    const M0Event & e0, const M1Event & e1, const M2Event & e2, const M3Event & e3,
    const M4Event & e4, const M5Event & e5, const M6Event & e6, const M7Event & e7,
    const M8Event & e8) = 0;

  using Ptr = std::shared_ptr<CallbackHelper9>;
};

template <
  typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6,
  typename P7, typename P8>
class CallbackHelper9T
: public CallbackHelper9<
    typename ParameterAdapter<P0>::Message, typename ParameterAdapter<P1>::Message,
    typename ParameterAdapter<P2>::Message, typename ParameterAdapter<P3>::Message,
    typename ParameterAdapter<P4>::Message, typename ParameterAdapter<P5>::Message,
    typename ParameterAdapter<P6>::Message, typename ParameterAdapter<P7>::Message,
    typename ParameterAdapter<P8>::Message>
{
private:
  using A0 = ParameterAdapter<P0>;
  using A1 = ParameterAdapter<P1>;
  using A2 = ParameterAdapter<P2>;
  using A3 = ParameterAdapter<P3>;
  using A4 = ParameterAdapter<P4>;
  using A5 = ParameterAdapter<P5>;
  using A6 = ParameterAdapter<P6>;
  using A7 = ParameterAdapter<P7>;
  using A8 = ParameterAdapter<P8>;
  using M0Event = typename A0::Event;
  using M1Event = typename A1::Event;
  using M2Event = typename A2::Event;
  using M3Event = typename A3::Event;
  using M4Event = typename A4::Event;
  using M5Event = typename A5::Event;
  using M6Event = typename A6::Event;
  using M7Event = typename A7::Event;
  using M8Event = typename A8::Event;

public:
  using Callback = std::function<void(
    typename A0::Parameter, typename A1::Parameter, typename A2::Parameter, typename A3::Parameter,
    typename A4::Parameter, typename A5::Parameter, typename A6::Parameter, typename A7::Parameter,
    typename A8::Parameter)>;

  explicit CallbackHelper9T(const Callback & cb) : callback_(cb) {}

  void call(
    const M0Event & e0, const M1Event & e1, const M2Event & e2, const M3Event & e3,
    const M4Event & e4, const M5Event & e5, const M6Event & e6, const M7Event & e7,
    const M8Event & e8) override
  {
    callback_(
      A0::getParameter(e0), A1::getParameter(e1), A2::getParameter(e2), A3::getParameter(e3),
      A4::getParameter(e4), A5::getParameter(e5), A6::getParameter(e6), A7::getParameter(e7),
      A8::getParameter(e8));
  }

private:
  Callback callback_;
};

template <
  typename M0, typename M1, typename M2, typename M3, typename M4, typename M5, typename M6,
  typename M7, typename M8>
class Signal9
{
  using CallbackHelper9Ptr = std::shared_ptr<CallbackHelper9<M0, M1, M2, M3, M4, M5, M6, M7, M8>>;
  using V_CallbackHelper9 = std::vector<CallbackHelper9Ptr>;

public:
  using M0Event = MessageEvent<M0 const>;
  using M1Event = MessageEvent<M1 const>;
  using M2Event = MessageEvent<M2 const>;
  using M3Event = MessageEvent<M3 const>;
  using M4Event = MessageEvent<M4 const>;
  using M5Event = MessageEvent<M5 const>;
  using M6Event = MessageEvent<M6 const>;
  using M7Event = MessageEvent<M7 const>;
  using M8Event = MessageEvent<M8 const>;
  using M0ConstPtr = ipc_shared_ptr<M0 const>;
  using M1ConstPtr = ipc_shared_ptr<M1 const>;
  using M2ConstPtr = ipc_shared_ptr<M2 const>;
  using M3ConstPtr = ipc_shared_ptr<M3 const>;
  using M4ConstPtr = ipc_shared_ptr<M4 const>;
  using M5ConstPtr = ipc_shared_ptr<M5 const>;
  using M6ConstPtr = ipc_shared_ptr<M6 const>;
  using M7ConstPtr = ipc_shared_ptr<M7 const>;
  using M8ConstPtr = ipc_shared_ptr<M8 const>;
  using NullP = const std::shared_ptr<NullType const> &;

  template <
    typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6,
    typename P7, typename P8>
  Connection addCallback(const std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, P8)> & callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(
      std::make_shared<CallbackHelper9T<P0, P1, P2, P3, P4, P5, P6, P7, P8>>(callback));
    return Connection(std::bind(&Signal9::removeCallback, this, callbacks_.back()));
  }

  template <typename P0, typename P1>
  Connection addCallback(void (*callback)(P0, P1))
  {
    return addCallback(std::function<void(P0, P1, NullP, NullP, NullP, NullP, NullP, NullP, NullP)>(
      std::bind(callback, std::placeholders::_1, std::placeholders::_2)));
  }

  template <typename P0, typename P1, typename P2>
  Connection addCallback(void (*callback)(P0, P1, P2))
  {
    return addCallback(std::function<void(P0, P1, P2, NullP, NullP, NullP, NullP, NullP, NullP)>(
      std::bind(callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));
  }

  template <typename P0, typename P1, typename P2, typename P3>
  Connection addCallback(void (*callback)(P0, P1, P2, P3))
  {
    return addCallback(
      std::function<void(P0, P1, P2, P3, NullP, NullP, NullP, NullP, NullP)>(std::bind(
        callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4)));
  }

  template <typename P0, typename P1, typename P2, typename P3, typename P4>
  Connection addCallback(void (*callback)(P0, P1, P2, P3, P4))
  {
    return addCallback(
      std::function<void(P0, P1, P2, P3, P4, NullP, NullP, NullP, NullP)>(std::bind(
        callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));
  }

  template <typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
  Connection addCallback(void (*callback)(P0, P1, P2, P3, P4, P5))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, NullP, NullP, NullP)>(std::bind(
      callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6)));
  }

  template <
    typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
  Connection addCallback(void (*callback)(P0, P1, P2, P3, P4, P5, P6))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, NullP, NullP)>(std::bind(
      callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7)));
  }

  template <
    typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6,
    typename P7>
  Connection addCallback(void (*callback)(P0, P1, P2, P3, P4, P5, P6, P7))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, NullP)>(std::bind(
      callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
      std::placeholders::_8)));
  }

  template <
    typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6,
    typename P7, typename P8>
  Connection addCallback(void (*callback)(P0, P1, P2, P3, P4, P5, P6, P7, P8))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, P8)>(std::bind(
      callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
      std::placeholders::_8, std::placeholders::_9)));
  }

  template <typename T, typename P0, typename P1>
  Connection addCallback(void (T::*callback)(P0, P1), T * t)
  {
    return addCallback(std::function<void(P0, P1, NullP, NullP, NullP, NullP, NullP, NullP, NullP)>(
      std::bind(callback, t, std::placeholders::_1, std::placeholders::_2)));
  }

  template <typename T, typename P0, typename P1, typename P2>
  Connection addCallback(void (T::*callback)(P0, P1, P2), T * t)
  {
    return addCallback(std::function<void(P0, P1, P2, NullP, NullP, NullP, NullP, NullP, NullP)>(
      std::bind(callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));
  }

  template <typename T, typename P0, typename P1, typename P2, typename P3>
  Connection addCallback(void (T::*callback)(P0, P1, P2, P3), T * t)
  {
    return addCallback(
      std::function<void(P0, P1, P2, P3, NullP, NullP, NullP, NullP, NullP)>(std::bind(
        callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4)));
  }

  template <typename T, typename P0, typename P1, typename P2, typename P3, typename P4>
  Connection addCallback(void (T::*callback)(P0, P1, P2, P3, P4), T * t)
  {
    return addCallback(
      std::function<void(P0, P1, P2, P3, P4, NullP, NullP, NullP, NullP)>(std::bind(
        callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));
  }

  template <
    typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
  Connection addCallback(void (T::*callback)(P0, P1, P2, P3, P4, P5), T * t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, NullP, NullP, NullP)>(std::bind(
      callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6)));
  }

  template <
    typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5,
    typename P6>
  Connection addCallback(void (T::*callback)(P0, P1, P2, P3, P4, P5, P6), T * t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, NullP, NullP)>(std::bind(
      callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7)));
  }

  template <
    typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5,
    typename P6, typename P7>
  Connection addCallback(void (T::*callback)(P0, P1, P2, P3, P4, P5, P6, P7), T * t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, NullP)>(std::bind(
      callback, t, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
      std::placeholders::_8)));
  }

  template <typename C>
  Connection addCallback(C & callback)
  {
    return addCallback<
      const M0ConstPtr &, const M1ConstPtr &, const M2ConstPtr &, const M3ConstPtr &,
      const M4ConstPtr &, const M5ConstPtr &, const M6ConstPtr &, const M7ConstPtr &,
      const M8ConstPtr &>(std::bind(
      callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
      std::placeholders::_8, std::placeholders::_9));
  }

  void removeCallback(const CallbackHelper9Ptr & helper)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end()) {
      callbacks_.erase(it);
    }
  }

  void call(
    const M0Event & e0, const M1Event & e1, const M2Event & e2, const M3Event & e3,
    const M4Event & e4, const M5Event & e5, const M6Event & e6, const M7Event & e7,
    const M8Event & e8)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & helper : callbacks_) {
      helper->call(e0, e1, e2, e3, e4, e5, e6, e7, e8);
    }
  }

private:
  std::mutex mutex_;
  V_CallbackHelper9 callbacks_;
};

}  // namespace message_filters
}  // namespace agnocast
