#pragma once

#include "agnocast/message_filters/message_event.hpp"
#include "agnocast/message_filters/parameter_adapter.hpp"

#include <message_filters/connection.h>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{
namespace message_filters
{

using ::message_filters::Connection;

template <class M>
class CallbackHelper1
{
public:
  virtual ~CallbackHelper1() {}

  virtual void call(const MessageEvent<M const> & event) = 0;

  using Ptr = std::shared_ptr<CallbackHelper1<M>>;
};

template <typename P, typename M>
class CallbackHelper1T : public CallbackHelper1<M>
{
public:
  using Adapter = ParameterAdapter<P>;
  using Callback = std::function<void(typename Adapter::Parameter)>;
  using Event = typename Adapter::Event;

  explicit CallbackHelper1T(const Callback & cb) : callback_(cb) {}

  void call(const MessageEvent<M const> & event) override
  {
    callback_(Adapter::getParameter(event));
  }

private:
  Callback callback_;
};

template <class M>
class Signal1
{
  using CallbackHelper1Ptr = std::shared_ptr<CallbackHelper1<M>>;
  using V_CallbackHelper1 = std::vector<CallbackHelper1Ptr>;

public:
  template <typename P>
  CallbackHelper1Ptr addCallback(const std::function<void(P)> & callback)
  {
    CallbackHelper1T<P, M> * helper = new CallbackHelper1T<P, M>(callback);

    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(CallbackHelper1Ptr(helper));
    return callbacks_.back();
  }

  void removeCallback(const CallbackHelper1Ptr & helper)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end()) {
      callbacks_.erase(it);
    }
  }

  void call(const MessageEvent<M const> & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & helper : callbacks_) {
      helper->call(event);
    }
  }

private:
  std::mutex mutex_;
  V_CallbackHelper1 callbacks_;
};

}  // namespace message_filters
}  // namespace agnocast
