#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/message_filters/message_event.hpp"
#include "agnocast/message_filters/signal9.hpp"

#include <message_filters/connection.h>
#include <message_filters/null_types.h>
#include <message_filters/synchronizer.h>

#include <cstdint>
#include <string>
#include <tuple>
#include <type_traits>

namespace agnocast
{
namespace message_filters
{

using ::message_filters::Connection;
using ::message_filters::noncopyable;
using ::message_filters::NullFilter;
using ::message_filters::NullType;

/**
 * @brief Synchronizes messages from 2–9 input filters based on a time policy.
 *
 * Drop-in replacement for `message_filters::Synchronizer<Policy>`. When matching messages are
 * found according to the policy, the registered callback is invoked with one
 * `agnocast::ipc_shared_ptr<const M>` per input.
 *
 * @tparam Policy Sync policy type (`ExactTime` or `ApproximateTime`).
 */
AGNOCAST_PUBLIC
template <class Policy>
class Synchronizer : public noncopyable, public Policy
{
public:
  using Messages = typename Policy::Messages;
  using Events = typename Policy::Events;
  using Signal = typename Policy::Signal;

  using M0 = typename std::tuple_element<0, Messages>::type;
  using M1 = typename std::tuple_element<1, Messages>::type;
  using M2 = typename std::tuple_element<2, Messages>::type;
  using M3 = typename std::tuple_element<3, Messages>::type;
  using M4 = typename std::tuple_element<4, Messages>::type;
  using M5 = typename std::tuple_element<5, Messages>::type;
  using M6 = typename std::tuple_element<6, Messages>::type;
  using M7 = typename std::tuple_element<7, Messages>::type;
  using M8 = typename std::tuple_element<8, Messages>::type;

  using M0Event = typename std::tuple_element<0, Events>::type;
  using M1Event = typename std::tuple_element<1, Events>::type;
  using M2Event = typename std::tuple_element<2, Events>::type;
  using M3Event = typename std::tuple_element<3, Events>::type;
  using M4Event = typename std::tuple_element<4, Events>::type;
  using M5Event = typename std::tuple_element<5, Events>::type;
  using M6Event = typename std::tuple_element<6, Events>::type;
  using M7Event = typename std::tuple_element<7, Events>::type;
  using M8Event = typename std::tuple_element<8, Events>::type;

  static const uint8_t MAX_MESSAGES = 9;

  /// Construct a synchronizer with 2-9 input filters.
  template <class F0, class F1>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1)
  {
    connectInput(f0, f1);
    init();
  }

  template <class F0, class F1, class F2>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1, F2 & f2)
  {
    connectInput(f0, f1, f2);
    init();
  }

  template <class F0, class F1, class F2, class F3>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3)
  {
    connectInput(f0, f1, f2, f3);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4)
  {
    connectInput(f0, f1, f2, f3, f4);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5)
  {
    connectInput(f0, f1, f2, f3, f4, f5);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  AGNOCAST_PUBLIC Synchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  AGNOCAST_PUBLIC Synchronizer(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
    init();
  }

  template <
    class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  AGNOCAST_PUBLIC Synchronizer(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7, F8 & f8)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init();
  }

  /// Construct an unconnected synchronizer. Call connectInput() to connect filters.
  AGNOCAST_PUBLIC Synchronizer() { init(); }

  /// Construct a synchronizer with a policy and 2–9 input filters.
  /// @param policy Sync policy instance.
  /// @param f0 First input filter.
  /// @param f1 Second input filter.
  template <class F0, class F1>
  AGNOCAST_PUBLIC Synchronizer(const Policy & policy, F0 & f0, F1 & f1) : Policy(policy)
  {
    connectInput(f0, f1);
    init();
  }

  template <class F0, class F1, class F2>
  AGNOCAST_PUBLIC Synchronizer(const Policy & policy, F0 & f0, F1 & f1, F2 & f2) : Policy(policy)
  {
    connectInput(f0, f1, f2);
    init();
  }

  template <class F0, class F1, class F2, class F3>
  AGNOCAST_PUBLIC Synchronizer(const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4>
  AGNOCAST_PUBLIC Synchronizer(const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5>
  AGNOCAST_PUBLIC Synchronizer(
    const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  AGNOCAST_PUBLIC Synchronizer(
    const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6);
    init();
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  AGNOCAST_PUBLIC Synchronizer(
    const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
    init();
  }

  template <
    class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  AGNOCAST_PUBLIC Synchronizer(
    const Policy & policy, F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7,
    F8 & f8)
  : Policy(policy)
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    init();
  }

  /// Construct a synchronizer with a policy but no input filters.
  /// @param policy Sync policy instance.
  AGNOCAST_PUBLIC explicit Synchronizer(const Policy & policy) : Policy(policy) { init(); }

  ~Synchronizer() { disconnectAll(); }

  void init() { Policy::initParent(this); }

  /// Connect 2–9 input filters to this synchronizer. Replaces any previous connections.
  /// @param f0 First input filter.
  /// @param f1 Second input filter.
  template <class F0, class F1>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1)
  {
    NullFilter<M2> f2;
    connectInput(f0, f1, f2);
  }

  template <class F0, class F1, class F2>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1, F2 & f2)
  {
    NullFilter<M3> f3;
    connectInput(f0, f1, f2, f3);
  }

  template <class F0, class F1, class F2, class F3>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1, F2 & f2, F3 & f3)
  {
    NullFilter<M4> f4;
    connectInput(f0, f1, f2, f3, f4);
  }

  template <class F0, class F1, class F2, class F3, class F4>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4)
  {
    NullFilter<M5> f5;
    connectInput(f0, f1, f2, f3, f4, f5);
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5)
  {
    NullFilter<M6> f6;
    connectInput(f0, f1, f2, f3, f4, f5, f6);
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  AGNOCAST_PUBLIC void connectInput(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6)
  {
    NullFilter<M7> f7;
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template <class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  AGNOCAST_PUBLIC void connectInput(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7)
  {
    NullFilter<M8> f8;
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  template <
    class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  AGNOCAST_PUBLIC void connectInput(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7, F8 & f8)
  {
    disconnectAll();

    input_connections_[0] = f0.registerCallback(std::function<void(const M0Event &)>(
      std::bind(&Synchronizer::template cb<0>, this, std::placeholders::_1)));
    input_connections_[1] = f1.registerCallback(std::function<void(const M1Event &)>(
      std::bind(&Synchronizer::template cb<1>, this, std::placeholders::_1)));
    input_connections_[2] = f2.registerCallback(std::function<void(const M2Event &)>(
      std::bind(&Synchronizer::template cb<2>, this, std::placeholders::_1)));
    input_connections_[3] = f3.registerCallback(std::function<void(const M3Event &)>(
      std::bind(&Synchronizer::template cb<3>, this, std::placeholders::_1)));
    input_connections_[4] = f4.registerCallback(std::function<void(const M4Event &)>(
      std::bind(&Synchronizer::template cb<4>, this, std::placeholders::_1)));
    input_connections_[5] = f5.registerCallback(std::function<void(const M5Event &)>(
      std::bind(&Synchronizer::template cb<5>, this, std::placeholders::_1)));
    input_connections_[6] = f6.registerCallback(std::function<void(const M6Event &)>(
      std::bind(&Synchronizer::template cb<6>, this, std::placeholders::_1)));
    input_connections_[7] = f7.registerCallback(std::function<void(const M7Event &)>(
      std::bind(&Synchronizer::template cb<7>, this, std::placeholders::_1)));
    input_connections_[8] = f8.registerCallback(std::function<void(const M8Event &)>(
      std::bind(&Synchronizer::template cb<8>, this, std::placeholders::_1)));
  }

  /// Register a callback invoked when matching messages are found.
  /// @param callback Callback to register.
  /// @return Connection object for disconnecting.
  template <class C>
  AGNOCAST_PUBLIC Connection registerCallback(C & callback)
  {
    return signal_.addCallback(callback);
  }

  /// Register a const callback.
  /// @param callback Callback to register.
  /// @return Connection object.
  template <class C>
  AGNOCAST_PUBLIC Connection registerCallback(const C & callback)
  {
    return signal_.addCallback(callback);
  }

  /// Register a member function callback.
  /// @param callback Member function pointer.
  /// @param t Object to call the member function on.
  /// @return Connection object.
  template <class C, typename T>
  AGNOCAST_PUBLIC Connection registerCallback(const C & callback, T * t)
  {
    return signal_.addCallback(callback, t);
  }

  /// Register a member function callback.
  /// @param callback Member function pointer.
  /// @param t Object to call the member function on.
  /// @return Connection object.
  template <class C, typename T>
  AGNOCAST_PUBLIC Connection registerCallback(C & callback, T * t)
  {
    return signal_.addCallback(callback, t);
  }

  /// Set the name of this synchronizer (for debugging).
  /// @param name Name string.
  AGNOCAST_PUBLIC void setName(const std::string & name) { name_ = name; }

  /// Return the name of this synchronizer.
  /// @return Name string.
  AGNOCAST_PUBLIC const std::string & getName() { return name_; }

  void signal(
    const M0Event & e0, const M1Event & e1, const M2Event & e2, const M3Event & e3,
    const M4Event & e4, const M5Event & e5, const M6Event & e6, const M7Event & e7,
    const M8Event & e8)
  {
    signal_.call(e0, e1, e2, e3, e4, e5, e6, e7, e8);
  }

  /// Return a pointer to the sync policy. Use this to configure policy parameters after
  /// construction (e.g., `sync.getPolicy()->setAgePenalty(0.5)`).
  /// @return Pointer to the policy object.
  AGNOCAST_PUBLIC Policy * getPolicy() { return static_cast<Policy *>(this); }

  using Policy::add;

  template <int i>
  void add(const ipc_shared_ptr<typename std::tuple_element<i, Messages>::type const> & msg)
  {
    this->template add<i>(typename std::tuple_element<i, Events>::type(msg));
  }

private:
  void disconnectAll()
  {
    for (int i = 0; i < MAX_MESSAGES; ++i) {
      input_connections_[i].disconnect();
    }
  }

  template <int i>
  void cb(const typename std::tuple_element<i, Events>::type & evt)
  {
    this->template add<i>(evt);
  }

  Signal signal_;

  Connection input_connections_[MAX_MESSAGES];

  std::string name_;
};

template <
  typename M0, typename M1, typename M2, typename M3, typename M4, typename M5, typename M6,
  typename M7, typename M8>
struct PolicyBase
{
  using RealTypeCount = typename ::message_filters::mp_count<
    std::tuple<M0, M1, M2, M3, M4, M5, M6, M7, M8>, NullType>::type;
  using Messages = std::tuple<M0, M1, M2, M3, M4, M5, M6, M7, M8>;
  using Signal = Signal9<M0, M1, M2, M3, M4, M5, M6, M7, M8>;
  using Events = std::tuple<
    MessageEvent<M0 const>, MessageEvent<M1 const>, MessageEvent<M2 const>, MessageEvent<M3 const>,
    MessageEvent<M4 const>, MessageEvent<M5 const>, MessageEvent<M6 const>, MessageEvent<M7 const>,
    MessageEvent<M8 const>>;

  using M0Event = typename std::tuple_element<0, Events>::type;
  using M1Event = typename std::tuple_element<1, Events>::type;
  using M2Event = typename std::tuple_element<2, Events>::type;
  using M3Event = typename std::tuple_element<3, Events>::type;
  using M4Event = typename std::tuple_element<4, Events>::type;
  using M5Event = typename std::tuple_element<5, Events>::type;
  using M6Event = typename std::tuple_element<6, Events>::type;
  using M7Event = typename std::tuple_element<7, Events>::type;
  using M8Event = typename std::tuple_element<8, Events>::type;
};

}  // namespace message_filters
}  // namespace agnocast
