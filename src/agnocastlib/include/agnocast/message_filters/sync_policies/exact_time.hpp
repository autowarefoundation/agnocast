#pragma once

#include "agnocast/message_filters/signal9.hpp"
#include "agnocast/message_filters/synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <message_filters/connection.h>
#include <message_filters/message_traits.h>
#include <message_filters/null_types.h>

#include <cassert>
#include <map>
#include <mutex>
#include <string>
#include <tuple>

namespace agnocast
{
namespace message_filters
{
namespace sync_policies
{

using ::message_filters::Connection;
using ::message_filters::NullType;

template <
  typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
  typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ExactTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  using Sync = Synchronizer<ExactTime>;
  using Super = PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>;
  using Messages = typename Super::Messages;
  using Signal = typename Super::Signal;
  using Events = typename Super::Events;
  using RealTypeCount = typename Super::RealTypeCount;
  using M0Event = typename Super::M0Event;
  using M1Event = typename Super::M1Event;
  using M2Event = typename Super::M2Event;
  using M3Event = typename Super::M3Event;
  using M4Event = typename Super::M4Event;
  using M5Event = typename Super::M5Event;
  using M6Event = typename Super::M6Event;
  using M7Event = typename Super::M7Event;
  using M8Event = typename Super::M8Event;
  using Tuple = Events;

  explicit ExactTime(uint32_t queue_size) : parent_(nullptr), queue_size_(queue_size) {}

  ExactTime(const ExactTime & e) { *this = e; }

  // cppcheck-suppress operatorEqVarError  // drop_signal_ is not copyable (contains std::mutex)
  ExactTime & operator=(const ExactTime & rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    last_signal_time_ = rhs.last_signal_time_;
    tuples_ = rhs.tuples_;

    return *this;
  }

  void initParent(Sync * parent) { parent_ = parent; }

  template <int i>
  void add(const typename std::tuple_element<i, Events>::type & evt)
  {
    assert(parent_);

    namespace mt = ::message_filters::message_traits;

    std::lock_guard<std::mutex> lock(mutex_);

    Tuple & t = tuples_[mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(
      *evt.getMessage())];
    std::get<i>(t) = evt;

    checkTuple(t);
  }

  template <class C>
  Connection registerDropCallback(const C & callback)
  {
    return drop_signal_.addCallback(callback);
  }

  template <class C>
  Connection registerDropCallback(C & callback)
  {
    return drop_signal_.addCallback(callback);
  }

  template <class C, typename T>
  Connection registerDropCallback(const C & callback, T * t)
  {
    return drop_signal_.addCallback(callback, t);
  }

  template <class C, typename T>
  Connection registerDropCallback(C & callback, T * t)
  {
    return drop_signal_.addCallback(callback, t);
  }

private:
  // assumes mutex_ is already locked
  void checkTuple(Tuple & t)
  {
    namespace mt = ::message_filters::message_traits;

    bool full = true;
    full = full && static_cast<bool>(std::get<0>(t).getMessage());
    full = full && static_cast<bool>(std::get<1>(t).getMessage());
    full = full && (RealTypeCount::value > 2 ? static_cast<bool>(std::get<2>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 3 ? static_cast<bool>(std::get<3>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 4 ? static_cast<bool>(std::get<4>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 5 ? static_cast<bool>(std::get<5>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 6 ? static_cast<bool>(std::get<6>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 7 ? static_cast<bool>(std::get<7>(t).getMessage())
                                             : true);  // NOLINT
    full = full && (RealTypeCount::value > 8 ? static_cast<bool>(std::get<8>(t).getMessage())
                                             : true);  // NOLINT

    if (full) {
      parent_->signal(
        std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t), std::get<4>(t),
        std::get<5>(t), std::get<6>(t), std::get<7>(t), std::get<8>(t));

      last_signal_time_ = mt::TimeStamp<M0>::value(*std::get<0>(t).getMessage());

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0) {
      while (tuples_.size() > queue_size_) {
        Tuple & t2 = tuples_.begin()->second;
        drop_signal_.call(
          std::get<0>(t2), std::get<1>(t2), std::get<2>(t2), std::get<3>(t2), std::get<4>(t2),
          std::get<5>(t2), std::get<6>(t2), std::get<7>(t2), std::get<8>(t2));
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes mutex_ is already locked
  void clearOldTuples()
  {
    auto it = tuples_.begin();
    auto end = tuples_.end();
    for (; it != end;) {
      if (it->first <= last_signal_time_) {
        auto old = it;
        ++it;

        Tuple & t = old->second;
        drop_signal_.call(
          std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t), std::get<4>(t),
          std::get<5>(t), std::get<6>(t), std::get<7>(t), std::get<8>(t));
        tuples_.erase(old);
      } else {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

private:
  Sync * parent_;

  uint32_t queue_size_;
  using M_TimeToTuple = std::map<rclcpp::Time, Tuple>;
  M_TimeToTuple tuples_;
  rclcpp::Time last_signal_time_;

  Signal drop_signal_;

  std::mutex mutex_;
};

}  // namespace sync_policies
}  // namespace message_filters
}  // namespace agnocast
