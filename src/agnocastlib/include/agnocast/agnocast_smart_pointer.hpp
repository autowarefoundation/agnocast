#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>

namespace agnocast
{

// These are cut out of the class for information hiding.
void decrement_rc(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id);
void increment_rc(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id);

extern int agnocast_fd;

template <typename T>
class ipc_shared_ptr
{
  T * ptr_ = nullptr;
  std::string topic_name_;
  topic_local_id_t pubsub_id_ = -1;
  int64_t entry_id_ = -1;

  // Unimplemented operators. If these are called, a compile error is raised.
  bool operator==(const ipc_shared_ptr & r) const = delete;
  bool operator!=(const ipc_shared_ptr & r) const = delete;

  template <typename U, typename V>
  friend ipc_shared_ptr<U> static_ipc_shared_ptr_cast(ipc_shared_ptr<V> && r);

public:
  using element_type = T;

  const std::string get_topic_name() const { return topic_name_; }
  topic_local_id_t get_pubsub_id() const { return pubsub_id_; }
  int64_t get_entry_id() const { return entry_id_; }
  void set_entry_id(const int64_t entry_id) { entry_id_ = entry_id; }

  ipc_shared_ptr() = default;

  template <typename S>
  explicit ipc_shared_ptr(T * ptr, S && topic_name, const topic_local_id_t pubsub_id)
  : ptr_(ptr), topic_name_(std::forward<S>(topic_name)), pubsub_id_(pubsub_id)
  {
  }

  template <typename S>
  explicit ipc_shared_ptr(
    T * ptr, S && topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id)
  : ptr_(ptr), topic_name_(std::forward<S>(topic_name)), pubsub_id_(pubsub_id), entry_id_(entry_id)
  {
  }

  ~ipc_shared_ptr() { reset(); }

  ipc_shared_ptr(const ipc_shared_ptr & r)
  : ptr_(r.ptr_), topic_name_(r.topic_name_), pubsub_id_(r.pubsub_id_), entry_id_(r.entry_id_)
  {
    if (ptr_ != nullptr) {
      increment_rc(topic_name_, pubsub_id_, entry_id_);
    }
  }

  ipc_shared_ptr & operator=(const ipc_shared_ptr & r)
  {
    if (this != &r) {
      reset();
      ptr_ = r.ptr_;
      topic_name_ = r.topic_name_;
      pubsub_id_ = r.pubsub_id_;
      entry_id_ = r.entry_id_;
      if (ptr_ != nullptr) {
        increment_rc(topic_name_, pubsub_id_, entry_id_);
      }
    }
    return *this;
  }

  ipc_shared_ptr(ipc_shared_ptr && r)
  : ptr_(r.ptr_), topic_name_(r.topic_name_), pubsub_id_(r.pubsub_id_), entry_id_(r.entry_id_)
  {
    r.ptr_ = nullptr;
  }

  ipc_shared_ptr & operator=(ipc_shared_ptr && r)
  {
    if (this != &r) {
      reset();
      ptr_ = r.ptr_;
      topic_name_ = r.topic_name_;
      pubsub_id_ = r.pubsub_id_;
      entry_id_ = r.entry_id_;

      r.ptr_ = nullptr;
    }
    return *this;
  }

  T & operator*() const noexcept { return *ptr_; }

  T * operator->() const noexcept { return ptr_; }

  operator bool() const noexcept { return ptr_; }

  T * get() const noexcept { return ptr_; }

  void reset()
  {
    if (ptr_ == nullptr) return;

    // Do not call decrement_rc() when entry_id is -1 since there is no corresponding
    // entry_node in kmod. This happens when reset() is called after borrow_loaned_message()
    // without calling publish() (i.e., the borrowed message was never published).
    if (entry_id_ != -1) {
      decrement_rc(topic_name_, pubsub_id_, entry_id_);
    }

    ptr_ = nullptr;
  }
};

template <typename T, typename U>
ipc_shared_ptr<T> static_ipc_shared_ptr_cast(ipc_shared_ptr<U> && r)
{
  T * ptr = static_cast<T *>(r.get());
  topic_local_id_t pubsub_id = r.get_pubsub_id();
  int64_t entry_id = r.get_entry_id();
  std::string topic_name = std::move(r.topic_name_);

  // Prevent decrement_rc() from being called.
  r.ptr_ = nullptr;

  return ipc_shared_ptr<T>{ptr, std::move(topic_name), pubsub_id, entry_id};
}

}  // namespace agnocast
