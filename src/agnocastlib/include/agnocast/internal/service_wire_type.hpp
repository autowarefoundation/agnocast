// On-the-wire layout of Agnocast service request/response messages.
//
// ┌───────────┬────────────────────────┐
// │           │  correlation metadata  │
// │  payload  ├─────────┬──────────────┤
// │           │  seqno  │  node_name   │
// └───────────┼─────────┴──────────────┘
//             └ 16-byte aligned
//
// payload: request/response payload (ServiceT::Request or ServiceT::Response).
//
// seqno: A sequence number to identify each service call (int64_t). To avoid nastiness that may be
// caused by compiler-inserted padding, this field is conservatively aligned to 16 bytes.
//
// node_name: The node name of the caller (std::string). This field only exists in the request.

#pragma once

#include "agnocast/agnocast_smart_pointer.hpp"

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <string>

namespace
{

constexpr size_t offsetof_meta(size_t base_size)
{
  return (base_size + 15) & (~15);
}

}  // namespace

namespace agnocast
{

template <typename ServiceT>
struct ServiceRequestWrapper : public ServiceT::Request
{
  alignas(16) int64_t seqno;
  std::string node_name;
};

template <typename ServiceT>
struct ServiceResponseWrapper : public ServiceT::Response
{
  alignas(16) int64_t seqno;
};

class GenericRequestWrapper
{
  struct Meta
  {
    alignas(16) int64_t seqno;
    std::string node_name;
  };

  static constexpr size_t get_wire_size(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members)
  {
    return offsetof_meta(request_members->size_of_) + sizeof(Meta);
  }

  static Meta * get_meta_ptr(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members,
    void * request_ptr)
  {
    return reinterpret_cast<Meta *>(
      static_cast<char *>(request_ptr) + offsetof_meta(request_members->size_of_));
  }

  ipc_shared_ptr<void> request_;
  Meta * const meta_ptr_;

public:
  explicit GenericRequestWrapper(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members,
    ipc_shared_ptr<void> && request)
  : request_(std::move(request)), meta_ptr_(get_meta_ptr(request_members, request_.get()))
  {
  }

  int64_t & seqno() { return meta_ptr_->seqno; }
  std::string & node_name() { return meta_ptr_->node_name; }

  ipc_shared_ptr<void> && take_request() && { return std::move(request_); }

  /// @brief Allocate a wire-format request buffer in shared memory.
  ///
  /// The payload buffer is initialized via the introspection init function with
  /// MessageInitialization::SKIP. The seqno number is uninitialized (garbage), and the node_name
  /// string is default-constructed via placement new.
  ///
  /// @param request_members The introspection message members for the request type.
  /// @param borrow_loaned_message A callable that allocates a buffer of the given size in shared
  /// memory. In practice, this should be TypeErasedPublisher::borrow_loaned_message().
  template <typename Func>
  static GenericRequestWrapper allocate(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members,
    Func && borrow_loaned_message)
  {
    ipc_shared_ptr<void> request = borrow_loaned_message(get_wire_size(request_members));
    auto wrapper = GenericRequestWrapper(request_members, std::move(request));

    request_members->init_function(
      wrapper.request_.get(), rosidl_runtime_cpp::MessageInitialization::SKIP);

    // Use placement new to construct node_name.
    auto * node_name_ptr = &wrapper.node_name();
    new (node_name_ptr) std::string{};

    return wrapper;
  }

  /// @brief Free a wire-format request buffer in shared memory.
  /// @param ptr Pointer to the request buffer.
  /// @param request_members The introspection message members for the request type.
  static void free(
    void * ptr, const rosidl_typesupport_introspection_cpp::MessageMembers * request_members)
  {
    request_members->fini_function(ptr);

    // Destroy node_name by calling std::destroy_at().
    Meta * meta_ptr = get_meta_ptr(request_members, ptr);
    auto * node_name_ptr = &meta_ptr->node_name;
    std::destroy_at(node_name_ptr);

    ::operator delete(ptr);
  }
};

class GenericResponseWrapper
{
  struct Meta
  {
    alignas(16) int64_t seqno;
  };

  static Meta * get_meta_ptr(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members,
    void * response_ptr)
  {
    return reinterpret_cast<Meta *>(
      static_cast<char *>(response_ptr) + offsetof_meta(response_members->size_of_));
  }

  ipc_shared_ptr<void> response_;
  Meta * const meta_ptr_;

public:
  explicit GenericResponseWrapper(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members,
    ipc_shared_ptr<void> && response)
  : response_(std::move(response)), meta_ptr_(get_meta_ptr(response_members, response_.get()))
  {
  }

  int64_t & seqno() { return meta_ptr_->seqno; }

  ipc_shared_ptr<void> && take_response() && { return std::move(response_); }
};

}  // namespace agnocast
