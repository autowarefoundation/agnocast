// On-the-wire layout of Agnocast service request/response messages.
//
// ┌───────────┬───────────────────────────────────────────────┐
// │           │             correlation metadata              │
// │  payload  ├─────────┬──────────────┬──────────────────────┤
// │           │  seqno  │  client_gid  │ response_topic_name  │
// └───────────┼─────────┴──────────────┴──────────────────────┘
//             └ 16-byte aligned
//
// payload: request/response payload (ServiceT::Request or ServiceT::Response).
//
// seqno: A sequence number to identify each service call (int64_t). To avoid nastiness that may be
// caused by compiler-inserted padding, this field is conservatively aligned to 16 bytes.
//
// client_gid: The client GID (uint8_t array of length RMW_GID_STORAGE_SIZE). This field only exists
// in the request.
//
// response_topic_name: The topic the caller listens for its response on (std::string). This field
// only exists in the request. A renaming domain bridge gives the two sides different service names,
// so the server publishes to this name rather than deriving one.

#pragma once

#include "agnocast/agnocast_smart_pointer.hpp"

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rmw/types.h>

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

// Dummy types used only for compile-time layout verification.
struct DummyRequest
{
  int32_t value;
};
struct DummyResponse
{
  int32_t value;
};
struct DummyService
{
  using Request = DummyRequest;
  using Response = DummyResponse;
};

}  // namespace

namespace agnocast
{

struct RequestMeta
{
  alignas(16) int64_t seqno;
  uint8_t client_gid[RMW_GID_STORAGE_SIZE];
  std::string response_topic_name;
};

struct ResponseMeta
{
  alignas(16) int64_t seqno;
};

// These wrappers combine the user payload with Agnocast correlation metadata. Since the payload may
// declare fields that share names with metadata fields, accessing metadata fields always requires
// explicit qualification (e.g. `wrapper->RequestMeta::seqno`) to avoid ambiguity.
template <typename ServiceT>
struct ServiceRequestWrapper : public ServiceT::Request, public RequestMeta
{
};

template <typename ServiceT>
struct ServiceResponseWrapper : public ServiceT::Response, public ResponseMeta
{
};

class GenericRequestWrapper
{
  // sanity check: ensure template and generic versions share the same size.
  static_assert(
    sizeof(ServiceRequestWrapper<DummyService>) ==
      offsetof_meta(sizeof(DummyRequest)) + sizeof(RequestMeta),
    "ServiceRequestWrapper and GenericRequestWrapper are inconsistent (Agnocast internal error)");

  static constexpr size_t get_wire_size(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members)
  {
    return offsetof_meta(request_members->size_of_) + sizeof(RequestMeta);
  }

  static RequestMeta * get_meta_ptr(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members,
    void * request_ptr)
  {
    return reinterpret_cast<RequestMeta *>(
      static_cast<char *>(request_ptr) + offsetof_meta(request_members->size_of_));
  }

  ipc_shared_ptr<void> request_;
  RequestMeta * const meta_ptr_;

public:
  explicit GenericRequestWrapper(
    const rosidl_typesupport_introspection_cpp::MessageMembers * request_members,
    ipc_shared_ptr<void> && request)
  : request_(std::move(request)), meta_ptr_(get_meta_ptr(request_members, request_.get()))
  {
  }

  int64_t & seqno() { return meta_ptr_->seqno; }
  auto client_gid() -> uint8_t (&)[RMW_GID_STORAGE_SIZE] { return meta_ptr_->client_gid; }
  std::string & response_topic_name() { return meta_ptr_->response_topic_name; }

  ipc_shared_ptr<void> && take_request() && { return std::move(request_); }

  /// @brief Allocate a wire-format request buffer in shared memory.
  ///
  /// The payload buffer is initialized via the introspection init function with
  /// MessageInitialization::SKIP. The seqno number and client_gid are uninitialized (garbage),
  /// and the response_topic_name string is default-constructed via placement new.
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

    // Use placement new to construct response_topic_name.
    auto * response_topic_name_ptr = &wrapper.response_topic_name();
    new (response_topic_name_ptr) std::string{};

    return wrapper;
  }

  /// @brief Free a wire-format request buffer in shared memory.
  /// @param ptr Pointer to the request buffer.
  /// @param request_members The introspection message members for the request type.
  static void free(
    void * ptr, const rosidl_typesupport_introspection_cpp::MessageMembers * request_members)
  {
    request_members->fini_function(ptr);

    // Destroy response_topic_name by calling std::destroy_at().
    RequestMeta * meta_ptr = get_meta_ptr(request_members, ptr);
    auto * response_topic_name_ptr = &meta_ptr->response_topic_name;
    std::destroy_at(response_topic_name_ptr);

    ::operator delete(ptr);
  }
};

class GenericResponseWrapper
{
  // sanity check: ensure template and generic versions share the same size.
  static_assert(
    sizeof(ServiceResponseWrapper<DummyService>) ==
      offsetof_meta(sizeof(DummyResponse)) + sizeof(ResponseMeta),
    "ServiceResponseWrapper and GenericResponseWrapper are inconsistent (Agnocast internal error)");

  static constexpr size_t get_wire_size(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members)
  {
    return offsetof_meta(response_members->size_of_) + sizeof(ResponseMeta);
  }

  static ResponseMeta * get_meta_ptr(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members,
    void * response_ptr)
  {
    return reinterpret_cast<ResponseMeta *>(
      static_cast<char *>(response_ptr) + offsetof_meta(response_members->size_of_));
  }

  ipc_shared_ptr<void> response_;
  ResponseMeta * const meta_ptr_;

public:
  explicit GenericResponseWrapper(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members,
    ipc_shared_ptr<void> && response)
  : response_(std::move(response)), meta_ptr_(get_meta_ptr(response_members, response_.get()))
  {
  }

  int64_t & seqno() { return meta_ptr_->seqno; }

  ipc_shared_ptr<void> && take_response() && { return std::move(response_); }

  /// @brief Allocate a wire-format response buffer in shared memory.
  ///
  /// The payload buffer is initialized via the introspection init function with
  /// MessageInitialization::SKIP. The seqno number is uninitialized (garbage).
  ///
  /// @param response_members The introspection message members for the response type.
  /// @param borrow_loaned_message A callable that allocates a buffer of the given size in shared
  /// memory. In practice, this should be TypeErasedPublisher::borrow_loaned_message().
  template <typename Func>
  static GenericResponseWrapper allocate(
    const rosidl_typesupport_introspection_cpp::MessageMembers * response_members,
    Func && borrow_loaned_message)
  {
    ipc_shared_ptr<void> response = borrow_loaned_message(get_wire_size(response_members));
    auto wrapper = GenericResponseWrapper(response_members, std::move(response));

    response_members->init_function(
      wrapper.response_.get(), rosidl_runtime_cpp::MessageInitialization::SKIP);

    return wrapper;
  }

  /// @brief Free a wire-format response buffer in shared memory.
  /// @param ptr Pointer to the response buffer.
  /// @param response_members The introspection message members for the response type.
  static void free(
    void * ptr, const rosidl_typesupport_introspection_cpp::MessageMembers * response_members)
  {
    response_members->fini_function(ptr);
    ::operator delete(ptr);
  }
};

}  // namespace agnocast
