#pragma once

// The submission API for GPU work on Agnocast messages.
//
// Agnocast owns the stream and passes it to the caller's lambda, so a message
// and the work touching it cannot be associated with different streams. Designs
// that take the stream through a publisher or a message allow writing on one
// stream and publishing against another, sending out unfinished data.
//
// dispatch() returns only after the submitted work has completed. That is what
// keeps the host-side invariant that a message is finished when its reference
// count reaches zero: on the GPU a read is asynchronous, so a callback that
// merely launched a kernel would return with the device still reading. Waiting
// costs this node its own GPU time, a quantity its schedulability analysis
// already accounts for, and adds no coupling to other nodes.
//
// Transfers are declared rather than written inside the lambda so the library
// owns them: it can check that host buffers are page-locked, and it can time
// transfers separately from kernels. Only the "upload first, download last"
// shape is expressible; interleaved copies belong in the lambda, where they
// count as GPU work.

#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <cuda_runtime.h>

#include <cstddef>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace agnocast::gpu
{

// ---------------------------------------------------------------------------
// Declarations
// ---------------------------------------------------------------------------
//
// Named reads/writes rather than in/out because a local variable of either name
// would shadow the function at unqualified lookup and break the call. Transfers
// are typed so an element count cannot be mistaken for a byte count.

template <typename T>
struct Reads
{
  const agnocast::ipc_shared_ptr<T> * message;
};

template <typename T>
struct Writes
{
  const agnocast::ipc_shared_ptr<T> * message;
};

// Per-transfer options, combined with `|`.
enum class TransferOptions : uint32_t {
  kNone = 0,
  // Allocate the device-side buffer on the dispatch stream when it is still
  // null, before the work runs. cudaMalloc is synchronous and would stall the
  // host inside a callback; a stream-ordered allocation does not, so device
  // scratch can be created where it is used rather than at construction. The
  // caller keeps the pointer and owns it from then on.
  kAllocateDeviceAsync = 1U << 0,
};

constexpr TransferOptions operator|(TransferOptions a, TransferOptions b)
{
  return static_cast<TransferOptions>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

constexpr bool has_option(TransferOptions set, TransferOptions flag)
{
  return (static_cast<uint32_t>(set) & static_cast<uint32_t>(flag)) != 0U;
}

struct Upload
{
  // Held by address so an allocation made here is visible to the caller.
  void ** device_dst;
  const void * host_src;
  size_t bytes;
  TransferOptions options;
};

struct Download
{
  void * host_dst;
  void ** device_src;
  size_t bytes;
  TransferOptions options;
};

template <typename T>
Reads<T> reads(const agnocast::ipc_shared_ptr<T> & message)
{
  return Reads<T>{&message};
}

template <typename T>
Writes<T> writes(const agnocast::ipc_shared_ptr<T> & message)
{
  return Writes<T>{&message};
}

template <typename T>
Upload uploads(
  T *& device_dst, const T * host_src, size_t count,
  TransferOptions options = TransferOptions::kNone)
{
  return Upload{reinterpret_cast<void **>(&device_dst), host_src, count * sizeof(T), options};
}

template <typename T>
Download downloads(
  T * host_dst, T *& device_src, size_t count, TransferOptions options = TransferOptions::kNone)
{
  return Download{host_dst, reinterpret_cast<void **>(&device_src), count * sizeof(T), options};
}

namespace detail
{

template <typename T>
struct is_declaration : std::false_type
{
};
template <typename T>
struct is_declaration<Reads<T>> : std::true_type
{
};
template <typename T>
struct is_declaration<Writes<T>> : std::true_type
{
};
template <>
struct is_declaration<Upload> : std::true_type
{
};
template <>
struct is_declaration<Download> : std::true_type
{
};

// Logged rather than returned only, because a CUDA failure on this path means
// the message a node is about to publish holds whatever was there before.
inline bool check(cudaError_t status, const char * what)
{
  if (status == cudaSuccess) return true;
  RCLCPP_ERROR(agnocast::logger, "%s failed: %s", what, cudaGetErrorString(status));
  return false;
}

// One stream per thread. A callback never runs two dispatches at once, and the
// callback-isolated executor gives each callback its own thread, so this is one
// stream per callback without the library tracking callbacks itself.
inline cudaStream_t stream()
{
  static thread_local cudaStream_t s = nullptr;
  if (
    s == nullptr && !check(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking), "stream create")) {
    s = nullptr;
  }
  return s;
}

// Blocking rather than spinning: until admission control bounds how many threads
// may wait at once, spinning costs one core per concurrent dispatch and frees a
// GPU window no one is queued for.
inline cudaEvent_t completion_event()
{
  static thread_local cudaEvent_t e = nullptr;
  if (
    e == nullptr && !check(
                      cudaEventCreateWithFlags(&e, cudaEventDisableTiming | cudaEventBlockingSync),
                      "event create")) {
    e = nullptr;
  }
  return e;
}

// Admission control. Empty until the dispatch scheduler exists; the call sites
// are here so that adding it changes neither this signature nor user code.
inline void gate()
{
}
inline void gate_release()
{
}

// Pageable host memory makes cudaMemcpyAsync behave synchronously, which would
// put an unexpected host block inside the GPU window and corrupt any measurement
// of it. A convention can be violated silently; this cannot.
inline bool host_buffer_is_pinned(const void * host_ptr)
{
  cudaPointerAttributes attributes = {};
  if (cudaPointerGetAttributes(&attributes, host_ptr) != cudaSuccess) {
    cudaGetLastError();  // clear, so a later unrelated call is not blamed
    return false;
  }
  return attributes.type == cudaMemoryTypeHost;
}

// A declared message must be mapped before the work can address it. Doing it
// here is what makes reads()/writes() load-bearing rather than documentation:
// the first frame from an unseen publisher establishes the mapping, and every
// later one resolves against it. The publisher's own region is already mapped,
// so this is a lookup for it.
// Returns whether the work may run. A message whose region could not be mapped
// resolves to a null device pointer, so launching anyway would fault the device
// or, worse, publish untouched memory.
template <typename T>
bool prepare(const Reads<T> & declaration, cudaStream_t)
{
  const auto & message = *declaration.message;
  if (!message) return true;

  const std::string topic_name = message.get_topic_name();
  const agnocast::internal::GpuRegionRef ref{
    topic_name, message->data.publisher_id(), 0, message->data.region_id()};
  if (!agnocast::internal::GpuRegionRegistry::instance().ensure_mapped(ref)) {
    RCLCPP_ERROR(
      agnocast::logger, "could not map the GPU region of topic '%s'", topic_name.c_str());
    return false;
  }
  return message->data.get() != nullptr;
}

template <typename T>
bool prepare(const Writes<T> & declaration, cudaStream_t)
{
  const auto & message = *declaration.message;
  return !message || message->data.get() != nullptr;
}

// Device buffers are created before the work and on the same stream, so the work
// can address them without the host having synchronized first.
inline bool allocate_if_requested(
  void ** device_ptr, size_t bytes, TransferOptions options, cudaStream_t s)
{
  if (!has_option(options, TransferOptions::kAllocateDeviceAsync)) return true;
  if (*device_ptr != nullptr) return true;
  return check(cudaMallocAsync(device_ptr, bytes, s), "cudaMallocAsync");
}

inline bool prepare(const Upload & u, cudaStream_t s)
{
  return allocate_if_requested(u.device_dst, u.bytes, u.options, s);
}

inline bool prepare(const Download & d, cudaStream_t s)
{
  return allocate_if_requested(d.device_src, d.bytes, d.options, s);
}

template <typename T>
bool issue_upload(const T &, cudaStream_t)
{
  return true;
}
inline bool issue_upload(const Upload & u, cudaStream_t s)
{
  if (!host_buffer_is_pinned(u.host_src)) {
    RCLCPP_WARN_ONCE(
      agnocast::logger,
      "uploads() was given pageable host memory; the copy will block the host and its cost will "
      "be counted as GPU time. Allocate it with cudaMallocHost.");
  }
  return check(
    cudaMemcpyAsync(*u.device_dst, u.host_src, u.bytes, cudaMemcpyHostToDevice, s), "upload");
}

template <typename T>
bool issue_download(const T &, cudaStream_t)
{
  return true;
}
inline bool issue_download(const Download & d, cudaStream_t s)
{
  if (!host_buffer_is_pinned(d.host_dst)) {
    RCLCPP_WARN_ONCE(
      agnocast::logger,
      "downloads() was given pageable host memory; the copy will block the host. Allocate it with "
      "cudaMallocHost.");
  }
  return check(
    cudaMemcpyAsync(d.host_dst, *d.device_src, d.bytes, cudaMemcpyDeviceToHost, s), "download");
}

template <typename Tuple, size_t... I>
bool run(Tuple && parts, std::index_sequence<I...>)
{
  constexpr size_t kWork = sizeof...(I);
  static_assert(
    (is_declaration<std::decay_t<std::tuple_element_t<I, std::decay_t<Tuple>>>>::value && ...),
    "every argument before the last must be reads(), writes(), uploads() or downloads()");

  cudaStream_t s = stream();
  cudaEvent_t done = completion_event();
  if (s == nullptr || done == nullptr) return false;

  // Folded so that every declaration is prepared even once one has failed:
  // allocations made here are the caller's from then on.
  bool ready = true;
  ((ready = prepare(std::get<I>(parts), s) && ready), ...);
  if (!ready) return false;

  gate();
  bool ok = true;
  ((ok = issue_upload(std::get<I>(parts), s) && ok), ...);

  std::get<kWork>(parts)(s);
  ok = check(cudaGetLastError(), "work submission") && ok;

  ok = check(cudaEventRecord(done, s), "event record") && ok;
  ok = check(cudaEventSynchronize(done), "event synchronize") && ok;
  gate_release();

  // Downloads follow the release so the GPU is available to the next admitted
  // node while this one copies its results back.
  ((ok = issue_download(std::get<I>(parts), s) && ok), ...);
  ok = check(cudaStreamSynchronize(s), "stream synchronize") && ok;
  return ok;
}

}  // namespace detail

/**
 * @brief Submit GPU work over Agnocast messages, returning once it has completed.
 * @return Whether everything succeeded. Failures are logged either way, so a
 * caller that has nothing better to do than carry on may ignore this.
 *
 * Takes any number of declarations followed by a callable of `void(cudaStream_t)`.
 * They arrive in one parameter pack with the callable taken from the tail,
 * because a parameter pack in non-final position is not deduced.
 */
template <typename... Args>
bool dispatch(Args &&... args)
{
  static_assert(sizeof...(Args) >= 1, "dispatch() needs at least the work lambda");
  return detail::run(
    std::forward_as_tuple(std::forward<Args>(args)...),
    std::make_index_sequence<sizeof...(Args) - 1>{});
}

}  // namespace agnocast::gpu
