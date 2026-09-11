#pragma once

// The submission API for GPU work on Agnocast messages: dispatch() owns the
// stream, blocks until the work completes, and takes transfers as declarations
// rather than letting the caller write them inside its callable. Why it is
// shaped that way is in docs/gpu_ipc.md.

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
  // scratch can be created where it is used rather than at construction.
  //
  // The caller keeps the pointer and owns it from then on, with two conditions.
  // No size is recorded, so a non-null target is reused as it is: it must be
  // sized for the largest transfer it will ever carry, or a later, longer one
  // runs past the allocation. And the null check and the store are the caller's
  // own variable, unsynchronized, so a target shared by two threads races --
  // give each thread its own.
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

// A declaration holds the address of the handle, which is safe for the whole of
// the dispatch() expression it is written in -- including a temporary, which
// outlives the full expression. Binding one to a temporary and *storing* it is
// not, so it is refused rather than left to dangle.
template <typename T>
Reads<T> reads(agnocast::ipc_shared_ptr<T> &&) = delete;
template <typename T>
Writes<T> writes(agnocast::ipc_shared_ptr<T> &&) = delete;

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

// Logged as well as returned, because a CUDA failure here means the message a
// node is about to publish holds whatever was in the slot before.
inline bool check(cudaError_t status, const char * what)
{
  if (status == cudaSuccess) return true;
  RCLCPP_ERROR(agnocast::logger, "%s failed: %s", what, cudaGetErrorString(status));
  return false;
}

// Owns a thread's stream and event so they are destroyed when the thread exits.
// Without this a node that dispatches from short-lived worker threads leaks one
// of each per thread. Errors are ignored: at thread exit during process teardown
// the driver may already be gone, and there is nothing to report to.
struct ThreadCudaObjects
{
  cudaStream_t stream = nullptr;
  cudaEvent_t event = nullptr;

  ThreadCudaObjects() = default;
  ThreadCudaObjects(const ThreadCudaObjects &) = delete;
  ThreadCudaObjects & operator=(const ThreadCudaObjects &) = delete;

  ~ThreadCudaObjects()
  {
    if (event != nullptr) cudaEventDestroy(event);
    if (stream != nullptr) cudaStreamDestroy(stream);
  }
};

inline ThreadCudaObjects & thread_cuda_objects()
{
  static thread_local ThreadCudaObjects objects;
  return objects;
}

// One stream per thread. A callback never runs two dispatches at once, and the
// callback-isolated executor gives each callback its own thread, so this is one
// stream per callback without the library tracking callbacks itself.
inline cudaStream_t stream()
{
  cudaStream_t & s = thread_cuda_objects().stream;
  if (
    s == nullptr && !check(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking), "stream create")) {
    s = nullptr;
  }
  return s;
}

// Blocking rather than spinning: spinning would cost a core per concurrent
// dispatch, and nothing is queued for the GPU window it would free.
inline cudaEvent_t completion_event()
{
  cudaEvent_t & e = thread_cuda_objects().event;
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

// Pageable host memory makes cudaMemcpyAsync behave synchronously, putting an
// unexpected host block inside the GPU window.
inline bool host_buffer_is_pinned(const void * host_ptr)
{
  // The caller's pending error, if any, is preserved: this probe runs before the
  // work and must not consume state the caller is entitled to read.
  const cudaError_t pending = cudaGetLastError();
  cudaPointerAttributes attributes = {};
  const cudaError_t probed = cudaPointerGetAttributes(&attributes, host_ptr);
  if (probed != cudaSuccess) static_cast<void>(cudaGetLastError());
  if (pending != cudaSuccess) {
    RCLCPP_WARN_ONCE(
      agnocast::logger, "a CUDA error was already pending on entry to dispatch(): %s",
      cudaGetErrorString(pending));
  }
  return probed == cudaSuccess && attributes.type == cudaMemoryTypeHost;
}

// Maps the region the message refers to, if this process has not mapped it yet:
// the first frame from a given publisher establishes the mapping and every later
// one resolves against it. For the publisher's own region this is a lookup.
//
// This is the one part of a dispatch that allocates host memory in the driver,
// so it lands in the shared-memory mempool when a node borrows before
// dispatching; see docs/gpu_ipc.md for why it is left here.
template <typename T>
bool ensure_message_mapped(const agnocast::ipc_shared_ptr<T> & message)
{
  if (!message) return true;

  // The common case -- every frame after the first from a given publisher -- is
  // settled without naming the topic at all. get_topic_name() returns a string
  // by value, and inside the borrow window that copy comes from the
  // shared-memory mempool, so doing it per frame would put an allocation on the
  // message path for the sake of a lookup that is about to be skipped.
  const uint32_t region_id = message->data.region_id();
  if (region_id != 0 && agnocast::internal::GpuRegionRegistry::instance().is_mapped(region_id)) {
    return message->data.get() != nullptr;
  }

  const std::string topic_name = message.get_topic_name();
  // The subscriber id is this handle's own endpoint: the kmod checks it against
  // the calling process before handing out a descriptor.
  const agnocast::internal::GpuRegionRef ref{
    topic_name, message->data.publisher_id(), message.get_pubsub_id(), region_id};
  if (!agnocast::internal::GpuRegionRegistry::instance().ensure_mapped(ref)) {
    RCLCPP_ERROR(
      agnocast::logger, "could not map the GPU region of topic '%s'", topic_name.c_str());
    return false;
  }
  return message->data.get() != nullptr;
}

// Returns whether the work may run. A message whose region could not be mapped
// resolves to a null device pointer, so launching anyway would fault the device
// or, worse, publish untouched memory.
template <typename T>
bool prepare(const Reads<T> & declaration, cudaStream_t)
{
  return ensure_message_mapped(*declaration.message);
}

template <typename T>
bool prepare(const Writes<T> & declaration, cudaStream_t)
{
  const auto & message = *declaration.message;
  if (!message || message->data.get() != nullptr) return true;

  // Otherwise the work would run against a null device pointer, or be skipped
  // with no trace: the two ways to get here -- a message borrowed without the
  // capacity overload, and one whose region is gone -- are both silent
  // everywhere else.
  RCLCPP_ERROR(
    agnocast::logger,
    "the GPU payload of a message declared with writes() does not resolve: it was not borrowed "
    "with the capacity overload, or its region is no longer mapped");
  return false;
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

  if (!ok) {
    // Running the work anyway would compute over a buffer an upload failed to
    // fill, and the result would then be published: a caller is invited to
    // ignore this function's result, so the frame has to be abandoned here
    // rather than completed with whatever the buffer held.
    RCLCPP_ERROR(agnocast::logger, "not submitting GPU work: a declared transfer failed");
    static_cast<void>(cudaStreamSynchronize(s));
    gate_release();
    return false;
  }

  // Whatever error the caller left pending is theirs; taking it as ours would
  // fail their next good dispatch, and cudaGetLastError() below would report it
  // as a submission failure.
  static_cast<void>(cudaGetLastError());

  // The completion guarantee has to survive an exception from the caller's work:
  // returning with a kernel still running would let the message's slot be
  // returned to its pool and handed to another message while the device is still
  // writing it. Synchronize, release the gate, then let the exception continue.
  try {
    std::get<kWork>(parts)(s);
  } catch (...) {
    static_cast<void>(cudaStreamSynchronize(s));
    gate_release();
    throw;
  }
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
