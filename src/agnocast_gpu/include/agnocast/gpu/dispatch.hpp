#pragma once

// The submission API for GPU work on Agnocast messages: dispatch() owns the
// stream, blocks until the work completes, and takes transfers as declarations
// rather than letting the caller write them inside its callable. Why it is
// shaped that way is in docs/gpu_ipc.md.

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"
#include "rcpputils/scope_exit.hpp"

#include <cuda_runtime.h>

#include <cstddef>
#include <optional>
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
//
// Never throws, and that is load-bearing rather than tidiness: most calls sit
// between submitting work and waiting for it, and RCLCPP_ERROR allocates --
// inside the borrow window from the shared-memory mempool, which returns null
// when exhausted. Unwinding from here would leave a kernel running over a slot
// whose message is about to be destroyed, handing that slot to the next borrow
// while the device is still writing it. Losing the log line is the lesser loss.
inline bool check(cudaError_t status, const char * what) noexcept
{
  if (status == cudaSuccess) return true;
  try {
    RCLCPP_ERROR(agnocast::logger, "%s failed: %s", what, cudaGetErrorString(status));
  } catch (...) {  // NOLINT(bugprone-empty-catch)
  }
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
// which is why run() keeps the borrow window closed across it; see
// docs/gpu_ipc.md.
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

  // Everything this function does outside the caller's callable is the library's
  // own work or the driver's, and none of it belongs to the message, so it is
  // kept out of the shared-memory mempool: creating this thread's stream and
  // event, mapping a region on first receipt, the stream-ordered allocator's
  // pool. Without this the first submission on a thread would leave the driver's
  // one-time bookkeeping in the segment for the life of the process -- and on an
  // executor that owns its callback threads there is no earlier moment at which
  // to do it instead.
  //
  // An optional rather than a unique_ptr, because the window has to be reopened
  // around the caller's callable and a unique_ptr allocates to do it: `operator
  // new` runs before the constructor that closes the window, so the allocation
  // lands in the very mempool this exists to keep out of, and throws there when
  // it is exhausted. emplace() constructs in place and does neither.
  std::optional<agnocast::internal::SuspendedBorrowWindow> suspended;
  suspended.emplace();

  cudaStream_t s = stream();
  cudaEvent_t done = completion_event();
  if (s == nullptr || done == nullptr) return false;

  // Armed before anything reaches the stream, because preparing a declaration
  // already queues work: a stream-ordered allocation for an upload, and a region
  // mapping that can throw from the string and the log line it takes. From here
  // the device may be touching the message's slot, so every path out of this
  // function has to wait for it first -- an exception as much as a return.
  // Unwinding with work still in flight would let the slot be returned to its
  // pool and handed to the next borrow while the device is still writing it. The
  // caller's callable is the obvious way that happens, but not the only one:
  // every RCLCPP_* call below allocates, and inside the borrow window that is
  // the mempool, which throws when exhausted.
  //
  // The guard closes the window itself rather than leaning on `suspended`:
  // guards are destroyed before objects declared above them, so when the
  // caller's callable throws this runs with the window open again, and the
  // driver call it makes allocates host memory of its own.
  auto drain = rcpputils::make_scope_exit([s]() noexcept {
    const agnocast::internal::SuspendedBorrowWindow drain_suspended;
    static_cast<void>(cudaStreamSynchronize(s));
  });

  // Folded so that every declaration is prepared even once one has failed:
  // allocations made here are the caller's from then on.
  bool ready = true;
  ((ready = prepare(std::get<I>(parts), s) && ready), ...);
  if (!ready) return false;

  bool ok = true;
  ((ok = issue_upload(std::get<I>(parts), s) && ok), ...);

  if (!ok) {
    // Running the work anyway would compute over a buffer an upload failed to
    // fill, and the result would then be published: a caller is invited to
    // ignore this function's result, so the frame has to be abandoned here
    // rather than completed with whatever the buffer held.
    RCLCPP_ERROR(agnocast::logger, "not submitting GPU work: a declared transfer failed");
    return false;
  }

  // Whatever error the caller left pending is theirs; taking it as ours would
  // fail their next good dispatch, and cudaGetLastError() below would report it
  // as a submission failure.
  static_cast<void>(cudaGetLastError());

  // The callable is the user's, and may well fill in message fields, so it runs
  // with the window as it found it.
  suspended.reset();
  std::get<kWork>(parts)(s);
  ok = check(cudaGetLastError(), "work submission") && ok;

  // Reopened for the driver calls below, which allocate host memory of their own.
  suspended.emplace();
  ok = check(cudaEventRecord(done, s), "event record") && ok;
  ok = check(cudaEventSynchronize(done), "event synchronize") && ok;

  // Downloads follow completion, so they read results rather than a buffer the
  // work is still writing.
  ((ok = issue_download(std::get<I>(parts), s) && ok), ...);
  ok = check(cudaStreamSynchronize(s), "stream synchronize") && ok;
  drain.cancel();
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
