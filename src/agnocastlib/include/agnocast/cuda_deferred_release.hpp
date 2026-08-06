// Reader-local deferred release — the GPU-IPC "done edge".
//
// Once subscribers are free to read on their own CUDA streams, dropping the last
// message reference at the end of a callback is no longer safe: the kernel-side
// reference count reaching zero lets the publisher return the pool slot, and the next
// publisher's kernel then overwrites bytes a reader is still consuming (silent
// write-after-read corruption).
//
// So the subscriber records a process-private marker on its declared stream when it
// drops its last reference, and Agnocast holds the kernel-side reference until that
// marker completes. Completion is *polled* (never waited on), so no GPU channel is
// held, no deadlock is possible, and a stuck reader degrades to pool pressure with a
// diagnosable warning rather than a hang.
//
// This header is CUDA-free. The three primitives are installed as function pointers
// by the CUDA message path (which lives in `if constexpr (is_cuda_message_v<T>)`
// branches), so agnocastlib itself never references the CUDA C ABI and a non-CUDA
// executable does not need to link libagnocast_cuda.
#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_public_api.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace agnocast
{

// Maximum number of concurrently deferred releases. Scales with in-flight messages
// per reader process, not with pool slots x subscribers. Reaching the cap makes the
// next deferral block on the oldest marker rather than release early, because
// releasing early is a correctness failure while blocking is only a slowdown.
constexpr size_t MAX_DEFERRED_SUBSCRIBER_RELEASES = 32;

// Hooks into libagnocast_cuda's read-done markers (see cuda_pool_api.hpp).
struct GpuDoneOps
{
  int (*record)(int stream_kind, void * stream, void ** out_token) = nullptr;
  int (*query)(void * token) = nullptr;
  void (*wait)(void * token) = nullptr;
};

// Installs the hooks. Idempotent: the first complete set of hooks wins, later calls
// with the same or another complete set are ignored.
void set_gpu_done_ops(const GpuDoneOps & ops);

// Records a read-done marker on the subscriber's declared stream and defers
// release_subscriber_reference() until it completes. Returns false when no marker
// could be recorded (no hooks installed, or the backend failed), in which case the
// caller must release the reference immediately.
bool defer_subscriber_release(
  const std::string & topic_name, topic_local_id_t pubsub_id, int64_t entry_id, int stream_kind,
  void * stream);

// Releases every deferred reference whose marker has completed. Never blocks, and is
// a cheap no-op when nothing is pending, so executors can call it unconditionally
// after each callback.
void drain_deferred_subscriber_releases();

// Releases every deferred reference immediately, WITHOUT polling its marker, and
// uninstalls the hooks. Called from Agnocast's static-destruction cleanup, where
// libagnocast_cuda's proxy singleton has already been destroyed — see the implementation
// for why polling there would be undefined behaviour, and why releasing without waiting
// is safe at that point. After this call, deferral is off: a later release goes straight
// through, which is what teardown wants.
void flush_deferred_subscriber_releases();

// Number of releases still deferred. For tests and diagnostics.
size_t deferred_subscriber_release_count();

}  // namespace agnocast
