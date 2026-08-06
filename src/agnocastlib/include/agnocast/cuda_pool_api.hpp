// Declarations of the GPU shared-memory-pool C ABI that agnocastlib calls on the
// CUDA publish/subscribe path. The definitions live in libagnocast_cuda (see
// agnocast_cuda/src/proxy_c_api.*) and are resolved at link time — exactly like
// agnocast::cuda::get_backend(), these symbols are only referenced from inside
// `if constexpr (is_cuda_message_v<T>)` branches, so a non-CUDA executable that
// does not link agnocast_cuda never references them.
//
// This header is CUDA-free: agnocastlib has no build-time CUDA dependency.
//
// Stream arguments come in pairs: `stream_kind` is an agnocast::CudaStreamKind value
// and `stream` is the opaque handle, meaningful only for the `kExplicit` kind. A null
// handle is NEVER passed down as "the default stream" — see cuda_stream.hpp for why
// that would silently mean the legacy stream inside libagnocast_cuda.
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

// Publisher borrow..publish window (thread-local): while active, the CUDA heaphook
// routes this thread's cudaMalloc to a pool slot.
void agnocast_cuda_set_publish_window(int active);

// Publisher: maps a pooled device pointer to its slot id. Returns 1 on success.
int agnocast_cuda_slot_id_from_ptr(void * ptr, std::uint32_t * out_slot_id);

// Publisher: records the slot's data-ready event on the given stream so subscribers
// can order their reads after the write. Returns 1 on success.
int agnocast_cuda_record_data_ready(void * ptr, int stream_kind, void * stream);

// Publisher reclaim: returns a pooled buffer to the pool, or frees a fallback real
// allocation. Self-contained (agnocastlib cannot call cudaFree itself).
void agnocast_cuda_reclaim_gpu_buffer(void * ptr);

// Subscriber: maps a slot id to this process's local imported device pointer.
// Returns 1 on success.
int agnocast_cuda_ptr_from_slot_id(std::uint32_t slot_id, void ** out_ptr);

// Subscriber: makes subsequent work on the given stream wait for the slot's
// data-ready event (call before the callback). Returns 1 on success.
int agnocast_cuda_wait_data_ready(std::uint32_t slot_id, int stream_kind, void * stream);

// --- Done edge: reader-local deferred release ------------------------------------
//
// The subscriber records a process-private (non-interprocess) marker on its declared
// stream when it drops its last message reference; Agnocast holds the kernel-side
// reference until the marker completes, so the publisher cannot hand the slot to the
// next writer while the reader's GPU reads are still in flight.

// Records a read-done marker on the given stream and writes an opaque token to
// *out_token. Returns 1 on success; on failure the caller must release immediately.
int agnocast_cuda_record_read_done(int stream_kind, void * stream, void ** out_token);

// Polls a token. Returns 1 when the marker has completed (the token is consumed and
// must not be used again) and 0 while it is still pending. Never blocks.
int agnocast_cuda_query_read_done(void * token);

// Blocks until the token's marker completes, then consumes the token. Used only when
// the deferred-release ring is full and at shutdown.
void agnocast_cuda_wait_read_done(void * token);

// --- Fail-fast on undeclared non-blocking streams --------------------------------

// Called by the CUDA heaphook when the process creates a stream with the
// non-blocking flag. A default (legacy / per-thread) stream does NOT order against
// such a stream, so combining the two silently breaks the ready edge.
void agnocast_cuda_note_non_blocking_stream(void);

// Returns 1 when using (stream_kind, stream) for Agnocast's ordering would be
// unsafe: this process created a non-blocking stream and the pair resolves to a
// default stream. The caller aborts with a message naming the fix.
int agnocast_cuda_stream_ordering_unsafe(int stream_kind, void * stream);

}  // extern "C"
