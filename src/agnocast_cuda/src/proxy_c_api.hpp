// C ABI over the GpuSharedMemoryPoolProxy singleton.
//
// This is the boundary the CUDA heaphook (a separate LD_PRELOAD library, written
// in Rust) calls through, and the hook that agnocast's publish path uses to mark
// the window in which a publisher's cudaMalloc should be served from the pool.
// Keeping it extern "C" gives a stable, name-mangling-free surface for FFI.
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

// Publish window (thread-local): set to non-zero by the publish path around the
// borrow_loaned_message() .. publish() region of a CUDA publisher, so the hook
// knows a cudaMalloc on this thread should be routed to the pool.
void agnocast_cuda_set_publish_window(int active);
int agnocast_cuda_in_publish_window(void);

// Reserves a pooled slot of at least `size` bytes via the proxy singleton and
// writes its device pointer to *out_ptr. Returns 1 on success, 0 otherwise (no
// daemon, pool exhausted after retries, or size too large) — the hook then falls
// back to the real cudaMalloc.
int agnocast_cuda_pool_allocate(size_t size, void ** out_ptr);

// Frees a pointer previously returned by agnocast_cuda_pool_allocate(). Returns 1
// if `ptr` was a pooled pointer (and was freed), 0 if it is not owned by the pool
// (the hook then calls the real cudaFree).
int agnocast_cuda_pool_free(void * ptr);

// --- Publish/subscribe hooks used by agnocastlib (guarded by is_cuda_message_v) ---

// Publisher: maps a pooled device pointer to its slot id (stored in the message's
// GpuMetadata). Returns 1 on success.
int agnocast_cuda_slot_id_from_ptr(void * ptr, std::uint32_t * out_slot_id);

// Subscriber: maps a slot id (from GpuMetadata) to this process's local imported
// device pointer for that slot. Returns 1 on success.
int agnocast_cuda_ptr_from_slot_id(std::uint32_t slot_id, void ** out_ptr);

// Stream declarations cross this ABI as a (kind, handle) pair, where `stream_kind` is
// an agnocast::CudaStreamKind value. A null handle is never used to mean "the default
// stream": this library resolves the unsuffixed cudaEventRecord through its own
// dlopen'd libcudart, where NULL always means the LEGACY stream — so a caller compiled
// with `--default-stream per-thread` passing NULL would silently get the wrong stream.

// Publisher: records the data-ready event for the slot backing `ptr` on the
// publisher's stream (call at publish). Returns 1 on success.
int agnocast_cuda_record_data_ready(void * ptr, int stream_kind, void * stream);

// Subscriber: makes subsequent GPU work on the subscriber's stream wait for the slot's
// data-ready event (call before invoking the callback). Returns 1 on success.
int agnocast_cuda_wait_data_ready(std::uint32_t slot_id, int stream_kind, void * stream);

// --- Done edge: reader-local deferred release ------------------------------------

// Records a read-done marker on the subscriber's stream, writing an opaque token to
// *out_token. Returns 1 on success.
int agnocast_cuda_record_read_done(int stream_kind, void * stream, void ** out_token);

// Polls a token: 1 = complete (token consumed), 0 = pending. Never blocks.
int agnocast_cuda_query_read_done(void * token);

// Blocks until the token's marker completes, then consumes the token.
void agnocast_cuda_wait_read_done(void * token);

// --- Fail-fast on undeclared non-blocking streams --------------------------------

// Called by the CUDA heaphook when this process creates a stream with the
// cudaStreamNonBlocking flag (runtime API) or CU_STREAM_NON_BLOCKING (driver API).
void agnocast_cuda_note_non_blocking_stream(void);

// Returns 1 when using (stream_kind, stream) for Agnocast's ordering would be unsafe:
// the process created a non-blocking stream and the pair resolves to a default stream,
// which does not order against it. Returns 0 otherwise.
int agnocast_cuda_stream_ordering_unsafe(int stream_kind, void * stream);

// Publisher reclaim: releases a GPU buffer allocated during a borrow..publish
// window. If `ptr` is a pooled pointer it is returned to the pool; otherwise (a
// fallback real cudaMalloc) the real device memory is freed. Self-contained so
// agnocastlib, which cannot call cudaFree, can reclaim in either case.
void agnocast_cuda_reclaim_gpu_buffer(void * ptr);

}  // extern "C"
