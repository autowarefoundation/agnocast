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

// Publisher: records the data-ready event for the slot backing `ptr` (call at
// publish, on the per-thread default stream). Returns 1 on success.
int agnocast_cuda_record_data_ready(void * ptr);

// Subscriber: makes subsequent per-thread-default-stream GPU reads wait for the
// slot's data-ready event (call before invoking the callback). Returns 1 on success.
int agnocast_cuda_wait_data_ready(std::uint32_t slot_id);

// Publisher reclaim: releases a GPU buffer allocated during a borrow..publish
// window. If `ptr` is a pooled pointer it is returned to the pool; otherwise (a
// fallback real cudaMalloc) the real device memory is freed. Self-contained so
// agnocastlib, which cannot call cudaFree, can reclaim in either case.
void agnocast_cuda_reclaim_gpu_buffer(void * ptr);

}  // extern "C"
