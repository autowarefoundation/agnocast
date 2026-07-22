// C ABI over the GpuSharedMemoryPoolProxy singleton.
//
// This is the boundary the CUDA heaphook (a separate LD_PRELOAD library, written
// in Rust) calls through, and the hook that agnocast's publish path uses to mark
// the window in which a publisher's cudaMalloc should be served from the pool.
// Keeping it extern "C" gives a stable, name-mangling-free surface for FFI.
#pragma once

#include <cstddef>

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

}  // extern "C"
