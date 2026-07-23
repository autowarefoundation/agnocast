// Declarations of the GPU shared-memory-pool C ABI that agnocastlib calls on the
// CUDA publish/subscribe path. The definitions live in libagnocast_cuda (see
// agnocast_cuda/src/proxy_c_api.*) and are resolved at link time — exactly like
// agnocast::cuda::get_backend(), these symbols are only referenced from inside
// `if constexpr (is_cuda_message_v<T>)` branches, so a non-CUDA executable that
// does not link agnocast_cuda never references them.
//
// This header is CUDA-free: agnocastlib has no build-time CUDA dependency.
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

// Publisher borrow..publish window (thread-local): while active, the CUDA heaphook
// routes this thread's cudaMalloc to a pool slot.
void agnocast_cuda_set_publish_window(int active);

// Publisher: maps a pooled device pointer to its slot id. Returns 1 on success.
int agnocast_cuda_slot_id_from_ptr(void * ptr, std::uint32_t * out_slot_id);

// Publisher: records the slot's data-ready event (per-thread default stream) so
// subscribers can order their reads after the write. Returns 1 on success.
int agnocast_cuda_record_data_ready(void * ptr);

// Publisher reclaim: returns a pooled buffer to the pool, or frees a fallback real
// allocation. Self-contained (agnocastlib cannot call cudaFree itself).
void agnocast_cuda_reclaim_gpu_buffer(void * ptr);

// Subscriber: maps a slot id to this process's local imported device pointer.
// Returns 1 on success.
int agnocast_cuda_ptr_from_slot_id(std::uint32_t slot_id, void ** out_ptr);

// Subscriber: makes subsequent per-thread-default-stream reads wait for the slot's
// data-ready event (call before the callback). Returns 1 on success.
int agnocast_cuda_wait_data_ready(std::uint32_t slot_id);

}  // extern "C"
