// Backend interface for the GPU-memory-sharing infrastructure the daemon uses to
// create and export slots. The GpuSharedMemoryPool DELEGATES to a GpuSlotBackend
// (per TASK.md: "employ delegation, not inheritance") rather than being subclassed
// per backend. This also lets unit tests inject a mock backend and exercise all
// pool bookkeeping without a GPU.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

// GPU resources owned by the daemon for one slot, plus the exportable handles that
// clients import. Handle blobs are opaque and backend-specific (CUDA IPC, NvSci, ...).
struct AllocatedSlotResources
{
  // Backend-owned opaque objects (e.g. CUDA device pointer / cudaEvent_t), freed by
  // destroy_slot(). Stored as void* so this struct carries no infrastructure type.
  void * device_ptr = nullptr;
  void * data_ready_event = nullptr;
  void * data_done_event = nullptr;

  // Export blobs handed to clients via a ListResponse.
  std::vector<std::uint8_t> mem_handle;
  std::vector<std::uint8_t> data_ready_event_handle;
  std::vector<std::uint8_t> data_done_event_handle;
};

class GpuSlotBackend
{
public:
  virtual ~GpuSlotBackend() = default;

  // Which infrastructure this backend speaks (advertised in the handshake).
  virtual BackendType backend_type() const = 0;

  // Selects the managed GPU and returns its UUID string (e.g. "GPU-...").
  // Called once before any create_slot(). Returns false on failure.
  virtual bool initialize(std::string & gpu_uuid_out) = 0;

  // Allocates one slot's GPU memory and its two inter-process events, and fills
  // `out` with the owned objects and their export blobs. Returns false on failure
  // (leaving no leaked resources for this call).
  virtual bool create_slot(std::size_t size, AllocatedSlotResources & out) = 0;

  // Releases the resources previously produced by create_slot(). Safe to call with
  // a default-constructed / already-released struct.
  virtual void destroy_slot(AllocatedSlotResources & resources) = 0;
};

}  // namespace agnocast::gpu_shared_memory_daemon
