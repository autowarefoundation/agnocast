// CUDA IPC implementation of GpuSlotBackend, for discrete NVIDIA GPUs.
//
// Each slot is a cudaMalloc'd device buffer plus two interprocess cudaEvents
// (data-ready / data-done). Their handles are exported via cudaIpcGetMemHandle /
// cudaIpcGetEventHandle so client processes can import them. All CUDA calls go
// through the runtime loader, so this compiles without the CUDA toolkit.
#pragma once

#include "gpu_slot_backend.hpp"

#include <cstddef>
#include <string>

namespace agnocast::gpu_shared_memory_daemon
{

class CudaIpcSlotBackend : public GpuSlotBackend
{
public:
  BackendType backend_type() const override { return BackendType::kCudaIpc; }

  bool initialize(std::string & gpu_uuid_out) override;
  bool create_slot(std::size_t size, AllocatedSlotResources & out) override;
  void destroy_slot(AllocatedSlotResources & resources) override;
};

}  // namespace agnocast::gpu_shared_memory_daemon
