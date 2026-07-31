// Internal header — kept in src/ so it is NOT installed or visible downstream.
// CUDA IPC implementation of GpuClientBackend for discrete NVIDIA GPUs: imports
// the daemon's cudaIpcMemHandle_t / cudaIpcEventHandle_t blobs via
// cudaIpcOpenMemHandle / cudaIpcOpenEventHandle. All CUDA calls go through the
// runtime loader, so this compiles without the CUDA toolkit.
#pragma once

#include "gpu_client_backend.hpp"

#include <string>

namespace agnocast::cuda
{

class CudaIpcClientBackend : public GpuClientBackend
{
public:
  gpu_shared_memory_daemon::BackendType backend_type() const override
  {
    return gpu_shared_memory_daemon::BackendType::kCudaIpc;
  }

  bool local_gpu_uuid(std::string & uuid_out) override;
  bool import_slot(
    const gpu_shared_memory_daemon::SlotDescriptor & descriptor, ImportedSlot & out) override;
  void release_slot(ImportedSlot & imported) override;
  bool record_data_ready(const ImportedSlot & slot, int stream_kind, void * stream) override;
  bool wait_data_ready(const ImportedSlot & slot, int stream_kind, void * stream) override;

  bool create_read_done_marker(void ** out_marker) override;
  void destroy_read_done_marker(void * marker) override;
  bool record_read_done_marker(void * marker, int stream_kind, void * stream) override;
  int query_read_done_marker(void * marker) override;
  void sync_read_done_marker(void * marker) override;
  bool is_default_stream(int stream_kind, void * stream) const override;
};

}  // namespace agnocast::cuda
