// Internal header — kept in src/ so it is NOT installed or visible downstream.
//
// Client-side counterpart of the daemon's GpuSlotBackend. The
// GpuSharedMemoryPoolProxy DELEGATES to a GpuClientBackend (per TASK.md:
// "delegation, not inheritance") to import the daemon-exported slot handles into
// this process and to query the local GPU. Concrete backends (CudaIpc, later
// NvSci) hide the infrastructure; tests inject a mock so all proxy bookkeeping
// runs without a GPU.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <string>

namespace agnocast::cuda
{

// Local (process-private) objects obtained by importing one slot's exported
// handles. Pointers/events are unique to this importer.
struct ImportedSlot
{
  void * device_ptr = nullptr;        // local GPU device pointer
  void * data_ready_event = nullptr;  // local imported event (opaque cudaEvent_t)
  void * data_done_event = nullptr;
};

class GpuClientBackend
{
public:
  virtual ~GpuClientBackend() = default;

  // Infrastructure this backend speaks; checked against the daemon's handshake.
  virtual gpu_shared_memory_daemon::BackendType backend_type() const = 0;

  // Returns the UUID of the GPU this process is currently using. Used to derive
  // the daemon socket path and to verify (against the handshake) that the daemon
  // manages the same GPU. Returns false on failure.
  virtual bool local_gpu_uuid(std::string & uuid_out) = 0;

  // Imports one slot's exported handles into this process. Returns false on
  // failure (leaving nothing to release for this call).
  virtual bool import_slot(
    const gpu_shared_memory_daemon::SlotDescriptor & descriptor, ImportedSlot & out) = 0;

  // Releases resources previously produced by import_slot(). Safe on a
  // default-constructed / already-released struct.
  virtual void release_slot(ImportedSlot & imported) = 0;

  // Publisher side: records the slot's data-ready event (GPU write complete) so
  // subscribers can order their reads after it. Returns false on failure.
  virtual bool record_data_ready(const ImportedSlot & slot) = 0;

  // Subscriber side: makes subsequent GPU reads wait for the slot's data-ready
  // event. Returns false on failure.
  virtual bool wait_data_ready(const ImportedSlot & slot) = 0;
};

}  // namespace agnocast::cuda
