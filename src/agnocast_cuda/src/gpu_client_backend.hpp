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

  // --- Ready edge: publisher write -> subscriber read ----------------------------
  //
  // `stream_kind` / `stream` are the C ABI's opaque stream declaration; see
  // cudart_loader.hpp::resolve_stream(). A CUDA-IPC backend maps them to a
  // cudaStream_t; an NvSci backend will map them to its own queue.

  // Publisher side: records the slot's data-ready event (GPU write complete) on the
  // publisher's stream so subscribers can order their reads after it. Returns false
  // on failure.
  virtual bool record_data_ready(const ImportedSlot & slot, int stream_kind, void * stream) = 0;

  // Subscriber side: makes subsequent GPU work on the subscriber's stream wait for
  // the slot's data-ready event. Returns false on failure.
  virtual bool wait_data_ready(const ImportedSlot & slot, int stream_kind, void * stream) = 0;

  // --- Done edge: subscriber read -> slot reuse by the next publisher -------------
  //
  // A "read-done marker" is a process-PRIVATE (non-interprocess) synchronization
  // object recorded on the reader's stream. The proxy polls it to decide when a
  // message's kernel-side reference may be released. Markers are pooled and reused,
  // so their count scales with in-flight messages per reader process — not with pool
  // slots x subscribers, which is what rules out per-subscriber interprocess events
  // (MAX_SUBSCRIBER_NUM is 3072 per topic).

  // Creates one marker. Returns false on failure.
  virtual bool create_read_done_marker(void ** out_marker) = 0;

  // Destroys a marker produced by create_read_done_marker().
  virtual void destroy_read_done_marker(void * marker) = 0;

  // Records the marker on the reader's stream. Returns false on failure.
  virtual bool record_read_done_marker(void * marker, int stream_kind, void * stream) = 0;

  // Polls a recorded marker: 1 = complete, 0 = still pending, -1 = error. Must never
  // block, so a stuck reader can never wedge the release path.
  virtual int query_read_done_marker(void * marker) = 0;

  // Blocks until a recorded marker completes. Used only under deferral pressure and
  // at shutdown.
  virtual void sync_read_done_marker(void * marker) = 0;

  // True when (stream_kind, stream) resolves to a *default* stream. Default streams do
  // not order against streams created non-blocking, which makes that combination a
  // correctness failure worth failing fast on.
  virtual bool is_default_stream(int stream_kind, void * stream) const = 0;
};

}  // namespace agnocast::cuda
