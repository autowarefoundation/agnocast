// Internal header — kept in src/ so it is NOT installed or visible downstream.
//
// Per-process client of the GpuSharedMemoryPoolDaemon. It connects over the Unix
// domain socket, verifies the daemon's backend/GPU via the handshake, and imports
// every slot's handles once at initialization so later allocate/free are cheap.
//
// This is a concrete class that DELEGATES all infrastructure-specific work
// (importing handles, querying the local GPU) to an injected GpuClientBackend
// (per TASK.md: "delegation, not inheritance"). A process uses the singleton
// getInstance(); tests construct it directly with a mock backend + explicit socket.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"
#include "gpu_client_backend.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agnocast::cuda
{

// Retry policy for allocate() when the daemon reports no free slot. The daemon
// never blocks; the client retries here (see TASK.md / non-blocking daemon).
struct AllocRetryPolicy
{
  int max_attempts = 1000;
  std::chrono::microseconds delay{1000};  // 1 ms between attempts
};

class GpuSharedMemoryPoolProxy
{
public:
  // Dependency-injecting constructor. If socket_path is empty it is derived from
  // the local GPU UUID at initialize() time. The backend must outlive the proxy.
  explicit GpuSharedMemoryPoolProxy(
    GpuClientBackend & backend, std::string socket_path = "",
    AllocRetryPolicy retry_policy = AllocRetryPolicy{});
  ~GpuSharedMemoryPoolProxy();

  GpuSharedMemoryPoolProxy(const GpuSharedMemoryPoolProxy &) = delete;
  GpuSharedMemoryPoolProxy & operator=(const GpuSharedMemoryPoolProxy &) = delete;

  // Lazily creates and initializes the process singleton (real CUDA backend,
  // UUID-derived socket). Returns nullptr if initialization fails.
  static GpuSharedMemoryPoolProxy * getInstance();

  // Connects, handshakes (verifying backend + GPU), lists and imports every slot.
  // Returns false on failure. Call once (subsequent calls are no-ops returning
  // the current state).
  bool initialize();

  // Releases all imported slots and closes the connection. Idempotent.
  void finalize();

  // Reserves a pooled slot of at least `size` bytes and returns its local device
  // pointer. Retries per AllocRetryPolicy while the daemon reports no free slot.
  // Returns false if unsatisfiable (too large) or unavailable after retries.
  bool allocateMemory(void ** device_ptr, size_t size);

  // Returns a pooled pointer previously handed out by allocateMemory(). No-op if
  // the pointer is unknown to this proxy.
  void freeMemory(void * device_ptr);

  bool getSlotIdFromDevicePtr(void * device_ptr, std::uint32_t & slot_id);
  bool getDevicePtrFromSlotId(std::uint32_t slot_id, void *& device_ptr);

private:
  struct ProxySlot
  {
    gpu_shared_memory_daemon::SlotDescriptor descriptor;
    ImportedSlot imported;
  };

  // Socket lifecycle. daemonIOMutex_ must be held.
  bool connectSocket();
  void closeSocket();
  bool handshakeAndImport();  // handshake + list + import all slots
  void releaseImportedSlots();

  // One request/response round-trip, reconnecting+reinitializing once on I/O
  // error. daemonIOMutex_ must be held.
  bool transact(
    gpu_shared_memory_daemon::MessageType request_type,
    const std::vector<std::uint8_t> & request_payload,
    gpu_shared_memory_daemon::MessageType expected_response_type,
    gpu_shared_memory_daemon::MessageHeader & response_header,
    std::vector<std::uint8_t> & response_payload);

  GpuClientBackend & backend_;
  std::string socket_path_;
  AllocRetryPolicy retry_policy_;

  std::mutex daemonIOMutex_;        // serializes socket I/O
  std::mutex slotManagementMutex_;  // guards the maps below

  int socket_fd_ = -1;
  bool initialized_ = false;

  std::unordered_map<std::uint32_t, ProxySlot> slots_;  // by slot id
  std::unordered_map<void *, std::uint32_t> device_ptr_to_slot_id_;
};

}  // namespace agnocast::cuda
