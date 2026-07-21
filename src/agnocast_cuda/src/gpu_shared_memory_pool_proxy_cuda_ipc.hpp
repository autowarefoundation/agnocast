// Internal header - kept in src/ so it is NOT installed or visible to downstream packages.
// The singleton instance of this class in a process communicates with the GpuSharedMemoryDaemon
// to manage shared memory buffers for GPU transfers.
// For client processes, GpuSharedMemoryPoolProxy forwards calls to the daemon and hides the IPC details,
// which are different depending on the platform (dedicated GPU or integrated GPU).
// This interface is implemented by backends.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"
#include "cudart_loader.hpp"
#include "gpu_shared_memory_pool_proxy.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agnocast::cuda
{

// Internal structure representing imported local objects for a slot.
typedef struct InternalSlotHandle_st {
  void * dataPtr;
  size_t dataSize;
  cudaEvent_t dataReadyEvent;
  cudaEvent_t dataDoneEvent;
} InternalSlotHandle;

// Daemon-exported CUDA IPC handles for a slot.
typedef struct CudaIpcSlotHandle_st {
  cudaIpcMemHandle_t ipcHandle;
  size_t dataSize;
  cudaIpcEventHandle_t dataReadyEvent;
  cudaIpcEventHandle_t dataDoneEvent;
} CudaIpcSlotHandle;
using CudaIpcSlotHandle_t = CudaIpcSlotHandle;

// Logical slot handle tracked by the proxy.
typedef struct SlotHandle_st {
  std::uint32_t shmSlotId;
  size_t shmSlotSize;
  bool is_allocated;
  CudaIpcSlotHandle_t cudaIpcHandle;
  InternalSlotHandle internalHandle;
} SlotHandle;

// This class is the singleton instance in a process that communicates with the GpuSharedMemoryDaemon.
class GpuSharedMemoryPoolProxyCudaIpc : public GpuSharedMemoryPoolProxy
{
public:
  // Lazily creates and initializes singleton instance. Returns nullptr on initialization failure.
  static GpuSharedMemoryPoolProxyCudaIpc * getInstance();

  /** @brief Allocates memory on the GPU.
   *  @param[out] device_ptr A pointer to the allocated GPU memory.
   *  @param[in] size The size of the memory to allocate.
   *  @return True if the allocation was successful, false otherwise.
   */
  bool allocateMemory(void ** device_ptr, size_t size) override;

  /** @brief Frees previously allocated GPU memory.
   *  @param[in] device_ptr A pointer to the GPU memory to free.
   */
  void freeMemory(void * device_ptr) override;

  /** @brief Translates a local GPU device pointer to its shared-memory slot ID.
   *  @param[in] device_ptr A local GPU device pointer managed by this proxy.
   *  @param[out] slot_id The corresponding shared-memory slot ID.
   *  @return True if the pointer is known to the proxy, false otherwise.
   */
  bool getSlotIdFromDevicePtr(void * device_ptr, std::uint32_t & slot_id);

  /** @brief Translates a shared-memory slot ID to its local GPU device pointer.
   *  @param[in] slot_id The shared-memory slot ID.
   *  @param[out] device_ptr The corresponding local GPU device pointer.
   *  @return True if the slot is known to the proxy, false otherwise.
   */
  bool getDevicePtrFromSlotId(std::uint32_t slot_id, void *& device_ptr);

private:
  // Singleton pattern
  GpuSharedMemoryPoolProxyCudaIpc() = default;
  GpuSharedMemoryPoolProxyCudaIpc(const GpuSharedMemoryPoolProxyCudaIpc &) = delete;
  GpuSharedMemoryPoolProxyCudaIpc & operator=(const GpuSharedMemoryPoolProxyCudaIpc &) = delete;
  ~GpuSharedMemoryPoolProxyCudaIpc() override;

  /** @brief Establishes connection to the GpuSharedMemoryDaemon and imports the exported resource handles.
   *  @return True if the initialization was successful, false otherwise.
   *  @note Importing of the handles may take some time, so it should be done in process initialization phase.
   */
  bool initialize() override;

  /** @brief Disconnects from the GpuSharedMemoryDaemon and releases all imported resource handles.
   *  @note  Should be called during process shutdown to clean up resources.
   */
  void finalize() override;

  /** @brief Allocates a slot in the shared memory pool.
   *  @param[in] size The size of the slot to allocate.
   *  @param[out] handle A reference to a SlotHandle that will be populated with the allocated slot's handle.
   *  @param[in] non_blocking If false, the allocation will block until a slot becomes available.
   * If true, the allocation will return immediately if no slot is available.
   *  @return True if the allocation was successful, false otherwise.
   */
  bool allocateSlot(size_t size, SlotHandle & handle, bool non_blocking = false) override;

  /** @brief Frees a previously allocated slot.
   *  @param[in] handle The handle of the slot to free.
   */
  void freeSlot(const SlotHandle & handle) override;

  /** @brief Converts a SlotHandle to an InternalSlotHandle.
   *  @param[in] handle The SlotHandle to convert.
   *  @param[out] internal_handle A reference to an InternalSlotHandle that will be populated with the converted handle.
   *  @return True if the conversion was successful, false otherwise.
   */
  bool convertSlotHandleToInternalHandle(
    const SlotHandle & handle, InternalSlotHandle & internal_handle);

  bool ensureConnected();
  bool connectDaemonSocket();
  void closeDaemonSocket();
  bool reinitializeAfterReconnect();
  bool refreshSlotCacheFromDaemon();

  // Mutex to protect access to the slot maps in multi-threaded scenarios.
  std::mutex slotManagementMutex_;
  // Socket I/O on one persistent fd must be serialized.
  std::mutex daemonIOMutex_;

  // Maps to keep track of allocated and free slots, keyed by shared memory slot ID.
  std::unordered_map<std::uint32_t, SlotHandle> slots_;
  // Map from device pointer to slot ID for quick lookup. This is needed for freeing memory by device pointer.
  std::unordered_map<void *, std::uint32_t> devicePtrToSlotIdMap_;

  std::string daemonSocketPath_;
  int daemonSocketFd_{-1};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> connected_{false};

  static std::mutex singletonMutex_;
  static GpuSharedMemoryPoolProxyCudaIpc * instance_;
};

}  // namespace agnocast::cuda
