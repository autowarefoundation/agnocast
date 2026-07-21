// Internal header — kept in src/ so it is NOT installed or visible to downstream packages.
// The singleton instance of this class in a process communicates with the GpuSharedMemoryDaemon
// to manage shared memory buffers for GPU transfers.
// For client processes, GpuSharedMemoryPoolProxy forwards calls to the daemon and hides the IPC details,
// which are different depending on the platform (dedicated GPU or integrated GPU).
// This interface is implemented by backends.
#pragma once

#include "gpu_shared_memory_pool_proxy.hpp"

#include <cuda_runtime.h>

namespace agnocast::cuda
{

// @brief Internal structure representing the details of a shared memory slot handle for CUDA IPC backend.
typedef struct InternalSlotHandle_st {
    // @brief Buffer object that the publisher process will use to share GPU memory with the subscriber process.
    const NvSciBufObj & dataBuffer;
    // @brief The size of actual data stored in dataBuffer.
    const size_t dataSize;
    // @brief Barrier object that the publisher process will signal when the data in the buffer is ready to be consumed.
    NvSciSyncObj & dataReadyBarrier;
    // @brief Barrier object that the subscriber process will signal when it has finished consuming the data.
    NvSciSyncObj & dataDoneBarrier;
} InternalSlotHandle;

class GpuSharedMemoryPoolProxyNvSci : public GpuSharedMemoryPoolProxy
{
public:
  /** @brief Allocates memory on the GPU.
   *  @param[out] device_ptr A pointer to the allocated GPU memory.
   *  @param[in] size The size of the memory to allocate.
   *  @return True if the allocation was successful, false otherwise.
   */
  bool allocateMemory(void** device_ptr, size_t size) override;

  /** @brief Frees previously allocated GPU memory.
   *  @param[in] device_ptr A pointer to the GPU memory to free.
   */
  void freeMemory(void* device_ptr) override;


private:
  // Singleton pattern
  GpuSharedMemoryPoolProxyNvSci() = default;
  GpuSharedMemoryPoolProxyNvSci(const GpuSharedMemoryPoolProxyNvSci &) = delete;
  GpuSharedMemoryPoolProxyNvSci & operator=(const GpuSharedMemoryPoolProxyNvSci &) = delete;
  virtual ~GpuSharedMemoryPoolProxyNvSci() = default;

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
  bool convertSlotHandleToInternalHandle(const SlotHandle & handle, InternalSlotHandle & internal_handle);
};

}  // namespace agnocast::cuda