// Internal header — kept in src/ so it is NOT installed or visible to downstream packages.
// The singleton instance of this class in a process communicates with the GpuSharedMemoryDaemon
// to manage shared memory buffers for GPU transfers.
// For client processes, GpuSharedMemoryPoolProxy forwards calls to the daemon and hides the IPC details,
// which are different depending on the platform (dedicated GPU or integrated GPU).
// This interface is implemented by backends.
#pragma once

#include "agnocast/gpu_transfer_backend.hpp"

namespace agnocast::cuda
{

struct SlotHandle_st;
using SlotHandle = SlotHandle_st;

class GpuSharedMemoryPoolProxy
{
public:
  /** @brief Allocates memory on the GPU.
   *  @param[out] device_ptr A pointer to the allocated GPU memory.
   *  @param[in] size The size of the memory to allocate.
   *  @return True if the allocation was successful, false otherwise.
   */
  virtual bool allocateMemory(void** device_ptr, size_t size) = 0;

  /** @brief Frees previously allocated GPU memory.
   *  @param[in] device_ptr A pointer to the GPU memory to free.
   */
  virtual void freeMemory(void* device_ptr) = 0;

protected:
  // Singleton pattern
  GpuSharedMemoryPoolProxy() = default;
  GpuSharedMemoryPoolProxy(const GpuSharedMemoryPoolProxy &) = delete;
  GpuSharedMemoryPoolProxy & operator=(const GpuSharedMemoryPoolProxy &) = delete;
  virtual ~GpuSharedMemoryPoolProxy() = default;

  /** @brief Establishes connection to the GpuSharedMemoryDaemon and imports the exported resource handles.
   *  @return True if the initialization was successful, false otherwise.
   *  @note Importing of the handles may take some time, so it should be done in process initialization phase.
   */
  virtual bool initialize() = 0;

  /** @brief Disconnects from the GpuSharedMemoryDaemon and releases all imported resource handles.
   *  @note  Should be called during process shutdown to clean up resources.
   */

  virtual void finalize() = 0;
  
  /** @brief Allocates a slot in the shared memory pool.
   *  @param[in] size The size of the slot to allocate.
   *  @param[out] handle A reference to a SlotHandle that will be populated with the allocated slot's handle.
   *  @param[in] non_blocking If false, the allocation will block until a slot becomes available.
   * If true, the allocation will return immediately if no slot is available.
   *  @return True if the allocation was successful, false otherwise.
   */
  virtual bool allocateSlot(size_t size, SlotHandle & handle, bool non_blocking = false) = 0;

  /** @brief Frees a previously allocated slot.
   *  @param[in] handle The handle of the slot to free.
   */
  virtual void freeSlot(const SlotHandle & handle) = 0;
};

}  // namespace agnocast::cuda