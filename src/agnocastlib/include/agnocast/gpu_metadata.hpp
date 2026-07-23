#pragma once

#include <cstddef>
#include <cstdint>

namespace agnocast
{

// GPU sharing metadata stored in shared memory alongside the message.
// Allocated by the publish path (while the heaphook is active) so it lands in the
// publisher's shared memory region and is readable by subscribers.
//
// Pool model: the GPU buffer is a daemon-managed pool slot, so the message carries
// the slot id (which any process resolves to its own imported device pointer +
// events) rather than a per-message IPC handle.
struct GpuMetadata
{
  std::uint32_t slot_id;  // pool slot backing this message's GPU buffer
  size_t gpu_data_size;   // size of the GPU data in bytes
};

}  // namespace agnocast
