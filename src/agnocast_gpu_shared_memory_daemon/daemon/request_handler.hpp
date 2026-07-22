// Turns one decoded protocol request into a response, by dispatching to the pool.
// Pure and connection-agnostic (no sockets), so it is unit tested directly.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <cstdint>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

class GpuSharedMemoryPool;

class RequestHandler
{
public:
  explicit RequestHandler(GpuSharedMemoryPool & pool);

  // Handles one request. On success, writes the response message type and payload
  // and returns true. Returns false when the request is malformed or of an
  // unexpected type, signalling the caller to close the connection.
  bool handle(
    const MessageHeader & header, const std::vector<std::uint8_t> & payload,
    MessageType & response_type, std::vector<std::uint8_t> & response_payload);

private:
  GpuSharedMemoryPool & pool_;
};

}  // namespace agnocast::gpu_shared_memory_daemon
