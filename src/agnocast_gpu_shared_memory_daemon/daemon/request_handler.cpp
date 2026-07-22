#include "request_handler.hpp"

#include "gpu_shared_memory_pool.hpp"

namespace agnocast::gpu_shared_memory_daemon
{

RequestHandler::RequestHandler(GpuSharedMemoryPool & pool) : pool_(pool)
{
}

bool RequestHandler::handle(
  const MessageHeader & header, const std::vector<std::uint8_t> & payload,
  MessageType & response_type, std::vector<std::uint8_t> & response_payload)
{
  switch (static_cast<MessageType>(header.type)) {
    case MessageType::kHandshakeRequest: {
      HandshakeResponse response;
      response.backend_type = static_cast<std::uint32_t>(pool_.backend_type());
      response.gpu_uuid = pool_.gpu_uuid();
      response_type = MessageType::kHandshakeResponse;
      response_payload = serialize_handshake_response(response);
      return true;
    }

    case MessageType::kListRequest: {
      response_type = MessageType::kListResponse;
      response_payload = serialize_list_response(pool_.make_list_response());
      return true;
    }

    case MessageType::kAllocRequest: {
      AllocRequest request;
      if (!deserialize_alloc_request(payload.data(), payload.size(), &request)) {
        return false;
      }
      std::uint32_t slot_id = 0;
      const Status status = pool_.allocate(request.size, slot_id);
      AllocResponse response;
      response.status = static_cast<std::uint32_t>(status);
      response.slot_id = (status == Status::kOk) ? slot_id : 0;
      response_type = MessageType::kAllocResponse;
      response_payload = serialize_alloc_response(response);
      return true;
    }

    case MessageType::kFreeRequest: {
      FreeRequest request;
      if (!deserialize_free_request(payload.data(), payload.size(), &request)) {
        return false;
      }
      FreeResponse response;
      response.status = static_cast<std::uint32_t>(pool_.free_slot(request.slot_id));
      response_type = MessageType::kFreeResponse;
      response_payload = serialize_free_response(response);
      return true;
    }

    default:
      // Response message types and anything unknown are not valid requests.
      return false;
  }
}

}  // namespace agnocast::gpu_shared_memory_daemon
