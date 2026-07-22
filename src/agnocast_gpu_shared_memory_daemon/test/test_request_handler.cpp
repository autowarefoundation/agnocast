// Unit tests for RequestHandler, driving a real pool with the mock backend.
#include "gpu_shared_memory_pool.hpp"
#include "mock_slot_backend.hpp"
#include "request_handler.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;
using proto::test::MockSlotBackend;
using proto::test::two_class_config;

namespace
{

// Wraps a header of the given request type around a payload.
proto::MessageHeader request_header(proto::MessageType type, std::size_t payload_size)
{
  proto::MessageHeader header;
  header.type = static_cast<std::uint32_t>(type);
  header.payload_size = static_cast<std::uint32_t>(payload_size);
  return header;
}

}  // namespace

TEST(RequestHandler, HandshakeReturnsBackendAndUuid)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  ASSERT_TRUE(
    handler.handle(request_header(proto::MessageType::kHandshakeRequest, 0), {}, type, payload));
  EXPECT_EQ(type, proto::MessageType::kHandshakeResponse);

  proto::HandshakeResponse response;
  ASSERT_TRUE(proto::deserialize_handshake_response(payload.data(), payload.size(), &response));
  EXPECT_EQ(response.backend_type, static_cast<std::uint32_t>(proto::BackendType::kCudaIpc));
  EXPECT_EQ(response.gpu_uuid, "GPU-mock-0001");
}

TEST(RequestHandler, ListReturnsAllSlots)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  ASSERT_TRUE(
    handler.handle(request_header(proto::MessageType::kListRequest, 0), {}, type, payload));
  EXPECT_EQ(type, proto::MessageType::kListResponse);

  proto::ListResponse response;
  ASSERT_TRUE(proto::deserialize_list_response(payload.data(), payload.size(), &response));
  EXPECT_EQ(response.slots.size(), 5u);
}

TEST(RequestHandler, AllocThenFreeSucceed)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::AllocRequest alloc_request;
  alloc_request.size = 1000;
  const auto alloc_payload = proto::serialize_alloc_request(alloc_request);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  ASSERT_TRUE(handler.handle(
    request_header(proto::MessageType::kAllocRequest, alloc_payload.size()), alloc_payload, type,
    payload));
  EXPECT_EQ(type, proto::MessageType::kAllocResponse);

  proto::AllocResponse alloc_response;
  ASSERT_TRUE(proto::deserialize_alloc_response(payload.data(), payload.size(), &alloc_response));
  ASSERT_EQ(alloc_response.status, static_cast<std::uint32_t>(proto::Status::kOk));

  proto::FreeRequest free_request;
  free_request.slot_id = alloc_response.slot_id;
  const auto free_payload = proto::serialize_free_request(free_request);
  ASSERT_TRUE(handler.handle(
    request_header(proto::MessageType::kFreeRequest, free_payload.size()), free_payload, type,
    payload));
  EXPECT_EQ(type, proto::MessageType::kFreeResponse);

  proto::FreeResponse free_response;
  ASSERT_TRUE(proto::deserialize_free_response(payload.data(), payload.size(), &free_response));
  EXPECT_EQ(free_response.status, static_cast<std::uint32_t>(proto::Status::kOk));
}

TEST(RequestHandler, AllocTooLargeReportsStatus)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::AllocRequest alloc_request;
  alloc_request.size = 1u << 20;  // larger than the largest (4096) size class
  const auto alloc_payload = proto::serialize_alloc_request(alloc_request);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  ASSERT_TRUE(handler.handle(
    request_header(proto::MessageType::kAllocRequest, alloc_payload.size()), alloc_payload, type,
    payload));

  proto::AllocResponse response;
  ASSERT_TRUE(proto::deserialize_alloc_response(payload.data(), payload.size(), &response));
  EXPECT_EQ(response.status, static_cast<std::uint32_t>(proto::Status::kSizeTooLarge));
}

TEST(RequestHandler, MalformedAllocRequestClosesConnection)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  // AllocRequest needs 9 payload bytes; an empty payload cannot be parsed.
  EXPECT_FALSE(
    handler.handle(request_header(proto::MessageType::kAllocRequest, 0), {}, type, payload));
}

TEST(RequestHandler, UnexpectedMessageTypeClosesConnection)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));
  proto::RequestHandler handler(pool);

  proto::MessageType type;
  std::vector<std::uint8_t> payload;
  // A response type is not a valid request.
  EXPECT_FALSE(
    handler.handle(request_header(proto::MessageType::kAllocResponse, 0), {}, type, payload));
}
