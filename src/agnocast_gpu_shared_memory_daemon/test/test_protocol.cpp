// Unit tests for the daemon <-> proxy wire protocol (de)serialization.
#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <numeric>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;

namespace
{

// Fills a handle blob with a deterministic, non-trivial byte pattern.
proto::IpcHandleBlob make_blob(std::uint8_t seed)
{
  proto::IpcHandleBlob blob{};
  for (std::size_t i = 0; i < blob.size(); ++i) {
    blob[i] = static_cast<std::uint8_t>(seed + i);
  }
  return blob;
}

}  // namespace

TEST(ProtocolHeader, RoundTrip)
{
  proto::MessageHeader in;
  in.type = static_cast<std::uint32_t>(proto::MessageType::kAllocResponse);
  in.payload_size = 12345u;

  const auto bytes = proto::serialize_header(in);
  ASSERT_EQ(bytes.size(), proto::kHeaderWireSize);

  proto::MessageHeader out;
  ASSERT_TRUE(proto::deserialize_header(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.magic, proto::kProtocolMagic);
  EXPECT_EQ(out.version, proto::kProtocolVersion);
  EXPECT_EQ(out.type, in.type);
  EXPECT_EQ(out.payload_size, in.payload_size);
  EXPECT_TRUE(proto::header_is_valid(out));
}

TEST(ProtocolHeader, LittleEndianEncoding)
{
  proto::MessageHeader in;
  in.magic = 0x04030201u;
  in.version = 0x08070605u;
  in.type = 0x0c0b0a09u;
  in.payload_size = 0x100f0e0du;

  const auto bytes = proto::serialize_header(in);
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    EXPECT_EQ(bytes[i], static_cast<std::uint8_t>(i + 1)) << "at byte " << i;
  }
}

TEST(ProtocolHeader, RejectsTruncatedInput)
{
  proto::MessageHeader in;
  const auto bytes = proto::serialize_header(in);
  proto::MessageHeader out;
  EXPECT_FALSE(proto::deserialize_header(bytes.data(), bytes.size() - 1, &out));
}

TEST(ProtocolHeader, ValidityChecks)
{
  proto::MessageHeader header;
  EXPECT_TRUE(proto::header_is_valid(header));

  header.magic = 0xdeadbeefu;
  EXPECT_FALSE(proto::header_is_valid(header));

  header.magic = proto::kProtocolMagic;
  header.version = proto::kProtocolVersion + 1u;
  EXPECT_FALSE(proto::header_is_valid(header));
}

TEST(ProtocolAllocRequest, RoundTrip)
{
  proto::AllocRequest in;
  in.size = 0x0123456789abcdefull;
  in.non_blocking = 1;

  const auto bytes = proto::serialize_alloc_request(in);
  ASSERT_EQ(bytes.size(), proto::kAllocRequestWireSize);

  proto::AllocRequest out;
  ASSERT_TRUE(proto::deserialize_alloc_request(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.size, in.size);
  EXPECT_EQ(out.non_blocking, in.non_blocking);
}

TEST(ProtocolAllocRequest, RejectsTruncatedInput)
{
  proto::AllocRequest in;
  in.size = 42;
  const auto bytes = proto::serialize_alloc_request(in);
  proto::AllocRequest out;
  EXPECT_FALSE(proto::deserialize_alloc_request(bytes.data(), bytes.size() - 1, &out));
}

TEST(ProtocolAllocResponse, RoundTrip)
{
  proto::AllocResponse in;
  in.status = static_cast<std::uint32_t>(proto::Status::kNoFreeSlot);
  in.slot_id = 7;

  const auto bytes = proto::serialize_alloc_response(in);
  ASSERT_EQ(bytes.size(), proto::kAllocResponseWireSize);

  proto::AllocResponse out;
  ASSERT_TRUE(proto::deserialize_alloc_response(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.status, in.status);
  EXPECT_EQ(out.slot_id, in.slot_id);
}

TEST(ProtocolFreeRequest, RoundTrip)
{
  proto::FreeRequest in;
  in.slot_id = 0xabcdef01u;

  const auto bytes = proto::serialize_free_request(in);
  ASSERT_EQ(bytes.size(), proto::kFreeRequestWireSize);

  proto::FreeRequest out;
  ASSERT_TRUE(proto::deserialize_free_request(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.slot_id, in.slot_id);
}

TEST(ProtocolFreeResponse, RoundTrip)
{
  proto::FreeResponse in;
  in.status = static_cast<std::uint32_t>(proto::Status::kInvalidSlot);

  const auto bytes = proto::serialize_free_response(in);
  ASSERT_EQ(bytes.size(), proto::kFreeResponseWireSize);

  proto::FreeResponse out;
  ASSERT_TRUE(proto::deserialize_free_response(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.status, in.status);
}

TEST(ProtocolListResponse, RoundTripEmpty)
{
  proto::ListResponse in;
  const auto bytes = proto::serialize_list_response(in);

  proto::ListResponse out;
  ASSERT_TRUE(proto::deserialize_list_response(bytes.data(), bytes.size(), &out));
  EXPECT_TRUE(out.slots.empty());
}

TEST(ProtocolListResponse, RoundTripMultipleSlots)
{
  proto::ListResponse in;
  for (std::uint32_t i = 0; i < 3; ++i) {
    proto::SlotDescriptor slot;
    slot.slot_id = i;
    slot.size_class_index = i % 2;
    slot.slot_size = (i + 1) * 1024ull * 1024ull;
    slot.mem_handle = make_blob(static_cast<std::uint8_t>(i));
    slot.data_ready_event = make_blob(static_cast<std::uint8_t>(i + 64));
    slot.data_done_event = make_blob(static_cast<std::uint8_t>(i + 128));
    in.slots.push_back(slot);
  }

  const auto bytes = proto::serialize_list_response(in);
  ASSERT_EQ(bytes.size(), 4 + in.slots.size() * proto::kSlotDescriptorWireSize);

  proto::ListResponse out;
  ASSERT_TRUE(proto::deserialize_list_response(bytes.data(), bytes.size(), &out));
  ASSERT_EQ(out.slots.size(), in.slots.size());
  for (std::size_t i = 0; i < out.slots.size(); ++i) {
    EXPECT_EQ(out.slots[i].slot_id, in.slots[i].slot_id);
    EXPECT_EQ(out.slots[i].size_class_index, in.slots[i].size_class_index);
    EXPECT_EQ(out.slots[i].slot_size, in.slots[i].slot_size);
    EXPECT_EQ(out.slots[i].mem_handle, in.slots[i].mem_handle);
    EXPECT_EQ(out.slots[i].data_ready_event, in.slots[i].data_ready_event);
    EXPECT_EQ(out.slots[i].data_done_event, in.slots[i].data_done_event);
  }
}

TEST(ProtocolListResponse, RejectsTruncatedInput)
{
  proto::ListResponse in;
  proto::SlotDescriptor slot;
  slot.slot_id = 1;
  in.slots.push_back(slot);
  const auto bytes = proto::serialize_list_response(in);

  proto::ListResponse out;
  // Chop off the last byte: the declared slot cannot be fully read.
  EXPECT_FALSE(proto::deserialize_list_response(bytes.data(), bytes.size() - 1, &out));
}

TEST(ProtocolFraming, HeaderPlusPayload)
{
  proto::AllocRequest request;
  request.size = 4096;
  request.non_blocking = 0;
  const auto payload = proto::serialize_alloc_request(request);
  const auto framed = proto::frame_message(proto::MessageType::kAllocRequest, payload);

  ASSERT_EQ(framed.size(), proto::kHeaderWireSize + payload.size());

  proto::MessageHeader header;
  ASSERT_TRUE(proto::deserialize_header(framed.data(), framed.size(), &header));
  EXPECT_TRUE(proto::header_is_valid(header));
  EXPECT_EQ(header.type, static_cast<std::uint32_t>(proto::MessageType::kAllocRequest));
  EXPECT_EQ(header.payload_size, payload.size());

  proto::AllocRequest decoded;
  ASSERT_TRUE(proto::deserialize_alloc_request(
    framed.data() + proto::kHeaderWireSize, header.payload_size, &decoded));
  EXPECT_EQ(decoded.size, request.size);
  EXPECT_EQ(decoded.non_blocking, request.non_blocking);
}
