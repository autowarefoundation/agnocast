// Unit tests for the daemon <-> proxy wire protocol (de)serialization.
#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;

namespace
{

// Builds a variable-length handle blob of `len` bytes with a deterministic pattern.
std::vector<std::uint8_t> make_blob(std::uint8_t seed, std::size_t len)
{
  std::vector<std::uint8_t> blob(len);
  for (std::size_t i = 0; i < len; ++i) {
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

TEST(ProtocolHandshakeResponse, RoundTrip)
{
  proto::HandshakeResponse in;
  in.backend_type = static_cast<std::uint32_t>(proto::BackendType::kCudaIpc);
  in.gpu_uuid = "GPU-01234567-89ab-cdef-0123-456789abcdef";

  const auto bytes = proto::serialize_handshake_response(in);

  proto::HandshakeResponse out;
  ASSERT_TRUE(proto::deserialize_handshake_response(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.backend_type, in.backend_type);
  EXPECT_EQ(out.gpu_uuid, in.gpu_uuid);
}

TEST(ProtocolHandshakeResponse, RoundTripEmptyUuid)
{
  proto::HandshakeResponse in;
  in.backend_type = static_cast<std::uint32_t>(proto::BackendType::kUnknown);
  in.gpu_uuid = "";

  const auto bytes = proto::serialize_handshake_response(in);

  proto::HandshakeResponse out;
  ASSERT_TRUE(proto::deserialize_handshake_response(bytes.data(), bytes.size(), &out));
  EXPECT_EQ(out.backend_type, in.backend_type);
  EXPECT_TRUE(out.gpu_uuid.empty());
}

TEST(ProtocolHandshakeResponse, RejectsTruncatedInput)
{
  proto::HandshakeResponse in;
  in.backend_type = static_cast<std::uint32_t>(proto::BackendType::kCudaIpc);
  in.gpu_uuid = "GPU-abc";
  const auto bytes = proto::serialize_handshake_response(in);

  proto::HandshakeResponse out;
  // Chop the last UUID byte: the declared string length exceeds the buffer.
  EXPECT_FALSE(proto::deserialize_handshake_response(bytes.data(), bytes.size() - 1, &out));
}

TEST(ProtocolSlotDescriptor, RoundTripVariableLengthHandles)
{
  // Exercises unequal blob lengths, including a larger-than-CUDA NvSci-sized blob
  // and an empty blob, to confirm handle size is not baked into the wire format.
  proto::SlotDescriptor in;
  in.slot_id = 3;
  in.size_class_index = 1;
  in.slot_size = 8ull * 1024ull * 1024ull;
  in.mem_handle = make_blob(0x10, 64);         // CUDA-sized
  in.data_ready_event = make_blob(0x20, 300);  // NvSci-sized (larger than 64)
  in.data_done_event = make_blob(0x30, 0);     // empty

  std::vector<std::uint8_t> buf;
  proto::serialize_slot_descriptor(buf, in);

  proto::SlotDescriptor out;
  std::size_t offset = 0;
  ASSERT_TRUE(proto::deserialize_slot_descriptor(buf.data(), buf.size(), &offset, &out));
  EXPECT_EQ(offset, buf.size());
  EXPECT_EQ(out.slot_id, in.slot_id);
  EXPECT_EQ(out.size_class_index, in.size_class_index);
  EXPECT_EQ(out.slot_size, in.slot_size);
  EXPECT_EQ(out.mem_handle, in.mem_handle);
  EXPECT_EQ(out.data_ready_event, in.data_ready_event);
  EXPECT_EQ(out.data_done_event, in.data_done_event);
}

TEST(ProtocolSlotDescriptor, EmptyHandlesHitMinimumWireSize)
{
  proto::SlotDescriptor in;  // all handles empty
  std::vector<std::uint8_t> buf;
  proto::serialize_slot_descriptor(buf, in);
  EXPECT_EQ(buf.size(), proto::kMinSlotDescriptorWireSize);
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
    slot.mem_handle = make_blob(static_cast<std::uint8_t>(i), 64);
    slot.data_ready_event = make_blob(static_cast<std::uint8_t>(i + 64), 64 + i);
    slot.data_done_event = make_blob(static_cast<std::uint8_t>(i + 128), 64);
    in.slots.push_back(slot);
  }

  const auto bytes = proto::serialize_list_response(in);

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
  slot.mem_handle = make_blob(0x01, 64);
  in.slots.push_back(slot);
  const auto bytes = proto::serialize_list_response(in);

  proto::ListResponse out;
  // Chop off the last byte: the declared slot cannot be fully read.
  EXPECT_FALSE(proto::deserialize_list_response(bytes.data(), bytes.size() - 1, &out));
}

TEST(ProtocolListResponse, RejectsImplausibleSlotCountWithoutOverAllocating)
{
  // Regression test for the unbounded reserve() bug: a frame that declares a huge
  // slot count but carries almost no payload must be rejected up front, not trigger
  // a multi-hundred-GiB speculative allocation.
  std::vector<std::uint8_t> buf;
  proto::detail::put_u32(buf, 0xffffffffu);  // count = ~4.29e9 slots
  // No slot bytes follow (only a couple of stray bytes).
  buf.push_back(0x00);
  buf.push_back(0x00);

  proto::ListResponse out;
  EXPECT_FALSE(proto::deserialize_list_response(buf.data(), buf.size(), &out));
  EXPECT_TRUE(out.slots.empty());
}

TEST(ProtocolListResponse, RejectsCountJustBeyondBuffer)
{
  // count is small enough not to overflow the division but still larger than the
  // buffer can hold: exercises the exact boundary of the cap.
  std::vector<std::uint8_t> buf;
  proto::detail::put_u32(buf, 2u);  // claim 2 slots
  // Provide only enough bytes for less than one minimal slot.
  for (std::size_t i = 0; i < proto::kMinSlotDescriptorWireSize - 1; ++i) {
    buf.push_back(0x00);
  }

  proto::ListResponse out;
  EXPECT_FALSE(proto::deserialize_list_response(buf.data(), buf.size(), &out));
}

TEST(ProtocolVarBlob, RejectsImplausibleBlobLength)
{
  // A slot descriptor whose first handle declares a huge length but provides no
  // bytes must be rejected without over-allocating.
  std::vector<std::uint8_t> buf;
  proto::detail::put_u32(buf, 0u);           // slot_id
  proto::detail::put_u32(buf, 0u);           // size_class_index
  proto::detail::put_u64(buf, 0u);           // slot_size
  proto::detail::put_u32(buf, 0xffffffffu);  // mem_handle length, no data follows

  proto::SlotDescriptor out;
  std::size_t offset = 0;
  EXPECT_FALSE(proto::deserialize_slot_descriptor(buf.data(), buf.size(), &offset, &out));
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

TEST(ProtocolSocketPath, DerivesFromGpuUuid)
{
  EXPECT_EQ(
    proto::socket_path_for_gpu("GPU-1234"),
    std::string(proto::kSocketDir) + "/gpu_shared_memory_daemon.GPU-1234.sock");
}

TEST(ProtocolSocketPath, DistinctUuidsGiveDistinctPaths)
{
  EXPECT_NE(proto::socket_path_for_gpu("GPU-aaaa"), proto::socket_path_for_gpu("GPU-bbbb"));
}

TEST(ProtocolSocketPath, SanitizesUnsafeCharacters)
{
  // Composite MIG identifiers contain '/', which must not create extra path
  // components; every unsafe character becomes '_'.
  const auto path = proto::socket_path_for_gpu("MIG-GPU-abc/3/0");
  EXPECT_EQ(
    path, std::string(proto::kSocketDir) + "/gpu_shared_memory_daemon.MIG-GPU-abc_3_0.sock");
  EXPECT_EQ(path.find("abc/3"), std::string::npos);
}
