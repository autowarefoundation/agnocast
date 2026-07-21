// Wire protocol for communication between GpuSharedMemoryPoolDaemon and
// GpuSharedMemoryPoolProxy over a Unix domain socket (SOCK_STREAM).
//
// This header is intentionally free of any CUDA dependency: exported IPC handles
// (cudaIpcMemHandle_t / cudaIpcEventHandle_t) are byte blobs of fixed size
// (CUDA_IPC_HANDLE_SIZE == 64) and are carried here as std::array<uint8_t, 64>.
// The daemon and the proxy memcpy between the concrete CUDA types and these blobs.
//
// The protocol is used only for local, same-host, same-build IPC. Nevertheless,
// all integers are encoded little-endian explicitly (not by struct memcpy) so the
// wire format is well defined independently of struct padding/alignment, and so the
// (de)serialization can be unit tested without a socket.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

// Size of a CUDA IPC handle blob (CUDA_IPC_HANDLE_SIZE). Both cudaIpcMemHandle_t
// and cudaIpcEventHandle_t are 64-byte opaque structs on the stable CUDA ABI.
constexpr std::size_t kIpcHandleSize = 64;

using IpcHandleBlob = std::array<std::uint8_t, kIpcHandleSize>;

// Magic marks the start of every message: ASCII "AGPD" (Agnocast GPU Pool Daemon).
constexpr std::uint32_t kProtocolMagic = 0x41475044u;
// Bumped whenever the wire format changes incompatibly.
constexpr std::uint32_t kProtocolVersion = 1u;

enum class MessageType : std::uint32_t {
  kListRequest = 1,
  kListResponse = 2,
  kAllocRequest = 3,
  kAllocResponse = 4,
  kFreeRequest = 5,
  kFreeResponse = 6,
};

// Result codes returned in response messages.
enum class Status : std::uint32_t {
  kOk = 0,
  kNoFreeSlot = 1,     // No free slot large enough (non-blocking alloc).
  kSizeTooLarge = 2,   // Requested size exceeds the largest configured slot.
  kInvalidSlot = 3,    // Free request referenced an unknown / not-allocated slot.
  kInternalError = 4,  // Daemon-side failure (e.g. CUDA error).
};

// Fixed 16-byte framing header prepended to every message payload.
struct MessageHeader
{
  std::uint32_t magic = kProtocolMagic;
  std::uint32_t version = kProtocolVersion;
  std::uint32_t type = 0;          // MessageType
  std::uint32_t payload_size = 0;  // bytes of payload following this header
};
constexpr std::size_t kHeaderWireSize = 16;

// Describes a single slot exported by the daemon. Sent in a ListResponse.
struct SlotDescriptor
{
  std::uint32_t slot_id = 0;
  std::uint32_t size_class_index = 0;  // index into the daemon's configured size classes
  std::uint64_t slot_size = 0;         // capacity of the slot in bytes
  IpcHandleBlob mem_handle{};          // cudaIpcMemHandle_t for the GPU memory block
  IpcHandleBlob data_ready_event{};    // cudaIpcEventHandle_t: write-complete event
  IpcHandleBlob data_done_event{};     // cudaIpcEventHandle_t: read-complete event
};
constexpr std::size_t kSlotDescriptorWireSize = 4 + 4 + 8 + 3 * kIpcHandleSize;

// ---- Payloads ----

// kListRequest has no payload.

struct ListResponse
{
  std::vector<SlotDescriptor> slots;
};

struct AllocRequest
{
  std::uint64_t size = 0;         // requested minimum size in bytes
  std::uint8_t non_blocking = 0;  // 1 = return immediately if no slot is free
};
constexpr std::size_t kAllocRequestWireSize = 8 + 1;

struct AllocResponse
{
  std::uint32_t status = 0;   // Status
  std::uint32_t slot_id = 0;  // valid only when status == kOk
};
constexpr std::size_t kAllocResponseWireSize = 4 + 4;

struct FreeRequest
{
  std::uint32_t slot_id = 0;
};
constexpr std::size_t kFreeRequestWireSize = 4;

struct FreeResponse
{
  std::uint32_t status = 0;  // Status
};
constexpr std::size_t kFreeResponseWireSize = 4;

// ---------------------------------------------------------------------------
// Little-endian primitive encode/decode helpers.
// ---------------------------------------------------------------------------
namespace detail
{

inline void put_u32(std::vector<std::uint8_t> & buf, std::uint32_t value)
{
  buf.push_back(static_cast<std::uint8_t>(value & 0xffu));
  buf.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
  buf.push_back(static_cast<std::uint8_t>((value >> 16) & 0xffu));
  buf.push_back(static_cast<std::uint8_t>((value >> 24) & 0xffu));
}

inline void put_u64(std::vector<std::uint8_t> & buf, std::uint64_t value)
{
  for (int i = 0; i < 8; ++i) {
    buf.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu));
  }
}

inline void put_blob(std::vector<std::uint8_t> & buf, const IpcHandleBlob & blob)
{
  buf.insert(buf.end(), blob.begin(), blob.end());
}

// Reads from data[*offset], advances *offset, and returns false on overrun.
inline bool get_u32(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, std::uint32_t * out)
{
  if (*offset + 4 > size) {
    return false;
  }
  *out = static_cast<std::uint32_t>(data[*offset]) |
         (static_cast<std::uint32_t>(data[*offset + 1]) << 8) |
         (static_cast<std::uint32_t>(data[*offset + 2]) << 16) |
         (static_cast<std::uint32_t>(data[*offset + 3]) << 24);
  *offset += 4;
  return true;
}

inline bool get_u64(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, std::uint64_t * out)
{
  if (*offset + 8 > size) {
    return false;
  }
  std::uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value |= static_cast<std::uint64_t>(data[*offset + static_cast<std::size_t>(i)]) << (8 * i);
  }
  *offset += 8;
  *out = value;
  return true;
}

inline bool get_blob(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, IpcHandleBlob * out)
{
  if (*offset + kIpcHandleSize > size) {
    return false;
  }
  std::memcpy(out->data(), data + *offset, kIpcHandleSize);
  *offset += kIpcHandleSize;
  return true;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Header (de)serialization.
// ---------------------------------------------------------------------------
inline std::array<std::uint8_t, kHeaderWireSize> serialize_header(const MessageHeader & header)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(kHeaderWireSize);
  detail::put_u32(buf, header.magic);
  detail::put_u32(buf, header.version);
  detail::put_u32(buf, header.type);
  detail::put_u32(buf, header.payload_size);
  std::array<std::uint8_t, kHeaderWireSize> out{};
  std::memcpy(out.data(), buf.data(), kHeaderWireSize);
  return out;
}

inline bool deserialize_header(const std::uint8_t * data, std::size_t size, MessageHeader * out)
{
  std::size_t offset = 0;
  return detail::get_u32(data, size, &offset, &out->magic) &&
         detail::get_u32(data, size, &offset, &out->version) &&
         detail::get_u32(data, size, &offset, &out->type) &&
         detail::get_u32(data, size, &offset, &out->payload_size);
}

// Returns true when the header is well formed and understood by this build.
inline bool header_is_valid(const MessageHeader & header)
{
  return header.magic == kProtocolMagic && header.version == kProtocolVersion;
}

// ---------------------------------------------------------------------------
// Payload (de)serialization.
// ---------------------------------------------------------------------------
inline void serialize_slot_descriptor(std::vector<std::uint8_t> & buf, const SlotDescriptor & slot)
{
  detail::put_u32(buf, slot.slot_id);
  detail::put_u32(buf, slot.size_class_index);
  detail::put_u64(buf, slot.slot_size);
  detail::put_blob(buf, slot.mem_handle);
  detail::put_blob(buf, slot.data_ready_event);
  detail::put_blob(buf, slot.data_done_event);
}

inline bool deserialize_slot_descriptor(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, SlotDescriptor * out)
{
  return detail::get_u32(data, size, offset, &out->slot_id) &&
         detail::get_u32(data, size, offset, &out->size_class_index) &&
         detail::get_u64(data, size, offset, &out->slot_size) &&
         detail::get_blob(data, size, offset, &out->mem_handle) &&
         detail::get_blob(data, size, offset, &out->data_ready_event) &&
         detail::get_blob(data, size, offset, &out->data_done_event);
}

inline std::vector<std::uint8_t> serialize_list_response(const ListResponse & response)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(4 + response.slots.size() * kSlotDescriptorWireSize);
  detail::put_u32(buf, static_cast<std::uint32_t>(response.slots.size()));
  for (const auto & slot : response.slots) {
    serialize_slot_descriptor(buf, slot);
  }
  return buf;
}

inline bool deserialize_list_response(
  const std::uint8_t * data, std::size_t size, ListResponse * out)
{
  std::size_t offset = 0;
  std::uint32_t count = 0;
  if (!detail::get_u32(data, size, &offset, &count)) {
    return false;
  }
  out->slots.clear();
  out->slots.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    SlotDescriptor slot;
    if (!deserialize_slot_descriptor(data, size, &offset, &slot)) {
      return false;
    }
    out->slots.push_back(slot);
  }
  return true;
}

inline std::vector<std::uint8_t> serialize_alloc_request(const AllocRequest & request)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(kAllocRequestWireSize);
  detail::put_u64(buf, request.size);
  buf.push_back(request.non_blocking);
  return buf;
}

inline bool deserialize_alloc_request(
  const std::uint8_t * data, std::size_t size, AllocRequest * out)
{
  std::size_t offset = 0;
  if (!detail::get_u64(data, size, &offset, &out->size)) {
    return false;
  }
  if (offset + 1 > size) {
    return false;
  }
  out->non_blocking = data[offset];
  return true;
}

inline std::vector<std::uint8_t> serialize_alloc_response(const AllocResponse & response)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(kAllocResponseWireSize);
  detail::put_u32(buf, response.status);
  detail::put_u32(buf, response.slot_id);
  return buf;
}

inline bool deserialize_alloc_response(
  const std::uint8_t * data, std::size_t size, AllocResponse * out)
{
  std::size_t offset = 0;
  return detail::get_u32(data, size, &offset, &out->status) &&
         detail::get_u32(data, size, &offset, &out->slot_id);
}

inline std::vector<std::uint8_t> serialize_free_request(const FreeRequest & request)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(kFreeRequestWireSize);
  detail::put_u32(buf, request.slot_id);
  return buf;
}

inline bool deserialize_free_request(const std::uint8_t * data, std::size_t size, FreeRequest * out)
{
  std::size_t offset = 0;
  return detail::get_u32(data, size, &offset, &out->slot_id);
}

inline std::vector<std::uint8_t> serialize_free_response(const FreeResponse & response)
{
  std::vector<std::uint8_t> buf;
  buf.reserve(kFreeResponseWireSize);
  detail::put_u32(buf, response.status);
  return buf;
}

inline bool deserialize_free_response(
  const std::uint8_t * data, std::size_t size, FreeResponse * out)
{
  std::size_t offset = 0;
  return detail::get_u32(data, size, &offset, &out->status);
}

// Convenience: build a complete framed message (header + payload) ready to write().
inline std::vector<std::uint8_t> frame_message(
  MessageType type, const std::vector<std::uint8_t> & payload)
{
  MessageHeader header;
  header.type = static_cast<std::uint32_t>(type);
  header.payload_size = static_cast<std::uint32_t>(payload.size());
  const auto header_bytes = serialize_header(header);

  std::vector<std::uint8_t> out;
  out.reserve(kHeaderWireSize + payload.size());
  out.insert(out.end(), header_bytes.begin(), header_bytes.end());
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

}  // namespace agnocast::gpu_shared_memory_daemon
