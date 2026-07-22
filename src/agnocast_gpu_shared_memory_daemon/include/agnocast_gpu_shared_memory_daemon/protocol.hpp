// Wire protocol for communication between GpuSharedMemoryPoolDaemon and
// GpuSharedMemoryPoolProxy over a Unix domain socket (SOCK_STREAM).
//
// This header is intentionally free of any CUDA / NvSci dependency and does not
// bake in any infrastructure-specific handle size. Exported IPC handles are
// carried as length-prefixed, variable-length byte blobs, so the same wire
// format accommodates CUDA IPC handles (64 bytes) as well as the larger and
// variable-length NvSciBuf / NvSciSync export descriptors. A per-connection
// handshake identifies which backend a daemon speaks and which GPU it manages.
//
// The protocol is used only for local, same-host, same-build IPC. Nevertheless,
// all integers are encoded little-endian explicitly (not by struct memcpy) so the
// wire format is well defined independently of struct padding/alignment, and so the
// (de)serialization can be unit tested without a socket. Every length read off the
// wire is validated against the bytes actually available before any allocation, so
// a truncated or corrupt frame cannot trigger a huge speculative allocation.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

// Magic marks the start of every message: ASCII "AGPD" (Agnocast GPU Pool Daemon).
constexpr std::uint32_t kProtocolMagic = 0x41475044u;
// Bumped whenever the wire format changes incompatibly. Carried in every frame
// header and validated by header_is_valid(), so version negotiation happens at
// the framing level rather than in any payload.
constexpr std::uint32_t kProtocolVersion = 1u;

// Directory holding daemon sockets. The full socket path is derived from the GPU
// UUID at runtime (see socket_path_for_gpu) rather than being configured, so the
// socket a daemon binds always matches the GPU it manages.
constexpr const char * kSocketDir = "/run/agnocast";

enum class MessageType : std::uint32_t {
  kListRequest = 1,
  kListResponse = 2,
  kAllocRequest = 3,
  kAllocResponse = 4,
  kFreeRequest = 5,
  kFreeResponse = 6,
  kHandshakeRequest = 7,
  kHandshakeResponse = 8,
};

// Result codes returned in response messages.
enum class Status : std::uint32_t {
  kOk = 0,
  kNoFreeSlot = 1,     // No free slot large enough (non-blocking alloc).
  kSizeTooLarge = 2,   // Requested size exceeds the largest configured slot.
  kInvalidSlot = 3,    // Free request referenced an unknown / not-allocated slot.
  kInternalError = 4,  // Daemon-side failure (e.g. CUDA error).
};

// GPU memory sharing infrastructure a daemon speaks. All slots on one daemon share
// the same backend (one GPU), so this is exchanged once per connection, not per slot.
enum class BackendType : std::uint32_t {
  kUnknown = 0,
  kCudaIpc = 1,   // Discrete GPU, CUDA IPC handles.
  kNvSciBuf = 2,  // Tegra / DRIVE, NvSciBuf + NvSciSync (later phase).
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
// Handle fields are opaque, backend-specific export blobs of arbitrary length.
struct SlotDescriptor
{
  std::uint32_t slot_id = 0;
  std::uint32_t size_class_index = 0;          // index into the daemon's configured size classes
  std::uint64_t slot_size = 0;                 // capacity of the slot in bytes
  std::vector<std::uint8_t> mem_handle;        // GPU memory block export blob
  std::vector<std::uint8_t> data_ready_event;  // write-complete event/sync export blob
  std::vector<std::uint8_t> data_done_event;   // read-complete event/sync export blob
};
// Smallest possible on-wire SlotDescriptor: the fixed fields plus three empty
// length-prefixed blobs. Used to bound the slot count of a ListResponse before
// allocating (see deserialize_list_response).
constexpr std::size_t kMinSlotDescriptorWireSize = 4 + 4 + 8 + 3 * 4;

// ---- Payloads ----

// kListRequest and kHandshakeRequest have no payload.

struct ListResponse
{
  std::vector<SlotDescriptor> slots;
};

// Daemon -> proxy identity, exchanged once at connect. Lets the proxy verify the
// daemon speaks the expected backend and manages the expected GPU, and fail loud
// on mismatch. protocol_version is not repeated here: it lives in every frame header.
struct HandshakeResponse
{
  std::uint32_t backend_type = 0;  // BackendType
  std::string gpu_uuid;            // e.g. "GPU-xxxxxxxx-..." or a MIG instance UUID
};

struct AllocRequest
{
  std::uint64_t size = 0;  // requested minimum size in bytes
  // Advisory hint for the CLIENT's own retry behavior. The daemon NEVER blocks:
  // it always answers immediately (kOk / kNoFreeSlot / kSizeTooLarge). A client
  // that wants blocking semantics retries on kNoFreeSlot on its side.
  std::uint8_t non_blocking = 0;
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

// Writes a length-prefixed (uint32) byte range.
inline void put_var_bytes(
  std::vector<std::uint8_t> & buf, const std::uint8_t * data, std::size_t len)
{
  put_u32(buf, static_cast<std::uint32_t>(len));
  buf.insert(buf.end(), data, data + len);
}

// Reads a length-prefixed blob. The declared length is checked against the bytes
// actually remaining before resize(), so a corrupt length cannot over-allocate.
// get_u32 guarantees *offset <= size on success, so (size - *offset) never underflows.
inline bool get_var_bytes(
  const std::uint8_t * data, std::size_t size, std::size_t * offset,
  std::vector<std::uint8_t> * out)
{
  std::uint32_t len = 0;
  if (!get_u32(data, size, offset, &len)) {
    return false;
  }
  if (len > size - *offset) {
    return false;
  }
  out->resize(len);
  if (len > 0) {
    std::memcpy(out->data(), data + *offset, len);
  }
  *offset += len;
  return true;
}

inline void put_var_string(std::vector<std::uint8_t> & buf, const std::string & value)
{
  put_var_bytes(buf, reinterpret_cast<const std::uint8_t *>(value.data()), value.size());
}

inline bool get_var_string(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, std::string * out)
{
  std::uint32_t len = 0;
  if (!get_u32(data, size, offset, &len)) {
    return false;
  }
  if (len > size - *offset) {
    return false;
  }
  out->assign(reinterpret_cast<const char *>(data + *offset), len);
  *offset += len;
  return true;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Socket addressing.
// ---------------------------------------------------------------------------

// Replaces any character outside [A-Za-z0-9._-] with '_', so a composite MIG
// identifier such as "MIG-GPU-<uuid>/<gi>/<ci>" yields a valid single path
// component. Both daemon and proxy call this so they derive the same path.
inline std::string sanitize_for_path(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    const bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                      c == '.' || c == '_' || c == '-';
    out.push_back(safe ? c : '_');
  }
  return out;
}

// Derives the Unix domain socket path for the daemon managing the given GPU.
// The GPU UUID is discovered at runtime; the path is never configured, so it
// always matches the managed GPU.
inline std::string socket_path_for_gpu(const std::string & gpu_uuid)
{
  return std::string(kSocketDir) + "/gpu_shared_memory_daemon." + sanitize_for_path(gpu_uuid) +
         ".sock";
}

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
  detail::put_var_bytes(buf, slot.mem_handle.data(), slot.mem_handle.size());
  detail::put_var_bytes(buf, slot.data_ready_event.data(), slot.data_ready_event.size());
  detail::put_var_bytes(buf, slot.data_done_event.data(), slot.data_done_event.size());
}

inline bool deserialize_slot_descriptor(
  const std::uint8_t * data, std::size_t size, std::size_t * offset, SlotDescriptor * out)
{
  return detail::get_u32(data, size, offset, &out->slot_id) &&
         detail::get_u32(data, size, offset, &out->size_class_index) &&
         detail::get_u64(data, size, offset, &out->slot_size) &&
         detail::get_var_bytes(data, size, offset, &out->mem_handle) &&
         detail::get_var_bytes(data, size, offset, &out->data_ready_event) &&
         detail::get_var_bytes(data, size, offset, &out->data_done_event);
}

inline std::vector<std::uint8_t> serialize_list_response(const ListResponse & response)
{
  std::vector<std::uint8_t> buf;
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
  // Reject a slot count the remaining buffer cannot possibly hold before reserve(),
  // so a corrupt/truncated frame cannot trigger a huge speculative allocation.
  // Each slot occupies at least kMinSlotDescriptorWireSize bytes on the wire.
  if (count > (size - offset) / kMinSlotDescriptorWireSize) {
    return false;
  }
  out->slots.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    SlotDescriptor slot;
    if (!deserialize_slot_descriptor(data, size, &offset, &slot)) {
      return false;
    }
    out->slots.push_back(std::move(slot));
  }
  return true;
}

inline std::vector<std::uint8_t> serialize_handshake_response(const HandshakeResponse & response)
{
  std::vector<std::uint8_t> buf;
  detail::put_u32(buf, response.backend_type);
  detail::put_var_string(buf, response.gpu_uuid);
  return buf;
}

inline bool deserialize_handshake_response(
  const std::uint8_t * data, std::size_t size, HandshakeResponse * out)
{
  std::size_t offset = 0;
  return detail::get_u32(data, size, &offset, &out->backend_type) &&
         detail::get_var_string(data, size, &offset, &out->gpu_uuid);
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
