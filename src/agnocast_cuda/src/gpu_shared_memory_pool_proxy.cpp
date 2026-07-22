#include "gpu_shared_memory_pool_proxy.hpp"

#include "agnocast_gpu_shared_memory_daemon/socket_io.hpp"
#include "cuda_ipc_client_backend.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>

namespace gpud = agnocast::gpu_shared_memory_daemon;

namespace agnocast::cuda
{

GpuSharedMemoryPoolProxy::GpuSharedMemoryPoolProxy(
  GpuClientBackend & backend, std::string socket_path, AllocRetryPolicy retry_policy)
: backend_(backend), socket_path_(std::move(socket_path)), retry_policy_(retry_policy)
{
}

GpuSharedMemoryPoolProxy::~GpuSharedMemoryPoolProxy()
{
  finalize();
}

GpuSharedMemoryPoolProxy * GpuSharedMemoryPoolProxy::getInstance()
{
  // Magic statics guarantee single, thread-safe construction/initialization.
  static CudaIpcClientBackend backend;
  static GpuSharedMemoryPoolProxy proxy(backend);
  static const bool ok = proxy.initialize();
  return ok ? &proxy : nullptr;
}

bool GpuSharedMemoryPoolProxy::initialize()
{
  std::lock_guard<std::mutex> io_lock(daemonIOMutex_);
  if (initialized_) {
    return true;
  }

  if (socket_path_.empty()) {
    std::string uuid;
    if (!backend_.local_gpu_uuid(uuid)) {
      std::fprintf(stderr, "[agnocast_cuda] proxy: could not determine local GPU UUID\n");
      return false;
    }
    socket_path_ = gpud::socket_path_for_gpu(uuid);
  }

  if (!connectSocket()) {
    return false;
  }
  if (!handshakeAndImport()) {
    closeSocket();
    return false;
  }

  initialized_ = true;
  return true;
}

void GpuSharedMemoryPoolProxy::finalize()
{
  std::lock_guard<std::mutex> io_lock(daemonIOMutex_);
  releaseImportedSlots();
  closeSocket();
  initialized_ = false;
}

bool GpuSharedMemoryPoolProxy::connectSocket()
{
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (socket_path_.size() >= sizeof(addr.sun_path)) {
    std::fprintf(stderr, "[agnocast_cuda] proxy: socket path too long: %s\n", socket_path_.c_str());
    return false;
  }

  socket_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::fprintf(stderr, "[agnocast_cuda] proxy: socket() failed: %s\n", std::strerror(errno));
    return false;
  }
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    std::fprintf(
      stderr, "[agnocast_cuda] proxy: connect(%s) failed: %s\n", socket_path_.c_str(),
      std::strerror(errno));
    closeSocket();
    return false;
  }
  return true;
}

void GpuSharedMemoryPoolProxy::closeSocket()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void GpuSharedMemoryPoolProxy::releaseImportedSlots()
{
  std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
  for (auto & entry : slots_) {
    backend_.release_slot(entry.second.imported);
  }
  slots_.clear();
  device_ptr_to_slot_id_.clear();
}

bool GpuSharedMemoryPoolProxy::handshakeAndImport()
{
  // --- Handshake: verify the daemon speaks our backend and manages our GPU. ---
  gpud::MessageHeader header;
  std::vector<std::uint8_t> payload;
  if (!gpud::write_message(socket_fd_, gpud::MessageType::kHandshakeRequest, {})) {
    return false;
  }
  if (!gpud::read_message(socket_fd_, header, payload)) {
    return false;
  }
  if (header.type != static_cast<std::uint32_t>(gpud::MessageType::kHandshakeResponse)) {
    return false;
  }
  gpud::HandshakeResponse handshake;
  if (!gpud::deserialize_handshake_response(payload.data(), payload.size(), &handshake)) {
    return false;
  }
  if (handshake.backend_type != static_cast<std::uint32_t>(backend_.backend_type())) {
    std::fprintf(
      stderr, "[agnocast_cuda] proxy: daemon backend %u does not match expected %u\n",
      handshake.backend_type, static_cast<std::uint32_t>(backend_.backend_type()));
    return false;
  }
  std::string local_uuid;
  if (!backend_.local_gpu_uuid(local_uuid)) {
    return false;
  }
  if (handshake.gpu_uuid != local_uuid) {
    std::fprintf(
      stderr,
      "[agnocast_cuda] proxy: daemon manages GPU '%s' but this process uses '%s'; refusing to "
      "share across GPUs\n",
      handshake.gpu_uuid.c_str(), local_uuid.c_str());
    return false;
  }

  // --- List and import every slot. ---
  if (!gpud::write_message(socket_fd_, gpud::MessageType::kListRequest, {})) {
    return false;
  }
  if (!gpud::read_message(socket_fd_, header, payload)) {
    return false;
  }
  if (header.type != static_cast<std::uint32_t>(gpud::MessageType::kListResponse)) {
    return false;
  }
  gpud::ListResponse list;
  if (!gpud::deserialize_list_response(payload.data(), payload.size(), &list)) {
    return false;
  }

  std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
  // Drop any slots imported by a previous connection before re-importing.
  for (auto & entry : slots_) {
    backend_.release_slot(entry.second.imported);
  }
  slots_.clear();
  device_ptr_to_slot_id_.clear();

  for (const auto & descriptor : list.slots) {
    ProxySlot slot;
    slot.descriptor = descriptor;
    if (!backend_.import_slot(descriptor, slot.imported)) {
      for (auto & entry : slots_) {
        backend_.release_slot(entry.second.imported);
      }
      slots_.clear();
      return false;
    }
    slots_.emplace(descriptor.slot_id, std::move(slot));
  }
  return true;
}

bool GpuSharedMemoryPoolProxy::transact(
  gpud::MessageType request_type, const std::vector<std::uint8_t> & request_payload,
  gpud::MessageType expected_response_type, gpud::MessageHeader & response_header,
  std::vector<std::uint8_t> & response_payload)
{
  auto attempt = [&]() -> bool {
    if (socket_fd_ < 0) {
      return false;
    }
    if (!gpud::write_message(socket_fd_, request_type, request_payload)) {
      return false;
    }
    if (!gpud::read_message(socket_fd_, response_header, response_payload)) {
      return false;
    }
    return response_header.type == static_cast<std::uint32_t>(expected_response_type);
  };

  if (attempt()) {
    return true;
  }

  // The connection broke (e.g. the daemon restarted). Reconnect and re-import the
  // now-fresh slot handles once, then retry the request.
  closeSocket();
  if (!connectSocket() || !handshakeAndImport()) {
    return false;
  }
  return attempt();
}

bool GpuSharedMemoryPoolProxy::allocateMemory(void ** device_ptr, size_t size)
{
  if (!initialized_ || device_ptr == nullptr) {
    return false;
  }

  gpud::AllocRequest request;
  request.size = size;
  request.non_blocking = 0;
  const auto request_payload = gpud::serialize_alloc_request(request);

  for (int attempt = 0; attempt < retry_policy_.max_attempts; ++attempt) {
    gpud::MessageHeader header;
    std::vector<std::uint8_t> response_payload;
    {
      std::lock_guard<std::mutex> io_lock(daemonIOMutex_);
      if (!transact(
            gpud::MessageType::kAllocRequest, request_payload, gpud::MessageType::kAllocResponse,
            header, response_payload)) {
        return false;
      }
    }

    gpud::AllocResponse response;
    if (!gpud::deserialize_alloc_response(
          response_payload.data(), response_payload.size(), &response)) {
      return false;
    }

    const auto status = static_cast<gpud::Status>(response.status);
    if (status == gpud::Status::kOk) {
      std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
      const auto it = slots_.find(response.slot_id);
      if (it == slots_.end()) {
        return false;  // slot vanished (e.g. across a reconnect); treat as failure
      }
      void * ptr = it->second.imported.device_ptr;
      device_ptr_to_slot_id_[ptr] = response.slot_id;
      *device_ptr = ptr;
      return true;
    }
    if (status == gpud::Status::kSizeTooLarge) {
      return false;
    }
    if (status == gpud::Status::kNoFreeSlot) {
      // Daemon never blocks; wait briefly and retry (without holding any lock, so
      // other threads can free slots meanwhile).
      std::this_thread::sleep_for(retry_policy_.delay);
      continue;
    }
    return false;  // kInvalidSlot / kInternalError
  }
  return false;
}

void GpuSharedMemoryPoolProxy::freeMemory(void * device_ptr)
{
  std::uint32_t slot_id = 0;
  {
    std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
    const auto it = device_ptr_to_slot_id_.find(device_ptr);
    if (it == device_ptr_to_slot_id_.end()) {
      return;  // not a pooled pointer handed out by this proxy
    }
    slot_id = it->second;
    device_ptr_to_slot_id_.erase(it);
  }

  gpud::FreeRequest request;
  request.slot_id = slot_id;
  const auto request_payload = gpud::serialize_free_request(request);

  gpud::MessageHeader header;
  std::vector<std::uint8_t> response_payload;
  std::lock_guard<std::mutex> io_lock(daemonIOMutex_);
  transact(
    gpud::MessageType::kFreeRequest, request_payload, gpud::MessageType::kFreeResponse, header,
    response_payload);
}

bool GpuSharedMemoryPoolProxy::getSlotIdFromDevicePtr(void * device_ptr, std::uint32_t & slot_id)
{
  std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
  const auto it = device_ptr_to_slot_id_.find(device_ptr);
  if (it == device_ptr_to_slot_id_.end()) {
    return false;
  }
  slot_id = it->second;
  return true;
}

bool GpuSharedMemoryPoolProxy::getDevicePtrFromSlotId(std::uint32_t slot_id, void *& device_ptr)
{
  std::lock_guard<std::mutex> slot_lock(slotManagementMutex_);
  const auto it = slots_.find(slot_id);
  if (it == slots_.end()) {
    return false;
  }
  device_ptr = it->second.imported.device_ptr;
  return true;
}

}  // namespace agnocast::cuda
