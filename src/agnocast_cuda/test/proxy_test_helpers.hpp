// Test helpers for the GpuSharedMemoryPoolProxy: a mock client backend (imports
// deterministic fake resources, no GPU) and an in-process fake daemon that speaks
// the real wire protocol over a real Unix domain socket.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"
#include "agnocast_gpu_shared_memory_daemon/socket_io.hpp"
#include "gpu_client_backend.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace agnocast::cuda::test
{

namespace gpud = agnocast::gpu_shared_memory_daemon;

// Deterministic per-slot device pointer, so tests can predict what allocate()
// returns for a given slot id.
inline void * fake_device_ptr(std::uint32_t slot_id)
{
  return reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x100000u + slot_id));
}

class MockClientBackend : public GpuClientBackend
{
public:
  gpud::BackendType backend_type() const override { return backend_type_; }

  bool local_gpu_uuid(std::string & uuid_out) override
  {
    uuid_out = gpu_uuid_;
    return uuid_result_;
  }

  bool import_slot(const gpud::SlotDescriptor & descriptor, ImportedSlot & out) override
  {
    ++import_calls;
    if (!import_result) {
      return false;
    }
    out = ImportedSlot{};
    out.device_ptr = fake_device_ptr(descriptor.slot_id);
    out.data_ready_event = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x1000));
    out.data_done_event = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x2000));
    return true;
  }

  void release_slot(ImportedSlot & imported) override
  {
    if (imported.device_ptr != nullptr) {
      ++release_calls;
    }
    imported = ImportedSlot{};
  }

  gpud::BackendType backend_type_ = gpud::BackendType::kCudaIpc;
  std::string gpu_uuid_ = "GPU-mock-0001";
  bool uuid_result_ = true;
  bool import_result = true;
  std::size_t import_calls = 0;
  std::size_t release_calls = 0;
};

// Builds `count` fake slot descriptors with 64-byte handle blobs.
inline std::vector<gpud::SlotDescriptor> make_fake_slots(std::uint32_t count)
{
  std::vector<gpud::SlotDescriptor> slots;
  for (std::uint32_t i = 0; i < count; ++i) {
    gpud::SlotDescriptor slot;
    slot.slot_id = i;
    slot.size_class_index = 0;
    slot.slot_size = 4096;
    slot.mem_handle.assign(64, static_cast<std::uint8_t>(i + 1));
    slot.data_ready_event.assign(64, static_cast<std::uint8_t>(i + 65));
    slot.data_done_event.assign(64, static_cast<std::uint8_t>(i + 129));
    slots.push_back(std::move(slot));
  }
  return slots;
}

// Minimal daemon stand-in: binds a socket, serves one connection at a time with
// canned handshake/list responses and a scripted sequence of alloc statuses.
class FakeDaemon
{
public:
  explicit FakeDaemon(std::string socket_path) : socket_path_(std::move(socket_path)) {}

  ~FakeDaemon() { stop(); }

  bool start()
  {
    ::unlink(socket_path_.c_str());
    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      return false;
    }
    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
      return false;
    }
    if (::listen(listen_fd_, 4) != 0) {
      return false;
    }
    thread_ = std::thread([this] { serve(); });
    return true;
  }

  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    ::unlink(socket_path_.c_str());
  }

  // Config (set before start()).
  std::uint32_t backend_type = static_cast<std::uint32_t>(gpud::BackendType::kCudaIpc);
  std::string gpu_uuid = "GPU-mock-0001";
  std::vector<gpud::SlotDescriptor> slots;
  // Statuses returned for successive alloc requests; once exhausted, kOk is used.
  std::vector<gpud::Status> alloc_status_script;

  // Observed activity.
  std::atomic<int> alloc_requests{0};
  std::atomic<int> free_requests{0};
  std::atomic<std::int64_t> last_freed_slot_id{-1};

private:
  void serve()
  {
    while (running_) {
      if (gpud::wait_readable(listen_fd_, 100) <= 0) {
        continue;
      }
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) {
        continue;
      }
      serve_connection(fd);
      ::close(fd);
    }
  }

  void serve_connection(int fd)
  {
    std::uint32_t next_alloc_slot = 0;
    while (running_) {
      if (gpud::wait_readable(fd, 100) <= 0) {
        continue;
      }
      gpud::MessageHeader header;
      std::vector<std::uint8_t> payload;
      if (!gpud::read_message(fd, header, payload)) {
        return;  // client closed
      }

      switch (static_cast<gpud::MessageType>(header.type)) {
        case gpud::MessageType::kHandshakeRequest: {
          gpud::HandshakeResponse response;
          response.backend_type = backend_type;
          response.gpu_uuid = gpu_uuid;
          gpud::write_message(
            fd, gpud::MessageType::kHandshakeResponse,
            gpud::serialize_handshake_response(response));
          break;
        }
        case gpud::MessageType::kListRequest: {
          gpud::ListResponse response;
          response.slots = slots;
          gpud::write_message(
            fd, gpud::MessageType::kListResponse, gpud::serialize_list_response(response));
          break;
        }
        case gpud::MessageType::kAllocRequest: {
          const int n = alloc_requests.fetch_add(1);
          gpud::AllocResponse response;
          gpud::Status status = gpud::Status::kOk;
          if (n < static_cast<int>(alloc_status_script.size())) {
            status = alloc_status_script[static_cast<std::size_t>(n)];
          }
          response.status = static_cast<std::uint32_t>(status);
          if (status == gpud::Status::kOk && !slots.empty()) {
            response.slot_id = slots[next_alloc_slot % slots.size()].slot_id;
            ++next_alloc_slot;
          }
          gpud::write_message(
            fd, gpud::MessageType::kAllocResponse, gpud::serialize_alloc_response(response));
          break;
        }
        case gpud::MessageType::kFreeRequest: {
          gpud::FreeRequest request;
          gpud::deserialize_free_request(payload.data(), payload.size(), &request);
          ++free_requests;
          last_freed_slot_id = static_cast<std::int64_t>(request.slot_id);
          gpud::FreeResponse response;
          response.status = static_cast<std::uint32_t>(gpud::Status::kOk);
          gpud::write_message(
            fd, gpud::MessageType::kFreeResponse, gpud::serialize_free_response(response));
          break;
        }
        default:
          return;  // unexpected: close
      }
    }
  }

  std::string socket_path_;
  int listen_fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace agnocast::cuda::test
