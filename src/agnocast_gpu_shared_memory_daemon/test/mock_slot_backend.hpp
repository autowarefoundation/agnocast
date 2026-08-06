// Shared test double for GpuSlotBackend, so the pool, request-handler, and server
// tests all exercise real bookkeeping without a GPU. Hands out deterministic fake
// resources and can be told to fail on the Nth create_slot().
#pragma once

#include "agnocast_gpu_shared_memory_daemon/pool_config.hpp"
#include "gpu_slot_backend.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace agnocast::gpu_shared_memory_daemon::test
{

class MockSlotBackend : public GpuSlotBackend
{
public:
  BackendType backend_type() const override { return BackendType::kCudaIpc; }

  bool initialize(std::string & gpu_uuid_out) override
  {
    initialize_calls++;
    gpu_uuid_out = "GPU-mock-0001";
    return initialize_result;
  }

  bool create_slot(std::size_t size, AllocatedSlotResources & out) override
  {
    if (
      fail_on_create_index >= 0 && create_calls == static_cast<std::size_t>(fail_on_create_index)) {
      ++create_calls;
      return false;
    }
    const auto tag = static_cast<std::uint8_t>(create_calls + 1);
    out = AllocatedSlotResources{};
    out.device_ptr = reinterpret_cast<void *>(static_cast<std::uintptr_t>(create_calls + 1));
    out.data_ready_event =
      reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x1000 + create_calls));
    out.data_done_event =
      reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x2000 + create_calls));
    out.mem_handle = {tag, static_cast<std::uint8_t>(size & 0xff)};
    out.data_ready_event_handle = {static_cast<std::uint8_t>(0xa0 + tag)};
    out.data_done_event_handle = {static_cast<std::uint8_t>(0xb0 + tag)};
    ++create_calls;
    return true;
  }

  void destroy_slot(AllocatedSlotResources & resources) override
  {
    if (resources.device_ptr != nullptr) {
      ++destroy_calls;
    }
    resources = AllocatedSlotResources{};
  }

  bool initialize_result = true;
  int fail_on_create_index = -1;  // -1 = never fail
  std::size_t initialize_calls = 0;
  std::size_t create_calls = 0;
  std::size_t destroy_calls = 0;
};

// Config: class 0 = 1024 B x 2, class 1 = 4096 B x 3 (5 slots total).
inline PoolConfig two_class_config()
{
  PoolConfig config;
  config.size_classes = {
    SizeClassConfig{1024u, 2u},
    SizeClassConfig{4096u, 3u},
  };
  return config;
}

}  // namespace agnocast::gpu_shared_memory_daemon::test
