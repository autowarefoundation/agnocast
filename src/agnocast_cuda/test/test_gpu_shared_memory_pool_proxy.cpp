// Integration tests for GpuSharedMemoryPoolProxy: real socket to an in-process
// fake daemon, with a mock client backend so no GPU is needed.
#include "gpu_shared_memory_pool_proxy.hpp"
#include "proxy_test_helpers.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <vector>

namespace cuda = agnocast::cuda;
namespace gpud = agnocast::gpu_shared_memory_daemon;
using cuda::test::fake_device_ptr;
using cuda::test::FakeDaemon;
using cuda::test::make_fake_slots;
using cuda::test::MockClientBackend;

namespace
{

std::string temp_socket_path()
{
  return "/tmp/agnocast_proxy_test_" + std::to_string(::getpid()) + ".sock";
}

// A fast retry policy so tests exercising kNoFreeSlot don't sleep.
cuda::AllocRetryPolicy fast_retry(int attempts)
{
  return cuda::AllocRetryPolicy{attempts, std::chrono::microseconds{0}};
}

}  // namespace

TEST(GpuSharedMemoryPoolProxy, InitializeHandshakesAndImportsAllSlots)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(3);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());
  EXPECT_EQ(backend.import_calls, 3u);
}

TEST(GpuSharedMemoryPoolProxy, InitializeFailsOnBackendMismatch)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  daemon.backend_type = static_cast<std::uint32_t>(gpud::BackendType::kNvSciBuf);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;  // reports kCudaIpc
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  EXPECT_FALSE(proxy.initialize());
}

TEST(GpuSharedMemoryPoolProxy, InitializeFailsOnGpuUuidMismatch)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  daemon.gpu_uuid = "GPU-some-other-gpu";
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;  // local uuid GPU-mock-0001
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  EXPECT_FALSE(proxy.initialize());
}

TEST(GpuSharedMemoryPoolProxy, AllocateReturnsImportedPointerAndMapping)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(3);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  void * ptr = nullptr;
  ASSERT_TRUE(proxy.allocateMemory(&ptr, 1000));
  // The fake hands out slot 0 first; its imported pointer is deterministic.
  EXPECT_EQ(ptr, fake_device_ptr(0));

  std::uint32_t slot_id = 999;
  ASSERT_TRUE(proxy.getSlotIdFromDevicePtr(ptr, slot_id));
  EXPECT_EQ(slot_id, 0u);
}

TEST(GpuSharedMemoryPoolProxy, AllocateRetriesWhileNoFreeSlot)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(2);
  daemon.alloc_status_script = {
    gpud::Status::kNoFreeSlot, gpud::Status::kNoFreeSlot, gpud::Status::kOk};
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path, fast_retry(10));
  ASSERT_TRUE(proxy.initialize());

  void * ptr = nullptr;
  ASSERT_TRUE(proxy.allocateMemory(&ptr, 100));
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(daemon.alloc_requests.load(), 3);
}

TEST(GpuSharedMemoryPoolProxy, AllocateGivesUpAfterMaxAttempts)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  daemon.alloc_status_script = {
    gpud::Status::kNoFreeSlot, gpud::Status::kNoFreeSlot, gpud::Status::kNoFreeSlot};
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path, fast_retry(3));
  ASSERT_TRUE(proxy.initialize());

  void * ptr = nullptr;
  EXPECT_FALSE(proxy.allocateMemory(&ptr, 100));
  EXPECT_EQ(daemon.alloc_requests.load(), 3);
}

TEST(GpuSharedMemoryPoolProxy, AllocateFailsImmediatelyOnSizeTooLarge)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  daemon.alloc_status_script = {gpud::Status::kSizeTooLarge};
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path, fast_retry(10));
  ASSERT_TRUE(proxy.initialize());

  void * ptr = nullptr;
  EXPECT_FALSE(proxy.allocateMemory(&ptr, 1u << 30));
  EXPECT_EQ(daemon.alloc_requests.load(), 1);  // did not retry
}

TEST(GpuSharedMemoryPoolProxy, FreeSendsFreeRequestAndClearsMapping)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(3);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  void * ptr = nullptr;
  ASSERT_TRUE(proxy.allocateMemory(&ptr, 100));
  proxy.freeMemory(ptr);

  // The mapping is gone and the daemon saw the free for slot 0.
  std::uint32_t slot_id = 0;
  EXPECT_FALSE(proxy.getSlotIdFromDevicePtr(ptr, slot_id));
  EXPECT_EQ(daemon.free_requests.load(), 1);
  EXPECT_EQ(daemon.last_freed_slot_id.load(), 0);
}

TEST(GpuSharedMemoryPoolProxy, FreeIgnoresUnknownPointer)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  proxy.freeMemory(reinterpret_cast<void *>(0xdeadbeef));
  EXPECT_EQ(daemon.free_requests.load(), 0);
}

TEST(GpuSharedMemoryPoolProxy, FinalizeReleasesImportedSlots)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(4);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  {
    cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
    ASSERT_TRUE(proxy.initialize());
    EXPECT_EQ(backend.import_calls, 4u);
  }  // destructor -> finalize
  EXPECT_EQ(backend.release_calls, 4u);
}
