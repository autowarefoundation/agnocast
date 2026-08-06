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

TEST(GpuSharedMemoryPoolProxy, RecordAndWaitDataReadyDelegateToBackend)
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

  auto * publisher_stream = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0xabc000));
  auto * subscriber_stream = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0xdef000));

  // Publisher records the ready event for the pooled pointer, on its own stream.
  ASSERT_TRUE(proxy.recordDataReady(ptr, 2 /* kExplicit */, publisher_stream));
  EXPECT_EQ(backend.record_ready_calls, 1u);
  EXPECT_EQ(backend.last_record_ready_ptr, ptr);
  EXPECT_EQ(backend.last_record_ready_stream_kind, 2);
  EXPECT_EQ(backend.last_record_ready_stream, publisher_stream);

  // Subscriber waits on the ready event by slot id (slot 0 -> same imported ptr),
  // on a different stream.
  ASSERT_TRUE(proxy.waitDataReady(0, 2 /* kExplicit */, subscriber_stream));
  EXPECT_EQ(backend.wait_ready_calls, 1u);
  EXPECT_EQ(backend.last_wait_ready_ptr, ptr);
  EXPECT_EQ(backend.last_wait_ready_stream_kind, 2);
  EXPECT_EQ(backend.last_wait_ready_stream, subscriber_stream);
}

TEST(GpuSharedMemoryPoolProxy, ReadDoneMarkersAreRecycledNotRecreated)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  auto * stream = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0xabc000));

  void * first = nullptr;
  ASSERT_TRUE(proxy.recordReadDone(2, stream, &first));
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(backend.create_marker_calls, 1u);
  EXPECT_EQ(backend.record_marker_calls, 1u);
  EXPECT_EQ(backend.last_marker_stream, stream);

  // Completing a marker returns it to the free list, so the next record reuses it.
  backend.query_marker_state = 1;
  EXPECT_EQ(proxy.queryReadDone(first), 1);

  void * second = nullptr;
  ASSERT_TRUE(proxy.recordReadDone(2, stream, &second));
  EXPECT_EQ(second, first);
  EXPECT_EQ(backend.create_marker_calls, 1u);  // reused, not recreated

  // A pending marker stays out of the free list, so a concurrent record creates one.
  backend.query_marker_state = 0;
  EXPECT_EQ(proxy.queryReadDone(second), 0);
  void * third = nullptr;
  ASSERT_TRUE(proxy.recordReadDone(2, stream, &third));
  EXPECT_NE(third, second);
  EXPECT_EQ(backend.create_marker_calls, 2u);

  // finalize() destroys everything that made it back to the free list.
  backend.query_marker_state = 1;
  EXPECT_EQ(proxy.queryReadDone(second), 1);
  EXPECT_EQ(proxy.queryReadDone(third), 1);
  proxy.finalize();
  EXPECT_EQ(backend.destroy_marker_calls, 2u);
}

TEST(GpuSharedMemoryPoolProxy, ReadDoneMarkerErrorReportsCompleteAndRecycles)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  void * token = nullptr;
  ASSERT_TRUE(proxy.recordReadDone(0, nullptr, &token));

  // A backend error must not pin the pool slot forever: report complete so the caller
  // releases the message reference.
  backend.query_marker_state = -1;
  EXPECT_EQ(proxy.queryReadDone(token), 1);
  // ...but the failed marker must be destroyed, not recycled. Putting it back in
  // circulation would make every later message report "complete" on its first poll under
  // a sticky context error, silently disabling the done edge for the rest of the run.
  EXPECT_EQ(backend.destroy_marker_calls, 1u);

  backend.query_marker_state = 1;
  void * fresh = nullptr;
  ASSERT_TRUE(proxy.recordReadDone(0, nullptr, &fresh));
  EXPECT_NE(fresh, token);
  EXPECT_EQ(backend.create_marker_calls, 2u);
}

TEST(GpuSharedMemoryPoolProxy, RecordReadDoneFailsWhenBackendCannotCreateMarker)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  backend.create_marker_result = false;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  void * token = nullptr;
  EXPECT_FALSE(proxy.recordReadDone(0, nullptr, &token));
}

TEST(GpuSharedMemoryPoolProxy, DefaultStreamDetectionDelegatesToBackend)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  EXPECT_TRUE(proxy.isDefaultStream(0 /* kLegacyDefault */, nullptr));
  EXPECT_TRUE(proxy.isDefaultStream(1 /* kPerThreadDefault */, nullptr));
  EXPECT_TRUE(proxy.isDefaultStream(2 /* kExplicit */, reinterpret_cast<void *>(0x2)));
  EXPECT_FALSE(proxy.isDefaultStream(2 /* kExplicit */, reinterpret_cast<void *>(0xabc000)));
}

TEST(GpuSharedMemoryPoolProxy, RecordAndWaitRejectUnknownSlots)
{
  const auto path = temp_socket_path();
  FakeDaemon daemon(path);
  daemon.slots = make_fake_slots(1);
  ASSERT_TRUE(daemon.start());

  MockClientBackend backend;
  cuda::GpuSharedMemoryPoolProxy proxy(backend, path);
  ASSERT_TRUE(proxy.initialize());

  EXPECT_FALSE(proxy.recordDataReady(reinterpret_cast<void *>(0xdeadbeef), 0, nullptr));
  EXPECT_FALSE(proxy.waitDataReady(999, 0, nullptr));
  EXPECT_EQ(backend.record_ready_calls, 0u);
  EXPECT_EQ(backend.wait_ready_calls, 0u);
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
