// End-to-end check of the cross-process path: a publisher allocates a region and
// hands its liveness reference to the kernel module, which installs a descriptor
// for it into a separate process, which maps it and reads what the publisher
// wrote through the values the message carries alone.
//
// This is the only coverage for descriptor installation, which depends on the
// calling process's file table and so cannot be reached from KUnit.
//
// Requires the agnocast kernel module and a GPU; see the requires_kernel_module
// label in CMakeLists.txt.

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"
#include "agnocast/internal/gpu_slot_pool.hpp"

#include <cuda_runtime.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <vector>

// The AGNOCAST_*_CMD macros name their argument types with unqualified
// elaborated-type-specifiers, so the types must be reachable by unqualified
// lookup or each use declares a new incomplete type.
using agnocast::ioctl_add_process_args;
using agnocast::ioctl_add_publisher_args;

namespace
{

constexpr const char * kTopic = "/gpu_region_e2e";
constexpr const char * kNode = "/gpu_region_e2e_node";
constexpr uint32_t kSlotSize = 1U << 20;
constexpr uint32_t kSlotCount = 4;
constexpr size_t kPayload = 4096;

// Stands in for the part of a message that reaches the subscriber through
// Agnocast's host shared memory. It carries no device address, because none
// would be meaningful in another process.
struct Message
{
  agnocast::topic_local_id_t publisher_id;
  uint32_t region_id;
  uint32_t slot_index;
  uint64_t count;
};

// Each process registers with the kernel module itself; no CUDA call happens
// before the fork, because CUDA does not support forking an initialized context.
bool open_device_and_register()
{
  agnocast::agnocast_fd = ::open("/dev/agnocast", O_RDWR);
  if (agnocast::agnocast_fd < 0) {
    std::fprintf(stderr, "open(/dev/agnocast) failed: %s\n", std::strerror(errno));
    return false;
  }

  ioctl_add_process_args args = {};
  args.domain_id = 0;
  if (ioctl(agnocast::agnocast_fd, AGNOCAST_ADD_PROCESS_CMD, &args) < 0) {
    std::fprintf(stderr, "ADD_PROCESS failed: %s\n", std::strerror(errno));
    return false;
  }
  return true;
}

int run_publisher(int notify_fd)
{
  if (!open_device_and_register()) return 1;

  ioctl_add_publisher_args pub_args = {};
  pub_args.topic_name = {kTopic, std::strlen(kTopic)};
  pub_args.node_name = {kNode, std::strlen(kNode)};
  pub_args.qos_depth = kSlotCount;
  pub_args.qos_is_transient_local = false;
  pub_args.is_bridge = false;
  if (ioctl(agnocast::agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, &pub_args) < 0) {
    std::fprintf(stderr, "ADD_PUBLISHER failed: %s\n", std::strerror(errno));
    return 1;
  }
  const agnocast::topic_local_id_t publisher_id = pub_args.ret_id;

  auto pool = agnocast::internal::GpuSlotPool::create(kTopic, publisher_id, kSlotSize, kSlotCount);
  if (pool == nullptr) {
    std::fprintf(stderr, "GpuSlotPool::create failed\n");
    return 1;
  }
  if (pool->available() != kSlotCount) return 1;

  uint32_t rejected = 0;
  if (pool->acquire(static_cast<uint64_t>(kSlotSize) + 1, rejected)) {
    std::fprintf(stderr, "pool handed out a slot too small for the payload\n");
    return 1;
  }

  bool subscriber_ok = false;
  {
    uint32_t slot_index = 0;
    if (!pool->acquire(kPayload, slot_index)) {
      std::fprintf(stderr, "pool exhausted with every slot free\n");
      return 1;
    }
    if (pool->available() != kSlotCount - 1) return 1;

    const agnocast::internal::gpu_array<uint8_t> data(
      pool->region_id(), slot_index, kPayload, publisher_id);

    std::vector<uint8_t> pattern(kPayload);
    for (size_t i = 0; i < kPayload; i++) pattern[i] = static_cast<uint8_t>(i % 251);

    const cudaError_t copy =
      cudaMemcpy(data.get(), pattern.data(), kPayload, cudaMemcpyHostToDevice);
    if (copy != cudaSuccess) {
      std::fprintf(stderr, "publisher cudaMemcpy failed: %s\n", cudaGetErrorString(copy));
      return 1;
    }
    cudaDeviceSynchronize();

    const Message msg = {publisher_id, data.region_id(), data.slot_index(), data.size()};
    if (write(notify_fd, &msg, sizeof(msg)) != sizeof(msg)) return 1;

    int status = 0;
    wait(&status);
    subscriber_ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
  }

  // Destroying the handle is what returns the slot. In a real publisher the same
  // path runs when the kmod releases the message entry.
  if (pool->available() != kSlotCount) {
    std::fprintf(stderr, "slot was not returned to the pool\n");
    return 1;
  }

  return subscriber_ok ? 0 : 1;
}

int run_subscriber(int notify_fd)
{
  Message msg = {};
  if (read(notify_fd, &msg, sizeof(msg)) != sizeof(msg)) return 1;

  if (!open_device_and_register()) return 1;

  // Mapping a publisher's region is a one-off on first receipt, as it would be
  // for a previously unseen publisher on a live topic.
  const agnocast::internal::GpuRegionRef ref{
    kTopic, msg.publisher_id, /*subscriber_id=*/0, msg.region_id};
  if (!agnocast::internal::GpuRegionRegistry::instance().ensure_mapped(ref)) {
    std::fprintf(stderr, "ensure_mapped failed\n");
    return 1;
  }

  // Resolves to a device address from the values the message carries plus this
  // process's own region table -- no knowledge of topic or publisher.
  const agnocast::internal::gpu_array<uint8_t> data(
    msg.region_id, msg.slot_index, msg.count, msg.publisher_id);
  if (data.get() == nullptr) {
    std::fprintf(stderr, "gpu_array did not resolve\n");
    return 1;
  }

  std::vector<uint8_t> readback(kPayload, 0);
  if (cudaMemcpy(readback.data(), data.get(), kPayload, cudaMemcpyDeviceToHost) != cudaSuccess) {
    std::fprintf(stderr, "subscriber cudaMemcpy failed\n");
    return 1;
  }

  size_t mismatches = 0;
  for (size_t i = 0; i < kPayload; i++) {
    if (readback[i] != static_cast<uint8_t>(i % 251)) mismatches++;
  }
  if (mismatches != 0) {
    std::fprintf(stderr, "%zu byte mismatches: the mappings do not alias\n", mismatches);
    return 1;
  }

  // A slot the publisher never wrote must differ, otherwise the comparison above
  // would pass against any mapping at all.
  const agnocast::internal::gpu_array<uint8_t> neighbour(
    msg.region_id, msg.slot_index + 1, msg.count, msg.publisher_id);
  std::vector<uint8_t> other(kPayload, 0);
  if (cudaMemcpy(other.data(), neighbour.get(), kPayload, cudaMemcpyDeviceToHost) != cudaSuccess) {
    return 1;
  }
  if (std::memcmp(other.data(), readback.data(), kPayload) == 0) {
    std::fprintf(stderr, "an unwritten slot matched the payload; the offset is not honoured\n");
    return 1;
  }

  std::printf(
    "subscriber: resolved region %u slot %u from the message alone; %zu bytes match what "
    "another process wrote; slot %u differs as expected\n",
    msg.region_id, msg.slot_index, kPayload, msg.slot_index + 1);
  std::fflush(stdout);
  return 0;
}

}  // namespace

int main()
{
  int fds[2];
  if (pipe(fds) != 0) return 1;

  const pid_t pid = fork();
  if (pid < 0) return 1;

  if (pid == 0) {
    close(fds[1]);
    const int rc = run_subscriber(fds[0]);
    std::fflush(nullptr);
    _exit(rc);
  }

  close(fds[0]);
  const int rc = run_publisher(fds[1]);
  std::printf("%s\n", rc == 0 ? "PASS" : "FAIL");
  return rc;
}
