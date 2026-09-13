#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_backend.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <sys/ioctl.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace agnocast::internal
{

namespace
{

struct RegionEntry
{
  MappedGpuRegion region;
  // Separates a region this process created, whose lifetime its slot pool owns,
  // from one imported from a peer, which is released once the kmod no longer
  // holds it and nothing here still refers to it.
  bool imported = false;
};

struct Registry
{
  // Shared rather than exclusive because resolving a slot is on the message
  // path, while mapping and releasing a region are cold. Guards both maps.
  std::shared_mutex rwlock;
  std::unordered_map<uint32_t, RegionEntry> regions;
  // How many received message handles still refer to a region, kept
  // independently of whether it is mapped: a handle is created before the frame
  // that maps its region. This is what makes releasing an imported region safe,
  // since the kmod's entry accounting is dropped by ~SubscriptionBase while
  // userspace may still hold handles.
  std::unordered_map<uint32_t, uint32_t> refs;
};

// Leaked deliberately: a message destructor can reach this after static
// destruction would have run.
Registry & registry()
{
  static auto * state = new Registry();  // NOLINT(cppcoreguidelines-owning-memory)
  return *state;
}

// Whether a handle in this process can still resolve a payload in this region.
// Caller holds the lock, in either mode.
bool is_referenced(const Registry & state, const uint32_t region_id)
{
  const auto it = state.refs.find(region_id);
  return it != state.refs.end() && it->second != 0;
}

// Which of `region_ids` the kmod still holds. A failed call reports everything
// as still held: releasing a live region on a transient error would leave a
// later message unable to resolve, which is far worse than keeping a mapping
// until the next sweep.
void query_regions_still_held(const std::vector<uint32_t> & region_ids, std::vector<bool> & held)
{
  held.assign(region_ids.size(), true);
  if (agnocast_fd < 0) return;

  for (size_t base = 0; base < region_ids.size(); base += MAX_GPU_REGION_QUERY_NUM) {
    const size_t batch = std::min<size_t>(MAX_GPU_REGION_QUERY_NUM, region_ids.size() - base);

    union ioctl_gpu_region_exists_args args = {};
    args.region_ids_addr = reinterpret_cast<uint64_t>(region_ids.data() + base);
    args.region_num = static_cast<uint32_t>(batch);
    if (ioctl(agnocast_fd, AGNOCAST_GPU_REGION_EXISTS_CMD, &args) < 0) {
      RCLCPP_ERROR(
        logger, "AGNOCAST_GPU_REGION_EXISTS_CMD failed: %s; keeping every mapping for now",
        strerror(errno));
      return;
    }
    for (size_t i = 0; i < batch; i++) {
      held[base + i] = (args.ret_exists_bitmap & (1ULL << i)) != 0;
    }
  }
}

uint32_t create_via_kmod(
  GpuMemoryBackend & backend, const std::string_view topic_name,
  const topic_local_id_t publisher_id, MappedGpuRegion & region)
{
  const std::optional<GpuRegionExport> exported = backend.export_region(region);
  if (!exported) return 0;

  // Unreachable while VMM is the only mechanism, but silently registering a
  // region whose handle the kmod would read as the wrong kind is not the way to
  // find out that changed.
  if (exported->backend != GpuMemoryBackendType::Vmm) {
    RCLCPP_ERROR(
      logger, "the GPU backend exported region of topic '%.*s' as mechanism %u, which is not VMM",
      static_cast<int>(topic_name.size()), topic_name.data(),
      static_cast<uint32_t>(exported->backend));
    return 0;
  }

  union ioctl_add_gpu_region_args args = {};
  args.topic_name = {topic_name.data(), topic_name.size()};
  args.publisher_id = publisher_id;
  args.backend_type = static_cast<uint32_t>(exported->backend);
  args.slot_size = exported->geometry.slot_size;
  args.slot_count = exported->geometry.slot_count;
  args.mapped_size = exported->geometry.mapped_size;
  std::memcpy(args.device_uuid, exported->geometry.device_uuid.data(), GPU_DEVICE_UUID_SIZE);
  args.handle_fd = exported->handle.get();

  // The export still owns the handle: the kmod takes its own reference on
  // success, so releasing ours when `exported` dies is correct either way.
  if (ioctl(agnocast_fd, AGNOCAST_ADD_GPU_REGION_CMD, &args) < 0) {
    RCLCPP_ERROR(
      logger, "AGNOCAST_ADD_GPU_REGION_CMD failed for topic '%.*s': %s",
      static_cast<int>(topic_name.size()), topic_name.data(), strerror(errno));
    return 0;
  }
  return args.ret_region_id;
}

MappedGpuRegion import_via_kmod(
  GpuMemoryBackend & backend, const GpuRegionRef & ref, uint32_t & out_region_id)
{
  union ioctl_get_gpu_region_args args = {};
  args.topic_name = {ref.topic_name.data(), ref.topic_name.size()};
  args.publisher_id = ref.publisher_id;
  args.subscriber_id = ref.subscriber_id;
  args.region_id = ref.region_id;

  if (ioctl(agnocast_fd, AGNOCAST_GET_GPU_REGION_CMD, &args) < 0) {
    RCLCPP_ERROR(
      logger, "AGNOCAST_GET_GPU_REGION_CMD failed for topic '%.*s': %s",
      static_cast<int>(ref.topic_name.size()), ref.topic_name.data(), strerror(errno));
    return MappedGpuRegion{};
  }

  GpuRegionExport exported;
  exported.backend = static_cast<GpuMemoryBackendType>(args.ret_backend_type);
  exported.geometry.slot_size = args.ret_slot_size;
  exported.geometry.slot_count = args.ret_slot_count;
  exported.geometry.mapped_size = args.ret_mapped_size;
  std::memcpy(exported.geometry.device_uuid.data(), args.ret_device_uuid, GPU_DEVICE_UUID_SIZE);
  // The kmod installed the descriptor in this process, so it is ours from here.
  exported.handle = UniqueFd(args.ret_handle_fd);

  out_region_id = args.ret_region_id;
  return backend.import_region(exported);
}

// Imported regions the kmod no longer holds and nothing here refers to.
//
// Three phases, because the middle one is a syscall and a thread resolving a
// slot must not queue behind it. The reference test is repeated under the final
// lock: the kmod's answer was obtained without it, and a message naming the
// region may have arrived since.
//
// Returned rather than destroyed so the caller can unmap outside the lock too --
// releasing a region synchronizes the whole device.
std::vector<RegionEntry> collect_unreachable_regions()
{
  Registry & state = registry();

  std::vector<uint32_t> candidates;
  {
    const std::shared_lock<std::shared_mutex> lock(state.rwlock);
    for (const auto & [region_id, entry] : state.regions) {
      if (entry.imported && !is_referenced(state, region_id)) candidates.push_back(region_id);
    }
  }
  if (candidates.empty()) return {};

  std::vector<bool> held;
  query_regions_still_held(candidates, held);

  std::vector<RegionEntry> released;
  {
    const std::lock_guard<std::shared_mutex> lock(state.rwlock);
    for (size_t i = 0; i < candidates.size(); i++) {
      const uint32_t region_id = candidates[i];
      if (held[i] || is_referenced(state, region_id)) continue;
      const auto it = state.regions.find(region_id);
      if (it == state.regions.end()) continue;
      RCLCPP_DEBUG(
        logger, "releasing the mapping of GPU region %u: the kernel module no longer holds it",
        region_id);
      released.push_back(std::move(it->second));
      state.regions.erase(it);
    }
  }
  return released;
}

void adjust_region_refs(const uint32_t region_id, const bool take)
{
  if (region_id == 0) return;

  // The map node, and any rehash of the bucket array, are this library's
  // bookkeeping rather than part of a message. A subscriber can be inside an
  // open borrow window when it takes a handle, and without this they would be
  // served from the mempool and stay resident in the segment every peer maps.
  const SuspendedBorrowWindow suspended;

  Registry & state = registry();
  const std::lock_guard<std::shared_mutex> lock(state.rwlock);
  if (take) {
    state.refs[region_id]++;
    return;
  }
  const auto it = state.refs.find(region_id);
  if (it == state.refs.end()) return;
  if (--it->second == 0) state.refs.erase(it);
}

}  // namespace

bool gpu_region_is_mapped(const uint32_t region_id)
{
  Registry & state = registry();
  const std::shared_lock<std::shared_mutex> lock(state.rwlock);
  return state.regions.count(region_id) != 0;
}

uint32_t create_gpu_region(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint32_t slot_size,
  const uint32_t slot_count)
{
  // Reached before the borrow window opens in the ordinary case, but a publisher
  // creating its first region while another borrow is outstanding would
  // otherwise leave the backend's one-time initialization in the mempool.
  const SuspendedBorrowWindow suspended;

  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return 0;

  MappedGpuRegion region = backend->create_region(slot_size, slot_count);
  if (!region.valid()) return 0;

  const uint32_t region_id = create_via_kmod(*backend, topic_name, publisher_id, region);
  if (region_id == 0) return 0;

  Registry & state = registry();
  const std::lock_guard<std::shared_mutex> lock(state.rwlock);
  state.regions.insert_or_assign(region_id, RegionEntry{std::move(region), false});
  return region_id;
}

bool ensure_gpu_region_mapped(const GpuRegionRef & ref)
{
  if (ref.region_id != 0 && gpu_region_is_mapped(ref.region_id)) return true;

  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return false;

  uint32_t region_id = 0;
  MappedGpuRegion region = import_via_kmod(*backend, ref, region_id);
  if (!region.valid()) return false;
  RegionEntry entry{std::move(region), true};

  // Swept before the insertion, so the region just imported cannot be swept out
  // from under the caller about to resolve against it.
  // cppcheck-suppress variableScope ; must outlive the lock scope below
  std::vector<RegionEntry> released = collect_unreachable_regions();
  {
    Registry & state = registry();
    const std::lock_guard<std::shared_mutex> lock(state.rwlock);
    // Presence is tested before the insert rather than after, because emplace
    // may consume the argument even when it does not insert. A concurrent
    // importer may have won the race, in which case the mapping in use is kept
    // and ours joins `released` -- to be unmapped outside the lock, since
    // releasing a region synchronizes the whole device.
    if (state.regions.count(region_id) == 0) {
      state.regions.emplace(region_id, std::move(entry));
    } else {
      released.push_back(std::move(entry));
    }
  }
  return true;
}

void destroy_gpu_region(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint32_t region_id)
{
  if (region_id == 0) return;

  // The kmod first: once it has dropped its reference no further importer can
  // reach the region, so unmapping afterwards cannot race an import. A failure
  // is reported and then ignored, since the mapping is still ours to release.
  struct ioctl_remove_gpu_region_args args = {};
  args.topic_name = {topic_name.data(), topic_name.size()};
  args.publisher_id = publisher_id;
  args.region_id = region_id;
  if (agnocast_fd >= 0 && ioctl(agnocast_fd, AGNOCAST_REMOVE_GPU_REGION_CMD, &args) < 0) {
    RCLCPP_ERROR(
      logger, "AGNOCAST_REMOVE_GPU_REGION_CMD failed for topic '%.*s' region %u: %s",
      static_cast<int>(topic_name.size()), topic_name.data(), region_id, strerror(errno));
  }

  unmap_gpu_region(region_id);
}

void unmap_gpu_region(const uint32_t region_id)
{
  if (region_id == 0) return;

  // Deliberately not gated on a reference count, unlike the sweep: the callers
  // are a publisher retiring a region it has proven idle, and publisher
  // teardown, where waiting for a handle to be dropped would pin this mapping
  // and its share of device memory for the life of the process. A handle the
  // owning process still holds then resolves to nullptr rather than to unmapped
  // memory.
  //
  // As in create_gpu_region(): releasing a mapping calls into the driver, which
  // allocates host memory of its own.
  const SuspendedBorrowWindow suspended;

  // Held past the lock on purpose: releasing a region synchronizes the whole
  // device, and a thread resolving a slot of another region should not wait on
  // that. The destructor at the end of this scope is what unmaps.
  RegionEntry released;
  {
    Registry & state = registry();
    const std::lock_guard<std::shared_mutex> lock(state.rwlock);
    const auto it = state.regions.find(region_id);
    if (it == state.regions.end()) return;
    // cppcheck-suppress unreadVariable ; holds the region so it unmaps after the lock
    released = std::move(it->second);
    state.regions.erase(it);
  }
}

// The three below are noexcept because their callers are: a message destructor,
// and an accessor on the payload of a message being read. Locking, the map
// insert and logging can all throw -- logging allocates, and inside the borrow
// window that is the mempool. A function-try-block, as on UniqueFd::reset and
// VmmBackend::release_region, so a throw costs the bookkeeping rather than the
// process.

bool ref_gpu_region(const uint32_t region_id) noexcept
try {
  adjust_region_refs(region_id, true);
  return true;
} catch (...) {
  // Losing the reference means the region may be unmapped while this handle
  // lives, so it is worth a line -- but only if saying so cannot throw again.
  std::fprintf(
    stderr, "[agnocast] failed to reference GPU region %u; its mapping may be released early\n",
    region_id);
  return false;
}

void unref_gpu_region(const uint32_t region_id) noexcept
try {
  adjust_region_refs(region_id, false);
} catch (...) {
  // The count stays high, so the region is never reclaimed: a bounded leak, and
  // the safe direction to fail in.
}

void * resolve_gpu_slot(
  const uint32_t region_id, const uint32_t slot_index, const uint64_t bytes) noexcept
try {
  if (region_id == 0) return nullptr;

  Registry & state = registry();
  const std::shared_lock<std::shared_mutex> lock(state.rwlock);
  const auto it = state.regions.find(region_id);
  if (it == state.regions.end()) {
    // Silence here is the trap the declaration-based API exists to avoid: a
    // message whose region was never mapped resolves to nullptr, which a kernel
    // launch turns into an illegal access that poisons the context.
    RCLCPP_ERROR_ONCE(
      logger,
      "GPU region %u is not mapped in this process: declare the message with reads() so "
      "dispatch() maps it, or borrow it with the capacity overload if it is being published",
      region_id);
    return nullptr;
  }
  return it->second.region.slot_address(slot_index, bytes);
} catch (...) {
  return nullptr;
}

}  // namespace agnocast::internal
