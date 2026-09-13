#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_backend.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <sys/ioctl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace agnocast::internal
{

namespace
{

// One mapped region. `imported` separates a region this process created, whose
// lifetime its slot pool owns, from one received from a peer, which is released
// once nothing can refer to it any more.
struct RegionEntry
{
  MappedGpuRegion region;
  // Separates a region this process created, whose lifetime its slot pool owns,
  // from one imported from a peer, which is released once the module no longer
  // holds it and nothing here still refers to it.
  bool imported = false;
};

// How many received message handles in this process still refer to a region,
// keyed by region id and kept independently of whether the region is mapped: a
// handle is created before the frame that maps its region, so a count stored in
// the mapping itself would miss the first message.
//
// This is what makes releasing an imported region safe. The kmod's entry
// accounting is not enough on its own: ~SubscriptionBase issues
// REMOVE_SUBSCRIBER while userspace may still hold handles, after which the
// publisher's registration can disappear even though a message is still in use
// here. The host data path sidesteps the whole question by never unmapping.
std::unordered_map<uint32_t, uint32_t> & region_refs()
{
  static auto * refs =
    new std::unordered_map<uint32_t, uint32_t>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *refs;
}

// Leaked for the same reason as the backend registry: a message destructor can
// reach this after static destruction would have run.
//
// Shared rather than exclusive because resolving a slot is a read on the message
// path, while mapping and releasing a region are cold. It also guards
// region_refs().
std::shared_mutex & table_rwlock()
{
  static auto * mtx = new std::shared_mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, RegionEntry> & table()
{
  static auto * regions =
    new std::unordered_map<uint32_t, RegionEntry>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *regions;
}

// Bumped under the exclusive lock whenever the table changes, so that a cached
// resolution can tell in one relaxed load whether it is still current.
std::atomic<uint64_t> & table_generation()
{
  static auto * gen = new std::atomic<uint64_t>(1);  // NOLINT(cppcoreguidelines-owning-memory)
  return *gen;
}

// Which of `region_ids` the module still holds. Ids are unique for its lifetime
// and never reused, so "gone" is final: no later message can name one, and no
// publisher or subscriber identity is needed to ask -- which matters, because
// the case this exists for is precisely the one where the exporting publisher
// is already gone and the importing subscription may be too.
//
// A failed call reports everything as still held: releasing a live region on a
// transient error would leave a later message unable to resolve, which is far
// worse than keeping a mapping until the next sweep.
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

// Whether a handle in this process can still resolve a payload in this region.
// Caller holds the lock, in either mode.
bool region_is_referenced(const uint32_t region_id)
{
  const auto refs = region_refs().find(region_id);
  return refs != region_refs().end() && refs->second != 0;
}

// One export serves every subscriber, because the kmod holds the reference and
// hands out a descriptor for the same open file per importer.
uint32_t create_via_kmod(
  GpuMemoryBackend & backend, const std::string_view topic_name,
  const topic_local_id_t publisher_id, MappedGpuRegion & region)
{
  const std::optional<GpuRegionExport> exported = backend.export_region(region);
  if (!exported) return 0;

  union ioctl_add_gpu_region_args args = {};
  args.topic_name = {topic_name.data(), topic_name.size()};
  args.publisher_id = publisher_id;
  args.backend_type = static_cast<uint32_t>(exported->backend);
  args.slot_size = exported->geometry.slot_size;
  args.slot_count = exported->geometry.slot_count;
  args.mapped_size = exported->geometry.mapped_size;
  std::memcpy(args.device_uuid, exported->geometry.device_uuid.data(), GPU_DEVICE_UUID_SIZE);
  args.handle_fd = -1;

  // Unreachable while VMM is the only mechanism, but silently registering a
  // region the module would refuse -- or worse, one whose handle it reads as the
  // wrong kind -- is not the way to find out that changed.
  const auto * vmm = std::get_if<VmmExportHandle>(&exported->handle);
  if (vmm == nullptr) {
    RCLCPP_ERROR(
      logger, "the GPU backend exported region of topic '%.*s' as mechanism %u, which is not VMM",
      static_cast<int>(topic_name.size()), topic_name.data(),
      static_cast<uint32_t>(exported->backend));
    return 0;
  }
  args.handle_fd = vmm->fd.get();

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
  if (args.ret_handle_fd >= 0) {
    exported.handle = VmmExportHandle{UniqueFd(args.ret_handle_fd)};
  }

  out_region_id = args.ret_region_id;
  return backend.import_region(exported);
}

// Imported regions the module no longer holds -- because their publisher is
// gone, or because it retired them and carried on. Both are the same question
// once it is asked about the region rather than about the publisher, and asking
// it that way is also what makes the answer exact: a publisher that retires a
// region answers "still registered" for as long as it lives, and so does one
// whose topic-local id has been handed to a restarted successor.
//
// Three phases, because the middle one is a syscall and a thread resolving a
// slot must not queue behind it. Only the first and last take the lock, and the
// reference test is repeated under the second: the module's answer was obtained
// without the lock, and a message naming the region may have arrived since.
//
// Returned rather than destroyed so the caller can unmap outside the lock too --
// releasing a region synchronizes the whole device.
std::vector<RegionEntry> collect_unreachable_regions()
{
  std::vector<uint32_t> candidates;
  {
    const std::shared_lock<std::shared_mutex> lock(table_rwlock());
    for (const auto & [region_id, entry] : table()) {
      if (!entry.imported || region_is_referenced(region_id)) continue;
      candidates.push_back(region_id);
    }
  }
  if (candidates.empty()) return {};

  std::vector<bool> held;
  query_regions_still_held(candidates, held);

  std::vector<RegionEntry> released;
  {
    const std::lock_guard<std::shared_mutex> lock(table_rwlock());
    for (size_t i = 0; i < candidates.size(); i++) {
      if (held[i]) continue;
      const uint32_t region_id = candidates[i];
      if (region_is_referenced(region_id)) continue;
      const auto it = table().find(region_id);
      if (it == table().end()) continue;
      RCLCPP_DEBUG(
        logger, "releasing the mapping of GPU region %u: the kernel module no longer holds it",
        region_id);
      released.push_back(std::move(it->second));
      table().erase(it);
    }
    if (!released.empty()) table_generation().fetch_add(1, std::memory_order_release);
  }
  return released;
}

}  // namespace

GpuRegionRegistry & GpuRegionRegistry::instance()
{
  static auto * registry = new GpuRegionRegistry();  // NOLINT(cppcoreguidelines-owning-memory)
  return *registry;
}

bool GpuRegionRegistry::is_mapped(const uint32_t region_id) const
{
  const std::shared_lock<std::shared_mutex> lock(table_rwlock());
  return table().count(region_id) != 0;
}

void * GpuRegionRegistry::resolve(
  const uint32_t region_id, const uint32_t slot_index, const uint64_t bytes) const
{
  if (region_id == 0) return nullptr;

  // A node resolves the same few regions over and over, so each thread caches
  // what it has resolved and consults the table only on a miss. Without this,
  // every access to a payload would serialize on a process-wide lock, which is
  // not what a zero-copy data path should do. Four entries because a node that
  // filters reads one region and writes another on every frame, and one that
  // subscribes to a couple of publishers alternates between theirs; a single
  // entry would miss every time in both cases.
  //
  // The generation makes an entry self-invalidating: any change to the table
  // makes every cached entry miss, so a released region is never resolved
  // through a stale cache.
  struct CacheEntry
  {
    uint64_t generation = 0;
    uint32_t region_id = 0;
    void * base = nullptr;
    uint32_t slot_size = 0;
    uint32_t slot_count = 0;
  };
  constexpr size_t cache_size = 4;
  thread_local std::array<CacheEntry, cache_size> cache;
  thread_local size_t next_victim = 0;

  const uint64_t generation = table_generation().load(std::memory_order_acquire);

  const CacheEntry * hit = nullptr;
  for (const CacheEntry & entry : cache) {
    // cppcheck-suppress useStlAlgorithm ; a four-element scan, clearer as a loop
    if (entry.region_id == region_id && entry.generation == generation) {
      hit = &entry;
      break;
    }
  }

  if (hit == nullptr) {
    CacheEntry filled;
    {
      const std::shared_lock<std::shared_mutex> lock(table_rwlock());
      const auto it = table().find(region_id);
      if (it == table().end()) {
        // Silence here is the trap the declaration-based API exists to avoid: a
        // message whose region was never mapped resolves to nullptr, which a
        // kernel launch turns into an illegal access that poisons the context.
        RCLCPP_ERROR_ONCE(
          logger,
          "GPU region %u is not mapped in this process: declare the message with reads() so "
          "dispatch() maps it, or borrow it with the capacity overload if it is being published",
          region_id);
        return nullptr;
      }
      // slot_address(0, 0) is the mapping base, null only for an invalid region,
      // which should never be in the table.
      filled.base = it->second.region.slot_address(0, 0);
      if (filled.base == nullptr) return nullptr;
      filled.slot_size = it->second.region.geometry().slot_size;
      filled.slot_count = it->second.region.geometry().slot_count;
    }
    filled.region_id = region_id;
    filled.generation = generation;
    cache[next_victim] = filled;
    hit = &cache[next_victim];
    next_victim = (next_victim + 1) % cache_size;
  }

  // The same bounds the mapping itself applies, against the cached geometry.
  if (slot_index >= hit->slot_count || bytes > hit->slot_size) return nullptr;
  return static_cast<uint8_t *>(hit->base) + static_cast<uint64_t>(slot_index) * hit->slot_size;
}

uint32_t GpuRegionRegistry::create(
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

  const std::lock_guard<std::shared_mutex> lock(table_rwlock());
  RegionEntry entry;
  entry.region = std::move(region);
  entry.imported = false;
  table().insert_or_assign(region_id, std::move(entry));
  table_generation().fetch_add(1, std::memory_order_release);
  return region_id;
}

bool GpuRegionRegistry::ensure_mapped(const GpuRegionRef & ref)
{
  if (ref.region_id != 0 && is_mapped(ref.region_id)) return true;

  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return false;

  uint32_t region_id = 0;
  MappedGpuRegion region = import_via_kmod(*backend, ref, region_id);
  if (!region.valid()) return false;

  RegionEntry entry;
  entry.region = std::move(region);
  entry.imported = true;

  // Swept before the insertion, so the region just imported is not in the table
  // yet and cannot be swept out from under the caller about to resolve against
  // it. Outside the lock, which is where the sweep does its own locking around
  // the call it has to make.
  // cppcheck-suppress variableScope ; must outlive the lock scope below
  std::vector<RegionEntry> released = collect_unreachable_regions();
  {
    const std::lock_guard<std::shared_mutex> lock(table_rwlock());
    // Presence is tested before the insert rather than after, because emplace
    // may consume the argument even when it does not insert: moving from it
    // twice would leave the loser's region owned by nobody and leaked. A
    // concurrent importer may have won the race, in which case the mapping in
    // use is kept and ours joins `released` -- releasing a region synchronizes
    // the whole device, which no resolver should wait on under the lock.
    if (table().count(region_id) == 0) {
      table().emplace(region_id, std::move(entry));
    } else {
      released.push_back(std::move(entry));
    }
    table_generation().fetch_add(1, std::memory_order_release);
  }
  return true;
}

void GpuRegionRegistry::destroy(
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

  unmap_local(region_id);
}

void GpuRegionRegistry::unmap_local(const uint32_t region_id)
{
  if (region_id == 0) return;

  // Deliberately not gated on region_is_referenced(), unlike the two sweeps: the
  // callers are a publisher retiring a region it has proven idle, and publisher
  // teardown, where waiting for a handle to be dropped would pin this mapping
  // and its share of device memory for the life of the process. See
  // ~GpuSlotPool. A handle the owning process still holds at teardown therefore
  // resolves to nullptr rather than to unmapped memory.
  //
  // As in create(): releasing a mapping calls into the driver, which allocates
  // host memory of its own, and a publisher retiring a region while another
  // borrow is outstanding would otherwise leave that bookkeeping in the mempool.
  const SuspendedBorrowWindow suspended;

  // Held past the lock on purpose: releasing a region synchronizes the whole
  // device, and a thread resolving a slot of another region should not wait on
  // that. The destructor at the end of this scope is what unmaps.
  RegionEntry released;
  {
    const std::lock_guard<std::shared_mutex> lock(table_rwlock());
    const auto it = table().find(region_id);
    if (it == table().end()) return;
    // cppcheck-suppress unreadVariable ; holds the region so it unmaps after the lock
    released = std::move(it->second);
    table().erase(it);
    table_generation().fetch_add(1, std::memory_order_release);
  }
}

namespace
{

void adjust_region_refs(const uint32_t region_id, const bool take)
{
  if (region_id == 0) return;

  // The map node, and any rehash of the bucket array, are this library's
  // bookkeeping rather than part of a message. A subscriber can be inside an
  // open borrow window when it takes a handle -- a filter node that receives
  // while holding a borrow -- and without this they would be served from the
  // mempool and stay resident in the segment every peer maps.
  const SuspendedBorrowWindow suspended;

  const std::lock_guard<std::shared_mutex> lock(table_rwlock());
  if (take) {
    region_refs()[region_id]++;
    return;
  }
  const auto it = region_refs().find(region_id);
  if (it == region_refs().end()) return;
  if (--it->second == 0) region_refs().erase(it);
}

}  // namespace

// The three below are noexcept because their callers are: a message destructor,
// and an accessor on the payload of a message being read. Locking and the map
// insert can both throw, and logging -- which allocates -- can throw inside the
// borrow window where the mempool is what serves it. A function-try-block, as on
// UniqueFd::reset and VmmBackend::release_region, so a throw costs the
// bookkeeping rather than the process.

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
  return GpuRegionRegistry::instance().resolve(region_id, slot_index, bytes);
} catch (...) {
  return nullptr;
}

}  // namespace agnocast::internal
