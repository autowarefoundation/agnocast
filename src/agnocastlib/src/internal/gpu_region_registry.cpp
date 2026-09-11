#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_backend.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <sys/ioctl.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <string>
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
  bool imported = false;
  // What to ask the kmod about to find out whether the exporting publisher is
  // still registered. Imported regions only.
  std::string topic_name;
  topic_local_id_t publisher_id = -1;
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

// Whether the kmod still knows the publisher that exported a region. Its QoS is
// asked for because that is a read-only query keyed on exactly the publisher;
// the answer that matters is only whether it is there at all.
//
// Anything other than the module's "no such topic or publisher" is treated as
// still registered: releasing a live region on a transient error would leave a
// later message unable to resolve, which is far worse than keeping a mapping.
bool publisher_still_registered(const RegionEntry & entry)
{
  if (agnocast_fd < 0) return true;

  struct ioctl_get_publisher_qos_args args = {};
  args.topic_name = {entry.topic_name.data(), entry.topic_name.size()};
  args.publisher_id = entry.publisher_id;
  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_QOS_CMD, &args) >= 0) return true;
  return errno != EINVAL;
}

// One export serves every subscriber, because the kmod holds the reference and
// hands out a descriptor for the same open file per importer.
uint32_t create_via_kmod(
  GpuMemoryBackend & backend, const std::string_view topic_name,
  const topic_local_id_t publisher_id, MappedGpuRegion & region)
{
  const std::optional<GpuRegionExport> exported = backend.export_for(region, 0);
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

  if (const auto * vmm = std::get_if<VmmExportHandle>(&exported->handle)) {
    args.handle_fd = vmm->fd.get();
  } else if (const auto * blob = std::get_if<NvSciBufExportHandle>(&exported->handle)) {
    args.blob_addr = reinterpret_cast<uint64_t>(blob->descriptor.data());
    args.blob_size = static_cast<uint32_t>(blob->descriptor.size());
  }

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
  std::vector<uint8_t> blob(MAX_GPU_HANDLE_BLOB_SIZE);

  union ioctl_get_gpu_region_args args = {};
  args.topic_name = {ref.topic_name.data(), ref.topic_name.size()};
  args.publisher_id = ref.publisher_id;
  args.subscriber_id = ref.subscriber_id;
  args.region_id = ref.region_id;
  args.blob_buffer_addr = reinterpret_cast<uint64_t>(blob.data());
  args.blob_buffer_size = static_cast<uint32_t>(blob.size());

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
  } else if (args.ret_blob_size > 0) {
    blob.resize(args.ret_blob_size);
    exported.handle = NvSciBufExportHandle{std::move(blob)};
  }

  out_region_id = args.ret_region_id;
  return backend.import_region(exported);
}

// Imported regions nothing can refer to any more, because the kmod has forgotten
// the publisher that exported them: a received message holds a reference on its
// kmod entry, so a publisher with a message still held here cannot have been
// forgotten, and one that has been can never produce another message naming
// these regions, since ids are never reused.
//
// Returned rather than destroyed so the caller can unmap outside the lock.
// Caller holds the lock exclusively.
std::vector<RegionEntry> collect_unreachable_regions()
{
  std::vector<RegionEntry> released;
  for (auto it = table().begin(); it != table().end();) {
    const auto refs = region_refs().find(it->first);
    const bool still_referred_to = refs != region_refs().end() && refs->second != 0;
    if (!it->second.imported || still_referred_to || publisher_still_registered(it->second)) {
      ++it;
      continue;
    }
    RCLCPP_DEBUG(
      logger, "releasing the mapping of GPU region %u: publisher %d of topic '%s' is gone",
      it->first, it->second.publisher_id, it->second.topic_name.c_str());
    released.push_back(std::move(it->second));
    it = table().erase(it);
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
  entry.topic_name = std::string(ref.topic_name);
  entry.publisher_id = ref.publisher_id;

  // Collected under the lock and unmapped after it: releasing a region
  // synchronizes the device, which no one resolving a slot should wait on.
  // cppcheck-suppress variableScope ; must outlive the lock scope below
  std::vector<RegionEntry> released;
  {
    const std::lock_guard<std::shared_mutex> lock(table_rwlock());
    // Swept before the insertion, so the region just imported is not itself a
    // candidate: its publisher answered a moment ago and probing it again would
    // only cost another call into the module.
    released = collect_unreachable_regions();
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
  // Unmapped outside the lock: releasing a region synchronizes the device, and
  // resolving a slot of another region should not wait on that.
}

void GpuRegionRegistry::unmap_local(const uint32_t region_id)
{
  if (region_id == 0) return;

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
  // Unmapped after the lock, as in destroy().
}

namespace
{

void adjust_region_refs(const uint32_t region_id, const bool take) noexcept
{
  if (region_id == 0) return;

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

void ref_gpu_region(const uint32_t region_id) noexcept
{
  adjust_region_refs(region_id, true);
}

void unref_gpu_region(const uint32_t region_id) noexcept
{
  adjust_region_refs(region_id, false);
}

void * resolve_gpu_slot(
  const uint32_t region_id, const uint32_t slot_index, const uint64_t bytes) noexcept
{
  return GpuRegionRegistry::instance().resolve(region_id, slot_index, bytes);
}

}  // namespace agnocast::internal
