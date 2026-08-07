#include "agnocast/internal/gpu_region.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <dlfcn.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <unordered_map>

namespace agnocast::internal
{

void UniqueFd::reset()
{
  if (fd_ >= 0) {
    if (close(fd_) != 0) {
      // A failure here means the descriptor was already closed elsewhere, which
      // is a double-ownership bug worth surfacing.
      RCLCPP_WARN(logger, "close() failed for a GPU region descriptor: %s", strerror(errno));
    }
    fd_ = -1;
  }
}

namespace
{

// Leaked deliberately. A user's static-duration object holding a GPU publisher is
// destroyed in an order this library does not control, so teardown can reach the
// registry after namespace-scope statics would already be gone.
std::mutex & backend_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

GpuMemoryBackendSelector & backend_selector()
{
  static auto * selector =
    new GpuMemoryBackendSelector(nullptr);  // NOLINT(cppcoreguidelines-owning-memory)
  return *selector;
}

// agnocast_gpu is loaded, not linked. A node that reaches GPU memory only through
// agnocastlib's API references none of its symbols, so --as-needed would drop the
// DT_NEEDED entry and its registering constructor would never run.
void ensure_backend_loaded()
{
  static const bool loaded = [] {
    // RTLD_NODELETE because regions dispatch into this library from their
    // destructors, which can run after anything that might unload it.
    if (dlopen("libagnocast_gpu.so", RTLD_NOW | RTLD_GLOBAL | RTLD_NODELETE) != nullptr) {
      return true;
    }
    const char * error = dlerror();
    RCLCPP_ERROR(
      logger, "Agnocast: GPU memory requested but libagnocast_gpu.so is unavailable: %s",
      error != nullptr ? error : "unknown error");
    return false;
  }();
  (void)loaded;
}

}  // namespace

void register_gpu_memory_backend_selector(GpuMemoryBackendSelector selector)
{
  const std::lock_guard<std::mutex> lock(backend_mutex());
  backend_selector() = selector;
}

GpuMemoryBackend * get_gpu_memory_backend()
{
  // Outside the lock: loading runs the package's constructor, which registers
  // the selector.
  ensure_backend_loaded();

  const std::lock_guard<std::mutex> lock(backend_mutex());

  // Resolved once. Support is a property of the machine, so a negative answer
  // will not change within the process, and probing it costs driver calls.
  static GpuMemoryBackend * resolved = nullptr;
  static bool resolution_attempted = false;
  if (!resolution_attempted) {
    resolution_attempted = true;
    const GpuMemoryBackendSelector selector = backend_selector();
    resolved = (selector != nullptr) ? selector() : nullptr;
    if (resolved == nullptr) {
      RCLCPP_ERROR(logger, "Agnocast: no GPU memory backend is supported on this machine");
    }
  }
  return resolved;
}

}  // namespace agnocast::internal

namespace agnocast::internal
{

bool create_and_register_gpu_region(
  const std::string & topic_name, const topic_local_id_t publisher_id, const uint32_t slot_size,
  const uint32_t slot_count, GpuRegion & out_region, uint32_t & out_region_id)
{
  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return false;

  if (!backend->create_region(slot_size, slot_count, out_region)) return false;

  // The kernel module holds the region's liveness reference, so the export is
  // produced once here rather than per subscriber.
  GpuRegionDescriptor descriptor;
  if (!backend->export_for(out_region, 0, descriptor)) {
    out_region.reset();
    return false;
  }

  union ioctl_add_gpu_region_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.publisher_id = publisher_id;
  args.backend_type = static_cast<uint32_t>(descriptor.backend);
  args.slot_size = descriptor.slot_size;
  args.slot_count = descriptor.slot_count;
  args.mapped_size = descriptor.mapped_size;
  std::memcpy(args.device_uuid, descriptor.device_uuid.data(), GPU_DEVICE_UUID_SIZE);
  args.handle_fd = -1;

  if (const auto * exported = std::get_if<VmmExportHandle>(&descriptor.handle)) {
    args.handle_fd = exported->fd.get();
  } else if (const auto * blob = std::get_if<NvSciBufExportHandle>(&descriptor.handle)) {
    args.blob_addr = reinterpret_cast<uint64_t>(blob->descriptor.data());
    args.blob_size = static_cast<uint32_t>(blob->descriptor.size());
  }

  // The descriptor still owns the handle: the kernel module takes its own
  // reference on success, so closing ours here is correct either way.
  if (ioctl(agnocast_fd, AGNOCAST_ADD_GPU_REGION_CMD, &args) < 0) {
    RCLCPP_ERROR(
      logger, "AGNOCAST_ADD_GPU_REGION_CMD failed for topic '%s': %s", topic_name.c_str(),
      strerror(errno));
    out_region.reset();
    return false;
  }
  out_region_id = args.ret_region_id;
  return true;
}

bool import_gpu_region(
  const std::string & topic_name, const topic_local_id_t publisher_id,
  const topic_local_id_t subscriber_id, const uint32_t wanted_region_id, GpuRegion & out_region,
  uint32_t & out_region_id)
{
  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return false;

  std::vector<uint8_t> blob(MAX_GPU_HANDLE_BLOB_SIZE);

  union ioctl_get_gpu_region_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.publisher_id = publisher_id;
  args.subscriber_id = subscriber_id;
  args.region_id = wanted_region_id;
  args.blob_buffer_addr = reinterpret_cast<uint64_t>(blob.data());
  args.blob_buffer_size = static_cast<uint32_t>(blob.size());

  if (ioctl(agnocast_fd, AGNOCAST_GET_GPU_REGION_CMD, &args) < 0) {
    RCLCPP_ERROR(
      logger, "AGNOCAST_GET_GPU_REGION_CMD failed for topic '%s': %s", topic_name.c_str(),
      strerror(errno));
    return false;
  }

  GpuRegionDescriptor descriptor;
  descriptor.backend = static_cast<GpuMemoryBackendType>(args.ret_backend_type);
  descriptor.slot_size = args.ret_slot_size;
  descriptor.slot_count = args.ret_slot_count;
  descriptor.mapped_size = args.ret_mapped_size;
  std::memcpy(descriptor.device_uuid.data(), args.ret_device_uuid, GPU_DEVICE_UUID_SIZE);

  // The kernel module installed the descriptor in this process, so ownership of
  // it is ours from here.
  if (args.ret_handle_fd >= 0) {
    descriptor.handle = VmmExportHandle{UniqueFd(args.ret_handle_fd)};
  } else if (args.ret_blob_size > 0) {
    blob.resize(args.ret_blob_size);
    descriptor.handle = NvSciBufExportHandle{std::move(blob)};
  }

  if (!backend->import_region(descriptor, out_region)) return false;
  out_region_id = args.ret_region_id;
  return true;
}

namespace
{

std::mutex & region_table_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, GpuRegion> & region_table()
{
  // Leaked for the same reason as the backend registry: a message destructor can
  // reach this after static destruction would have run.
  static auto * table =
    new std::unordered_map<uint32_t, GpuRegion>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *table;
}

}  // namespace

void register_mapped_region(const uint32_t region_id, GpuRegion && region)
{
  const std::lock_guard<std::mutex> lock(region_table_mutex());
  region_table().insert_or_assign(region_id, std::move(region));
}

const GpuRegion * find_mapped_region(const uint32_t region_id)
{
  const std::lock_guard<std::mutex> lock(region_table_mutex());
  const auto it = region_table().find(region_id);
  return (it == region_table().end()) ? nullptr : &it->second;
}

bool ensure_gpu_region_mapped(
  const std::string & topic_name, const topic_local_id_t publisher_id,
  const topic_local_id_t subscriber_id, const uint32_t wanted_region_id)
{
  if (wanted_region_id != 0 && find_mapped_region(wanted_region_id) != nullptr) return true;

  GpuRegion region;
  uint32_t region_id = 0;
  if (!import_gpu_region(
        topic_name, publisher_id, subscriber_id, wanted_region_id, region, region_id)) {
    return false;
  }

  // A second import of the same region would map it twice; keep the first.
  if (find_mapped_region(region_id) == nullptr) {
    register_mapped_region(region_id, std::move(region));
  }
  return true;
}

GpuSlotPool::GpuSlotPool(const uint32_t region_id, const uint32_t slot_count)
: region_id_(region_id)
{
  free_slots_.reserve(slot_count);
  for (uint32_t i = slot_count; i > 0; i--) {
    free_slots_.push_back(i - 1);
  }
}

bool GpuSlotPool::acquire(uint32_t & out_slot_index)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (free_slots_.empty()) return false;
  out_slot_index = free_slots_.back();
  free_slots_.pop_back();
  return true;
}

void GpuSlotPool::release(const uint32_t slot_index)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  free_slots_.push_back(slot_index);
}

size_t GpuSlotPool::available() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size();
}

namespace
{

std::mutex & pool_table_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, GpuSlotPool *> & pool_table()
{
  static auto * table =
    new std::unordered_map<uint32_t, GpuSlotPool *>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *table;
}

}  // namespace

void register_slot_pool(const uint32_t region_id, GpuSlotPool * pool)
{
  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  pool_table()[region_id] = pool;
}

void release_gpu_slot(const uint32_t region_id, const uint32_t slot_index)
{
  GpuSlotPool * pool = nullptr;
  {
    const std::lock_guard<std::mutex> lock(pool_table_mutex());
    const auto it = pool_table().find(region_id);
    if (it == pool_table().end()) return;  // not ours: a peer's region
    pool = it->second;
  }
  pool->release(slot_index);
}

std::unique_ptr<GpuSlotPool> create_gpu_slot_pool(
  const std::string & topic_name, const topic_local_id_t publisher_id, const uint32_t slot_size,
  const uint32_t slot_count)
{
  GpuRegion region;
  uint32_t region_id = 0;
  if (!create_and_register_gpu_region(
        topic_name, publisher_id, slot_size, slot_count, region, region_id)) {
    return nullptr;
  }
  register_mapped_region(region_id, std::move(region));
  auto pool = std::make_unique<GpuSlotPool>(region_id, slot_count);
  register_slot_pool(region_id, pool.get());
  return pool;
}

}  // namespace agnocast::internal
