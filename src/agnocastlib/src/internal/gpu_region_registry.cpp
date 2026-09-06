#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_backend.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace agnocast::internal
{

namespace
{

// Leaked for the same reason as the backend registry: a message destructor can
// reach this after static destruction would have run.
std::mutex & table_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, MappedGpuRegion> & table()
{
  static auto * regions =
    new std::unordered_map<uint32_t, MappedGpuRegion>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *regions;
}

// The kmod holds the region's liveness reference, so the export is produced once
// here rather than per subscriber.
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

}  // namespace

GpuRegionRegistry & GpuRegionRegistry::instance()
{
  static auto * registry = new GpuRegionRegistry();  // NOLINT(cppcoreguidelines-owning-memory)
  return *registry;
}

const MappedGpuRegion * GpuRegionRegistry::find(const uint32_t region_id) const
{
  const std::lock_guard<std::mutex> lock(table_mutex());
  const auto it = table().find(region_id);
  return (it == table().end()) ? nullptr : &it->second;
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

  const std::lock_guard<std::mutex> lock(table_mutex());
  table().insert_or_assign(region_id, std::move(region));
  return region_id;
}

bool GpuRegionRegistry::ensure_mapped(const GpuRegionRef & ref)
{
  if (ref.region_id != 0 && find(ref.region_id) != nullptr) return true;

  GpuMemoryBackend * backend = get_gpu_memory_backend();
  if (backend == nullptr) return false;

  uint32_t region_id = 0;
  MappedGpuRegion region = import_via_kmod(*backend, ref, region_id);
  if (!region.valid()) return false;

  const std::lock_guard<std::mutex> lock(table_mutex());
  // A concurrent importer may have won the race; keep the mapping already in use.
  table().emplace(region_id, std::move(region));
  return true;
}

void * resolve_gpu_slot(
  const uint32_t region_id, const uint32_t slot_index, const uint64_t bytes) noexcept
{
  const MappedGpuRegion * region = GpuRegionRegistry::instance().find(region_id);
  if (region == nullptr) return nullptr;
  return region->slot_address(slot_index, bytes);
}

}  // namespace agnocast::internal
