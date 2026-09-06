#include "vmm_backend.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "cuda_driver_loader.hpp"

#include <cstring>

namespace agnocast::gpu
{

using agnocast::internal::GpuMemoryBackendType;
using agnocast::internal::GpuRegionExport;
using agnocast::internal::GpuRegionGeometry;
using agnocast::internal::MappedGpuRegion;
using agnocast::internal::UniqueFd;
using agnocast::internal::VmmExportHandle;

namespace
{

// requestedHandleTypes must be set even when only querying granularity, since
// granularity may depend on the handle type the allocation will use.
void fill_prop(CUmemAllocationProp & prop, CUdevice device)
{
  prop = {};
  prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  prop.location.id = static_cast<int>(device);
  prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
}

size_t round_up(size_t value, size_t multiple)
{
  return ((value + multiple - 1) / multiple) * multiple;
}

}  // namespace

VmmBackend::ScopedContext::ScopedContext(const CudaDriverLoader * cuda, CUcontext ctx) : cuda_(cuda)
{
  const CUresult r = cuda_->cuCtxPushCurrent(ctx);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuCtxPushCurrent failed: %s", cuda_->describe(r).c_str());
    return;
  }
  pushed_ = true;
}

VmmBackend::ScopedContext::~ScopedContext()
{
  if (!pushed_) return;

  CUcontext popped = nullptr;
  const CUresult r = cuda_->cuCtxPopCurrent(&popped);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuCtxPopCurrent failed: %s", cuda_->describe(r).c_str());
  }
}

bool VmmBackend::ensure_context() const
{
  const std::lock_guard<std::mutex> lock(mtx_);
  if (context_ready_) return true;

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  if (cuda == nullptr) return false;

  CUresult r = cuda->cuInit(0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuInit failed: %s", cuda->describe(r).c_str());
    return false;
  }

  // Follow the device the caller already selected. Assuming ordinal 0 would place
  // message buffers on a different GPU than the user's kernels run on.
  CUcontext current = nullptr;
  const bool has_context = cuda->cuCtxGetCurrent(&current) == CUDA_SUCCESS && current != nullptr;
  r = has_context ? cuda->cuCtxGetDevice(&device_) : cuda->cuDeviceGet(&device_, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cannot resolve device: %s", cuda->describe(r).c_str());
    return false;
  }

  CUuuid uuid = {};
  r = cuda->cuDeviceGetUuid_v2(&uuid, device_);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuDeviceGetUuid_v2 failed: %s", cuda->describe(r).c_str());
    return false;
  }
  std::memcpy(device_uuid_.data(), uuid.bytes, device_uuid_.size());

  // Retain the primary context rather than create one, so message buffers are
  // addressable by the user's runtime-API kernels. The retain is held for the
  // life of the process; note it does not protect against cudaDeviceReset(),
  // which destroys the context's resources whether or not it has been retained.
  r = cuda->cuDevicePrimaryCtxRetain(&context_, device_);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuDevicePrimaryCtxRetain failed: %s", cuda->describe(r).c_str());
    return false;
  }

  context_ready_ = true;
  return true;
}

bool VmmBackend::has_attribute(CUdevice_attribute attr, int number, const char * name) const
{
  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  int value = 0;
  const CUresult r = cuda->cuDeviceGetAttribute(&value, attr, device_);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: querying device attribute %d (%s) failed: %s", number, name,
      cuda->describe(r).c_str());
    return false;
  }
  if (value == 0) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: device attribute %d (%s) is 0; this GPU cannot share memory.", number,
      name);
    return false;
  }
  return true;
}

bool VmmBackend::is_supported() const noexcept
try {
  if (!ensure_context()) return false;

  // Attribute numbers are quoted because that is how the CUDA documentation
  // identifies them.
  return has_attribute(
           CU_DEVICE_ATTRIBUTE_VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED, 102,
           "VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED") &&
         has_attribute(
           CU_DEVICE_ATTRIBUTE_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED, 103,
           "HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED");
} catch (...) {
  // Locking and logging can throw; a capability probe must not terminate.
  return false;
}

size_t VmmBackend::query_granularity() const
{
  const CudaDriverLoader * cuda = CudaDriverLoader::instance();

  CUmemAllocationProp prop;
  fill_prop(prop, device_);

  // MINIMUM, because cuMemCreate and cuMemMap state their size and alignment
  // requirements against it.
  size_t granularity = 0;
  const CUresult r =
    cuda->cuMemGetAllocationGranularity(&granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
  if (r != CUDA_SUCCESS || granularity == 0) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemGetAllocationGranularity failed: %s", cuda->describe(r).c_str());
    return 0;
  }
  return granularity;
}

bool VmmBackend::map_and_grant(
  CUmemGenericAllocationHandle handle, size_t size, size_t granularity, void ** out_base) const
{
  const CudaDriverLoader * cuda = CudaDriverLoader::instance();

  // cuMemMap requires a granularity-aligned address, and the API does not define
  // its default alignment to be that granularity.
  CUdeviceptr ptr = 0;
  CUresult r = cuda->cuMemAddressReserve(&ptr, size, granularity, 0, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemAddressReserve(%zu) failed: %s", size, cuda->describe(r).c_str());
    return false;
  }

  r = cuda->cuMemMap(ptr, size, 0, handle, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuMemMap failed: %s", cuda->describe(r).c_str());
    cuda->cuMemAddressFree(ptr, size);
    return false;
  }

  // Access is granted per process; the exporter's grant does not carry over.
  CUmemAccessDesc access = {};
  access.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access.location.id = static_cast<int>(device_);
  access.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;

  r = cuda->cuMemSetAccess(ptr, size, &access, 1);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cuMemSetAccess failed: %s", cuda->describe(r).c_str());
    cuda->cuMemUnmap(ptr, size);
    cuda->cuMemAddressFree(ptr, size);
    return false;
  }

  *out_base = reinterpret_cast<void *>(ptr);
  return true;
}

MappedGpuRegion VmmBackend::create_region(uint32_t slot_size, uint32_t slot_count)
{
  if (slot_size == 0 || slot_count == 0) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: invalid region geometry (slot_size=%u, slot_count=%u)", slot_size,
      slot_count);
    return MappedGpuRegion{};
  }
  if (!ensure_context()) return MappedGpuRegion{};

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return MappedGpuRegion{};

  const size_t granularity = query_granularity();
  if (granularity == 0) return MappedGpuRegion{};

  // The region is rounded, not each slot, so peers compute slot offsets without
  // knowing the granularity and the rounding waste is one granule per region.
  const size_t total =
    round_up(static_cast<size_t>(slot_size) * static_cast<size_t>(slot_count), granularity);

  CUmemAllocationProp prop;
  fill_prop(prop, device_);

  CUmemGenericAllocationHandle handle = 0;
  const CUresult r = cuda->cuMemCreate(&handle, total, &prop, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemCreate(%zu) failed: %s", total, cuda->describe(r).c_str());
    return MappedGpuRegion{};
  }

  void * base = nullptr;
  if (!map_and_grant(handle, total, granularity, &base)) {
    cuda->cuMemRelease(handle);
    return MappedGpuRegion{};
  }

  GpuRegionGeometry geometry;
  geometry.slot_size = slot_size;
  geometry.slot_count = slot_count;
  geometry.mapped_size = total;
  geometry.device_uuid = device_uuid_;
  return MappedGpuRegion(*this, base, geometry, static_cast<uint64_t>(handle));
}

std::optional<GpuRegionExport> VmmBackend::export_for(
  const MappedGpuRegion & region, topic_local_id_t /*subscriber_id*/)
{
  // A VMM shareable handle is not bound to a destination, so every subscriber
  // receives an equivalent export.
  if (!region.valid()) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cannot export an unmapped region");
    return std::nullopt;
  }
  if (!ensure_context()) return std::nullopt;

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return std::nullopt;

  int raw_fd = -1;
  const CUresult r = cuda->cuMemExportToShareableHandle(
    &raw_fd, static_cast<CUmemGenericAllocationHandle>(region.backend_token()),
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemExportToShareableHandle failed: %s", cuda->describe(r).c_str());
    return std::nullopt;
  }

  GpuRegionExport exported;
  exported.backend = GpuMemoryBackendType::Vmm;
  exported.geometry = region.geometry();
  exported.handle = VmmExportHandle{UniqueFd(raw_fd)};
  return exported;
}

MappedGpuRegion VmmBackend::import_region(const GpuRegionExport & exported)
{
  const auto * vmm = std::get_if<VmmExportHandle>(&exported.handle);
  if (exported.backend != GpuMemoryBackendType::Vmm || vmm == nullptr || !vmm->fd.valid()) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: region export is not a VMM export (backend=%u)",
      static_cast<uint32_t>(exported.backend));
    return MappedGpuRegion{};
  }
  // Geometry crosses a process boundary, so it is checked rather than trusted.
  if (!exported.geometry.is_consistent()) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: region geometry does not fit its mapping (%u * %u > %lu)",
      exported.geometry.slot_size, exported.geometry.slot_count, exported.geometry.mapped_size);
    return MappedGpuRegion{};
  }
  if (!ensure_context()) return MappedGpuRegion{};

  if (exported.geometry.device_uuid != device_uuid_) {
    RCLCPP_ERROR(
      logger,
      "Agnocast GPU: the publisher's region is on a different GPU than this process uses. "
      "Check CUDA_VISIBLE_DEVICES on both processes.");
    return MappedGpuRegion{};
  }

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return MappedGpuRegion{};

  // The descriptor is borrowed, not surrendered: the caller's UniqueFd closes it.
  // NVIDIA documents no ownership rule for this API -- the wording about the
  // driver taking the descriptor belongs to cudaImportExternalMemory, which is a
  // different mechanism -- so it was measured instead. On driver 610.43.02 the
  // import duplicates no descriptor, leaves ours open, and the allocation stays
  // usable after we close it, meaning the driver's reference is on the memory
  // object and is released by the importer's own cuMemRelease.
  CUmemGenericAllocationHandle handle = 0;
  const CUresult r = cuda->cuMemImportFromShareableHandle(
    &handle, reinterpret_cast<void *>(static_cast<uintptr_t>(vmm->fd.get())),
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemImportFromShareableHandle failed: %s", cuda->describe(r).c_str());
    return MappedGpuRegion{};
  }

  const size_t granularity = query_granularity();
  if (granularity == 0) {
    cuda->cuMemRelease(handle);
    return MappedGpuRegion{};
  }

  void * base = nullptr;
  if (!map_and_grant(handle, exported.geometry.mapped_size, granularity, &base)) {
    cuda->cuMemRelease(handle);
    return MappedGpuRegion{};
  }

  return MappedGpuRegion(*this, base, exported.geometry, static_cast<uint64_t>(handle));
}

void VmmBackend::release_region(
  void * base, const GpuRegionGeometry & geometry, uint64_t backend_token) noexcept
try {
  if (!ensure_context()) return;

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return;

  // Makes a violation of the no-in-flight-work contract deterministic rather
  // than a fault at an arbitrary later point.
  CUresult r = cuda->cuCtxSynchronize();
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(
      logger, "Agnocast GPU: cuCtxSynchronize before release failed: %s",
      cuda->describe(r).c_str());
  }

  // Each failure leaks device memory or address space for the process lifetime,
  // and the caller has no other way to learn of it.
  const auto ptr = reinterpret_cast<CUdeviceptr>(base);
  r = cuda->cuMemUnmap(ptr, geometry.mapped_size);
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemUnmap failed: %s", cuda->describe(r).c_str());
  }
  r = cuda->cuMemAddressFree(ptr, geometry.mapped_size);
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemAddressFree failed: %s", cuda->describe(r).c_str());
  }
  r = cuda->cuMemRelease(static_cast<CUmemGenericAllocationHandle>(backend_token));
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemRelease failed: %s", cuda->describe(r).c_str());
  }
} catch (...) {
  // Reached from ~MappedGpuRegion. Locking and logging can throw, and
  // terminating during teardown would be worse than the leak it implies.
}

}  // namespace agnocast::gpu
