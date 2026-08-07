#include "vmm_backend.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "cuda_driver_loader.hpp"

#include <cstring>

namespace agnocast::gpu
{

using agnocast::internal::GpuMemoryBackendType;
using agnocast::internal::GpuRegion;
using agnocast::internal::GpuRegionDescriptor;
using agnocast::internal::GpuRegionMapping;
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

bool VmmBackend::create_region(uint32_t slot_size, uint32_t slot_count, GpuRegion & out_region)
{
  if (slot_size == 0 || slot_count == 0) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: invalid region geometry (slot_size=%u, slot_count=%u)", slot_size,
      slot_count);
    return false;
  }
  if (!ensure_context()) return false;

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return false;

  const size_t granularity = query_granularity();
  if (granularity == 0) return false;

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
    return false;
  }

  void * base = nullptr;
  if (!map_and_grant(handle, total, granularity, &base)) {
    cuda->cuMemRelease(handle);
    return false;
  }

  GpuRegionMapping mapping;
  mapping.base = base;
  mapping.mapped_size = total;
  mapping.slot_size = slot_size;
  mapping.slot_count = slot_count;
  mapping.device_uuid = device_uuid_;
  mapping.backend_token = static_cast<uint64_t>(handle);
  adopt(out_region, mapping);
  return true;
}

bool VmmBackend::export_for(
  const GpuRegion & region, topic_local_id_t /*subscriber_id*/, GpuRegionDescriptor & out_desc)
{
  // A VMM shareable handle is not bound to a destination, so every subscriber
  // receives an equivalent descriptor.
  if (!region.valid()) {
    RCLCPP_ERROR(logger, "Agnocast GPU: cannot export an unmapped region");
    return false;
  }
  if (!ensure_context()) return false;

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return false;

  const GpuRegionMapping & mapping = mapping_of(region);

  int raw_fd = -1;
  const CUresult r = cuda->cuMemExportToShareableHandle(
    &raw_fd, static_cast<CUmemGenericAllocationHandle>(mapping.backend_token),
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemExportToShareableHandle failed: %s", cuda->describe(r).c_str());
    return false;
  }

  out_desc.backend = GpuMemoryBackendType::Vmm;
  out_desc.slot_size = mapping.slot_size;
  out_desc.slot_count = mapping.slot_count;
  out_desc.mapped_size = mapping.mapped_size;
  out_desc.device_uuid = mapping.device_uuid;
  out_desc.handle = VmmExportHandle{UniqueFd(raw_fd)};
  return true;
}

bool VmmBackend::import_region(const GpuRegionDescriptor & desc, GpuRegion & out_region)
{
  const auto * exported = std::get_if<VmmExportHandle>(&desc.handle);
  if (desc.backend != GpuMemoryBackendType::Vmm || exported == nullptr || !exported->fd.valid()) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: region descriptor is not a VMM export (backend=%u)",
      static_cast<uint32_t>(desc.backend));
    return false;
  }
  // Geometry crosses a process boundary, so it is checked here rather than
  // trusted. slot_address() derives an offset from slot_size and bounds it only
  // against slot_count, so a slot_count too large for the mapping would produce
  // addresses past its end.
  if (
    desc.slot_size == 0 || desc.slot_count == 0 ||
    static_cast<uint64_t>(desc.slot_size) * desc.slot_count > desc.mapped_size) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: region descriptor geometry does not fit its mapping (%u * %u > %lu)",
      desc.slot_size, desc.slot_count, desc.mapped_size);
    return false;
  }
  if (!ensure_context()) return false;

  if (desc.device_uuid != device_uuid_) {
    RCLCPP_ERROR(
      logger,
      "Agnocast GPU: the publisher's region is on a different GPU than this process uses. "
      "Check CUDA_VISIBLE_DEVICES on both processes.");
    return false;
  }

  const CudaDriverLoader * cuda = CudaDriverLoader::instance();
  const ScopedContext ctx(cuda, context_);
  if (!ctx.ok()) return false;

  // The descriptor is borrowed, not surrendered: the caller's UniqueFd closes it.
  // NVIDIA documents no ownership rule for this API -- the wording about the
  // driver taking the descriptor belongs to cudaImportExternalMemory, which is a
  // different mechanism -- so it was measured instead. On driver 610.43.02 the
  // import duplicates no descriptor, leaves ours open, and the allocation stays
  // usable after we close it, meaning the driver's reference is on the memory
  // object and is released by the importer's own cuMemRelease.
  CUmemGenericAllocationHandle handle = 0;
  const CUresult r = cuda->cuMemImportFromShareableHandle(
    &handle, reinterpret_cast<void *>(static_cast<uintptr_t>(exported->fd.get())),
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  if (r != CUDA_SUCCESS) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: cuMemImportFromShareableHandle failed: %s", cuda->describe(r).c_str());
    return false;
  }

  const size_t granularity = query_granularity();
  if (granularity == 0) {
    cuda->cuMemRelease(handle);
    return false;
  }

  void * base = nullptr;
  if (!map_and_grant(handle, desc.mapped_size, granularity, &base)) {
    cuda->cuMemRelease(handle);
    return false;
  }

  GpuRegionMapping mapping;
  mapping.base = base;
  mapping.mapped_size = desc.mapped_size;
  mapping.slot_size = desc.slot_size;
  mapping.slot_count = desc.slot_count;
  mapping.device_uuid = desc.device_uuid;
  mapping.backend_token = static_cast<uint64_t>(handle);
  adopt(out_region, mapping);
  return true;
}

void VmmBackend::release_region(const GpuRegionMapping & mapping) noexcept
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
  const auto ptr = reinterpret_cast<CUdeviceptr>(mapping.base);
  r = cuda->cuMemUnmap(ptr, mapping.mapped_size);
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemUnmap failed: %s", cuda->describe(r).c_str());
  }
  r = cuda->cuMemAddressFree(ptr, mapping.mapped_size);
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemAddressFree failed: %s", cuda->describe(r).c_str());
  }
  r = cuda->cuMemRelease(static_cast<CUmemGenericAllocationHandle>(mapping.backend_token));
  if (r != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "Agnocast GPU: cuMemRelease failed: %s", cuda->describe(r).c_str());
  }
} catch (...) {
  // Reached from ~GpuRegion. Locking and logging can throw, and terminating
  // during teardown would be worse than the leak the exception implies.
}

}  // namespace agnocast::gpu
