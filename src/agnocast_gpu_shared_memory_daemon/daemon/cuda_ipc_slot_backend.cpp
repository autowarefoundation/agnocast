#include "cuda_ipc_slot_backend.hpp"

#include "cudart_loader.hpp"

#include <cstdio>
#include <cstring>

namespace agnocast::gpu_shared_memory_daemon
{

namespace
{

bool cuda_ok(cudaError_t err, const char * operation)
{
  if (err != cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_gpu_shared_memory_daemon] %s failed: %s\n", operation,
      CudartLoader::instance().cudaGetErrorString(err));
    return false;
  }
  return true;
}

// Formats a 16-byte CUDA UUID as the canonical "GPU-8-4-4-4-12" lowercase-hex
// string, matching `nvidia-smi -L`.
std::string format_gpu_uuid(const cudaUUID_t & uuid)
{
  const auto * b = reinterpret_cast<const unsigned char *>(uuid.bytes);
  char text[64];
  std::snprintf(
    text, sizeof(text), "GPU-%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
    b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14],
    b[15]);
  return std::string(text);
}

std::vector<std::uint8_t> to_blob(const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const std::uint8_t *>(data);
  return std::vector<std::uint8_t>(bytes, bytes + size);
}

}  // namespace

bool CudaIpcSlotBackend::initialize(std::string & gpu_uuid_out)
{
  auto & cuda = CudartLoader::instance();

  // The GPU to manage is selected at launch via CUDA_VISIBLE_DEVICES, so the
  // daemon always operates on device 0 within its own visible set.
  if (!cuda_ok(cuda.cudaSetDevice(0), "cudaSetDevice")) {
    return false;
  }

  cudaUUID_t uuid{};
  if (!cuda_ok(cuda.cudaDeviceGetUuid(&uuid, 0), "cudaDeviceGetUuid")) {
    return false;
  }
  gpu_uuid_out = format_gpu_uuid(uuid);
  return true;
}

bool CudaIpcSlotBackend::create_slot(std::size_t size, AllocatedSlotResources & out)
{
  auto & cuda = CudartLoader::instance();
  out = AllocatedSlotResources{};

  void * device_ptr = nullptr;
  if (!cuda_ok(cuda.cudaMalloc(&device_ptr, size), "cudaMalloc")) {
    return false;
  }

  // These 2 flags need to be set together for the events to be usable across processes.
  const unsigned int event_flags = cudaEventInterprocess | cudaEventDisableTiming;
  cudaEvent_t ready_event = nullptr;
  cudaEvent_t done_event = nullptr;
  if (!cuda_ok(
        cuda.cudaEventCreateWithFlags(&ready_event, event_flags), "cudaEventCreateWithFlags")) {
    cuda.cudaFree(device_ptr);
    return false;
  }
  if (!cuda_ok(
        cuda.cudaEventCreateWithFlags(&done_event, event_flags), "cudaEventCreateWithFlags")) {
    cuda.cudaEventDestroy(ready_event);
    cuda.cudaFree(device_ptr);
    return false;
  }

  cudaIpcMemHandle_t mem_handle{};
  cudaIpcEventHandle_t ready_handle{};
  cudaIpcEventHandle_t done_handle{};
  const bool exported =
    cuda_ok(cuda.cudaIpcGetMemHandle(&mem_handle, device_ptr), "cudaIpcGetMemHandle") &&
    cuda_ok(cuda.cudaIpcGetEventHandle(&ready_handle, ready_event), "cudaIpcGetEventHandle") &&
    cuda_ok(cuda.cudaIpcGetEventHandle(&done_handle, done_event), "cudaIpcGetEventHandle");
  if (!exported) {
    cuda.cudaEventDestroy(done_event);
    cuda.cudaEventDestroy(ready_event);
    cuda.cudaFree(device_ptr);
    return false;
  }

  out.device_ptr = device_ptr;
  out.data_ready_event = ready_event;
  out.data_done_event = done_event;
  out.mem_handle = to_blob(&mem_handle, sizeof(mem_handle));
  out.data_ready_event_handle = to_blob(&ready_handle, sizeof(ready_handle));
  out.data_done_event_handle = to_blob(&done_handle, sizeof(done_handle));
  return true;
}

void CudaIpcSlotBackend::destroy_slot(AllocatedSlotResources & resources)
{
  auto & cuda = CudartLoader::instance();
  if (resources.data_done_event != nullptr) {
    cuda.cudaEventDestroy(resources.data_done_event);
  }
  if (resources.data_ready_event != nullptr) {
    cuda.cudaEventDestroy(resources.data_ready_event);
  }
  if (resources.device_ptr != nullptr) {
    cuda.cudaFree(resources.device_ptr);
  }
  resources = AllocatedSlotResources{};
}

}  // namespace agnocast::gpu_shared_memory_daemon
