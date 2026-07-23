#include "cuda_ipc_client_backend.hpp"

#include "cudart_loader.hpp"

#include <cstdio>
#include <cstring>

namespace agnocast::cuda
{

namespace
{

bool cuda_ok(cudaError_t err, const char * operation)
{
  if (err != cudaSuccess) {
    std::fprintf(
      stderr, "[agnocast_cuda] %s failed: %s\n", operation,
      CudartLoader::instance().cudaGetErrorString(err));
    return false;
  }
  return true;
}

// Formats a 16-byte CUDA UUID as the canonical "GPU-8-4-4-4-12" lowercase-hex
// string, matching `nvidia-smi -L` and the daemon's formatting.
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

// Copies a fixed-size IPC handle blob out of a wire vector. Returns false if the
// blob is not the expected 64 bytes.
template <typename HandleT>
bool blob_to_handle(const std::vector<std::uint8_t> & blob, HandleT & out)
{
  if (blob.size() != sizeof(out)) {
    std::fprintf(
      stderr, "[agnocast_cuda] unexpected IPC handle size: %zu (want %zu)\n", blob.size(),
      sizeof(out));
    return false;
  }
  std::memcpy(&out, blob.data(), sizeof(out));
  return true;
}

}  // namespace

bool CudaIpcClientBackend::local_gpu_uuid(std::string & uuid_out)
{
  auto & cuda = CudartLoader::instance();
  int device = 0;
  if (!cuda_ok(cuda.cudaGetDevice(&device), "cudaGetDevice")) {
    return false;
  }
  cudaUUID_t uuid{};
  if (!cuda_ok(cuda.cudaDeviceGetUuid(&uuid, device), "cudaDeviceGetUuid")) {
    return false;
  }
  uuid_out = format_gpu_uuid(uuid);
  return true;
}

bool CudaIpcClientBackend::import_slot(
  const gpu_shared_memory_daemon::SlotDescriptor & descriptor, ImportedSlot & out)
{
  auto & cuda = CudartLoader::instance();
  out = ImportedSlot{};

  cudaIpcMemHandle_t mem_handle{};
  cudaIpcEventHandle_t ready_handle{};
  cudaIpcEventHandle_t done_handle{};
  if (
    !blob_to_handle(descriptor.mem_handle, mem_handle) ||
    !blob_to_handle(descriptor.data_ready_event, ready_handle) ||
    !blob_to_handle(descriptor.data_done_event, done_handle)) {
    return false;
  }

  void * device_ptr = nullptr;
  if (!cuda_ok(
        cuda.cudaIpcOpenMemHandle(&device_ptr, mem_handle, cudaIpcMemLazyEnablePeerAccess),
        "cudaIpcOpenMemHandle")) {
    return false;
  }

  cudaEvent_t ready_event = nullptr;
  cudaEvent_t done_event = nullptr;
  if (!cuda_ok(cuda.cudaIpcOpenEventHandle(&ready_event, ready_handle), "cudaIpcOpenEventHandle")) {
    cuda.cudaIpcCloseMemHandle(device_ptr);
    return false;
  }
  if (!cuda_ok(cuda.cudaIpcOpenEventHandle(&done_event, done_handle), "cudaIpcOpenEventHandle")) {
    cuda.cudaEventDestroy(ready_event);
    cuda.cudaIpcCloseMemHandle(device_ptr);
    return false;
  }

  out.device_ptr = device_ptr;
  out.data_ready_event = ready_event;
  out.data_done_event = done_event;
  return true;
}

void CudaIpcClientBackend::release_slot(ImportedSlot & imported)
{
  auto & cuda = CudartLoader::instance();
  if (imported.data_done_event != nullptr) {
    cuda.cudaEventDestroy(imported.data_done_event);
  }
  if (imported.data_ready_event != nullptr) {
    cuda.cudaEventDestroy(imported.data_ready_event);
  }
  if (imported.device_ptr != nullptr) {
    cuda.cudaIpcCloseMemHandle(imported.device_ptr);
  }
  imported = ImportedSlot{};
}

bool CudaIpcClientBackend::record_data_ready(const ImportedSlot & slot)
{
  auto & cuda = CudartLoader::instance();
  // Records on the per-thread default stream, so the publisher's GPU writes on
  // that stream are captured without serializing with other streams.
  return cuda_ok(
    cuda.cudaEventRecord(slot.data_ready_event, cuda_stream_per_thread()), "cudaEventRecord");
}

bool CudaIpcClientBackend::wait_data_ready(const ImportedSlot & slot)
{
  auto & cuda = CudartLoader::instance();
  // Makes the caller's per-thread default stream wait for the publisher's write.
  return cuda_ok(
    cuda.cudaStreamWaitEvent(cuda_stream_per_thread(), slot.data_ready_event, cudaEventWaitDefault),
    "cudaStreamWaitEvent");
}

}  // namespace agnocast::cuda
