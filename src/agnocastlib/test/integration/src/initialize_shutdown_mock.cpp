#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <string>

int shm_fd;

struct initialize_agnocast_result
{
  void * mempool_ptr;
  uint64_t mempool_size;
};

namespace agnocast
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
extern "C" struct initialize_agnocast_result initialize_agnocast(
  const unsigned char * heaphook_version_ptr, const size_t heaphook_version_str_len)
{
  // 8GB: same as kernel module default
  constexpr uint64_t shm_size = 8ULL * 1024ULL * 1024ULL * 1024ULL;

  shm_fd = memfd_create("agnocast_shm_heaphook_test", MFD_CLOEXEC);
  if (shm_fd == -1) {
    return {nullptr, 0};
  }

  if (ftruncate(shm_fd, static_cast<off_t>(shm_size)) == -1) {
    return {nullptr, 0};
  }

  int prot = PROT_READ | PROT_WRITE;
  void * ret = mmap(
    reinterpret_cast<void *>(0x40000000000), shm_size, prot, MAP_SHARED | MAP_FIXED_NOREPLACE,
    shm_fd, 0);

  if (ret == MAP_FAILED) {
    return {nullptr, 0};
  }

  return {ret, shm_size};
}
#pragma GCC diagnostic pop

}  // namespace agnocast
