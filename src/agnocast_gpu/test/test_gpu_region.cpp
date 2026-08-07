#include "agnocast/internal/gpu_region.hpp"
#include "vmm_backend.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <utility>

using agnocast::internal::GpuMemoryBackendType;
using agnocast::internal::GpuRegion;
using agnocast::internal::GpuRegionDescriptor;
using agnocast::internal::UniqueFd;
using agnocast::internal::VmmExportHandle;

namespace
{

int make_test_fd()
{
  const int fd = ::open("/dev/null", O_RDONLY);
  EXPECT_GE(fd, 0);
  return fd;
}

bool fd_is_open(int fd)
{
  return ::fcntl(fd, F_GETFD) != -1;
}

agnocast::internal::GpuMemoryBackend * gpu_backend_or_skip()
{
  auto * backend = agnocast::internal::get_gpu_memory_backend();
  return (backend != nullptr && backend->is_supported()) ? backend : nullptr;
}

}  // namespace

TEST(UniqueFdTest, ClosesOnDestruction)
{
  const int raw = make_test_fd();
  {
    const UniqueFd fd(raw);
    EXPECT_TRUE(fd_is_open(raw));
  }
  EXPECT_FALSE(fd_is_open(raw));
}

TEST(UniqueFdTest, MoveTransfersOwnership)
{
  const int raw = make_test_fd();
  UniqueFd first(raw);
  UniqueFd second(std::move(first));

  EXPECT_FALSE(first.valid());  // NOLINT(bugprone-use-after-move)
  EXPECT_TRUE(fd_is_open(raw));

  second = UniqueFd();
  EXPECT_FALSE(fd_is_open(raw));
}

TEST(UniqueFdTest, ReleaseHandsOwnershipToCaller)
{
  const int raw = make_test_fd();
  int taken = -1;
  {
    UniqueFd fd(raw);
    taken = fd.release();
    EXPECT_FALSE(fd.valid());
  }
  EXPECT_TRUE(fd_is_open(taken));
  ::close(taken);
}

TEST(GpuRegionDescriptorTest, ExportHandleIsMoveOnlyAndTyped)
{
  static_assert(
    !std::is_copy_constructible_v<GpuRegionDescriptor>,
    "a copyable descriptor would duplicate export-handle ownership");

  GpuRegionDescriptor desc;
  EXPECT_TRUE(std::holds_alternative<std::monostate>(desc.handle));

  desc.handle = VmmExportHandle{UniqueFd(make_test_fd())};
  const int raw = std::get<VmmExportHandle>(desc.handle).fd.get();

  const GpuRegionDescriptor moved = std::move(desc);
  ASSERT_TRUE(std::holds_alternative<VmmExportHandle>(moved.handle));
  EXPECT_EQ(std::get<VmmExportHandle>(moved.handle).fd.get(), raw);
  EXPECT_TRUE(fd_is_open(raw));
}

TEST(GpuRegionTest, DefaultConstructedIsInvalidAndReleasesNothing)
{
  GpuRegion region;
  EXPECT_FALSE(region.valid());
  region.reset();  // must be safe with no backend attached
  EXPECT_FALSE(region.valid());
}

// Must hold without the caller having linked agnocast_gpu: --as-needed drops a
// DT_NEEDED entry whose symbols are never referenced.
TEST(BackendRegistryTest, BackendIsSelectedOnDemand)
{
  auto * backend = agnocast::internal::get_gpu_memory_backend();
  if (backend == nullptr) GTEST_SKIP() << "no supported GPU memory backend";
  EXPECT_EQ(backend->type(), GpuMemoryBackendType::Vmm);
}

TEST(VmmBackendGpuTest, CreateExportImportRoundTrip)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  const uint32_t slot_size = 1U << 20;  // below the granularity, so rounding applies
  const uint32_t slot_count = 4;

  GpuRegion region;
  ASSERT_TRUE(backend->create_region(slot_size, slot_count, region));
  EXPECT_TRUE(region.valid());
  EXPECT_EQ(region.slot_count(), slot_count);
  EXPECT_GE(region.mapped_size(), static_cast<uint64_t>(slot_size) * slot_count);

  // Slots tile the region without overlapping.
  EXPECT_EQ(
    static_cast<uint8_t *>(region.slot_address(1)) - static_cast<uint8_t *>(region.slot_address(0)),
    static_cast<ptrdiff_t>(slot_size));

  GpuRegionDescriptor desc;
  ASSERT_TRUE(backend->export_for(region, /*subscriber_id=*/7, desc));
  ASSERT_TRUE(std::holds_alternative<VmmExportHandle>(desc.handle));
  EXPECT_TRUE(std::get<VmmExportHandle>(desc.handle).fd.valid());
  EXPECT_EQ(desc.mapped_size, region.mapped_size());
  EXPECT_EQ(desc.device_uuid, region.device_uuid());

  GpuRegion imported;
  ASSERT_TRUE(backend->import_region(desc, imported));
  EXPECT_EQ(imported.mapped_size(), region.mapped_size());
  EXPECT_EQ(imported.slot_count(), slot_count);
}

TEST(VmmBackendGpuTest, MovedRegionReleasesExactlyOnce)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  GpuRegion region;
  ASSERT_TRUE(backend->create_region(1U << 20, 2, region));

  const GpuRegion moved = std::move(region);
  EXPECT_FALSE(region.valid());  // NOLINT(bugprone-use-after-move)
  EXPECT_TRUE(moved.valid());
}

TEST(VmmBackendGpuTest, ImportRejectsAForeignDevice)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  GpuRegion region;
  ASSERT_TRUE(backend->create_region(1U << 20, 2, region));

  GpuRegionDescriptor desc;
  ASSERT_TRUE(backend->export_for(region, 0, desc));
  desc.device_uuid[0] = static_cast<uint8_t>(desc.device_uuid[0] ^ 0xFFU);

  GpuRegion imported;
  EXPECT_FALSE(backend->import_region(desc, imported));
  EXPECT_FALSE(imported.valid());
}
