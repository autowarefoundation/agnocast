#include "agnocast/internal/gpu_backend.hpp"
#include "vmm_backend.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <utility>

using agnocast::internal::GpuMemoryBackendType;
using agnocast::internal::GpuRegionExport;
using agnocast::internal::GpuRegionGeometry;
using agnocast::internal::MappedGpuRegion;
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

// Geometry arrives from another process, so these are the checks that stand
// between a corrupt message and an out-of-bounds device address.
TEST(GpuRegionGeometryTest, RejectsSlotsThatDoNotFitTheMapping)
{
  EXPECT_TRUE((GpuRegionGeometry{2048, 4, 8192, {}}).is_consistent());
  EXPECT_TRUE((GpuRegionGeometry{2048, 4, 16384, {}}).is_consistent());
  EXPECT_FALSE((GpuRegionGeometry{2048, 4, 8191, {}}).is_consistent());
  EXPECT_FALSE((GpuRegionGeometry{0, 4, 8192, {}}).is_consistent());
  EXPECT_FALSE((GpuRegionGeometry{2048, 0, 8192, {}}).is_consistent());
}

TEST(GpuRegionGeometryTest, BoundsSlotIndexAndPayload)
{
  const GpuRegionGeometry geometry{2048, 4, 8192, {}};

  EXPECT_TRUE(geometry.contains(3, 2048));
  EXPECT_FALSE(geometry.contains(4, 1));
  EXPECT_FALSE(geometry.contains(0, 2049));
  EXPECT_EQ(geometry.offset_of(3), 6144u);
}

TEST(GpuRegionExportTest, HandleIsMoveOnlyAndTyped)
{
  static_assert(
    !std::is_copy_constructible_v<GpuRegionExport>,
    "a copyable export would duplicate handle ownership");

  GpuRegionExport exported;
  EXPECT_TRUE(std::holds_alternative<std::monostate>(exported.handle));

  exported.handle = VmmExportHandle{UniqueFd(make_test_fd())};
  const int raw = std::get<VmmExportHandle>(exported.handle).fd.get();

  const GpuRegionExport moved = std::move(exported);
  ASSERT_TRUE(std::holds_alternative<VmmExportHandle>(moved.handle));
  EXPECT_EQ(std::get<VmmExportHandle>(moved.handle).fd.get(), raw);
  EXPECT_TRUE(fd_is_open(raw));
}

TEST(MappedGpuRegionTest, DefaultConstructedIsInvalidAndReleasesNothing)
{
  MappedGpuRegion region;
  EXPECT_FALSE(region.valid());
  EXPECT_EQ(region.slot_address(0, 0), nullptr);
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

  MappedGpuRegion region = backend->create_region(slot_size, slot_count);
  ASSERT_TRUE(region.valid());
  EXPECT_EQ(region.geometry().slot_count, slot_count);
  EXPECT_GE(region.geometry().mapped_size, static_cast<uint64_t>(slot_size) * slot_count);

  // Slots tile the region without overlapping.
  EXPECT_EQ(
    static_cast<uint8_t *>(region.slot_address(1, slot_size)) -
      static_cast<uint8_t *>(region.slot_address(0, slot_size)),
    static_cast<ptrdiff_t>(slot_size));

  const auto exported = backend->export_for(region, /*subscriber_id=*/7);
  ASSERT_TRUE(exported.has_value());
  ASSERT_TRUE(std::holds_alternative<VmmExportHandle>(exported->handle));
  EXPECT_TRUE(std::get<VmmExportHandle>(exported->handle).fd.valid());
  EXPECT_EQ(exported->geometry.mapped_size, region.geometry().mapped_size);
  EXPECT_EQ(exported->geometry.device_uuid, region.geometry().device_uuid);

  const MappedGpuRegion imported = backend->import_region(*exported);
  ASSERT_TRUE(imported.valid());
  EXPECT_EQ(imported.geometry().mapped_size, region.geometry().mapped_size);
  EXPECT_EQ(imported.geometry().slot_count, slot_count);
}

TEST(VmmBackendGpuTest, SlotAddressRejectsOutOfRangeIndex)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  const MappedGpuRegion region = backend->create_region(1U << 20, 2);
  ASSERT_TRUE(region.valid());

  EXPECT_NE(region.slot_address(1, 1U << 20), nullptr);
  EXPECT_EQ(region.slot_address(2, 1), nullptr);
  EXPECT_EQ(region.slot_address(0, (1U << 20) + 1), nullptr);
}

TEST(VmmBackendGpuTest, MovedRegionReleasesExactlyOnce)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  MappedGpuRegion region = backend->create_region(1U << 20, 2);
  ASSERT_TRUE(region.valid());

  const MappedGpuRegion moved = std::move(region);
  EXPECT_FALSE(region.valid());  // NOLINT(bugprone-use-after-move)
  EXPECT_TRUE(moved.valid());
}

TEST(VmmBackendGpuTest, ImportRejectsAForeignDevice)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  const MappedGpuRegion region = backend->create_region(1U << 20, 2);
  ASSERT_TRUE(region.valid());

  auto exported = backend->export_for(region, 0);
  ASSERT_TRUE(exported.has_value());
  exported->geometry.device_uuid[0] =
    static_cast<uint8_t>(exported->geometry.device_uuid[0] ^ 0xFFU);

  EXPECT_FALSE(backend->import_region(*exported).valid());
}

TEST(VmmBackendGpuTest, ImportRejectsInconsistentGeometry)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  const MappedGpuRegion region = backend->create_region(1U << 20, 2);
  ASSERT_TRUE(region.valid());

  auto exported = backend->export_for(region, 0);
  ASSERT_TRUE(exported.has_value());
  exported->geometry.slot_count = 1024;  // no longer fits mapped_size

  EXPECT_FALSE(backend->import_region(*exported).valid());
}
