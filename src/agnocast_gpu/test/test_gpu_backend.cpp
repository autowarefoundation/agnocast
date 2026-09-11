#include "agnocast/internal/gpu_backend.hpp"
#include "vmm_backend.hpp"

#include <cuda_runtime.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <optional>
#include <utility>
#include <vector>

using agnocast::internal::GpuMemoryBackendType;
using agnocast::internal::GpuRegionExport;
using agnocast::internal::GpuRegionGeometry;
using agnocast::internal::is_consistent;
using agnocast::internal::MappedGpuRegion;
using agnocast::internal::UniqueFd;
using agnocast::internal::VmmExportHandle;

namespace
{

constexpr size_t kPayload = 4096;

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

// A region whose "mapping" is host memory, so slot addressing can be exercised
// on a machine with no GPU. Nothing is ever released through this backend.
class NullBackend : public agnocast::internal::GpuMemoryBackend
{
public:
  [[nodiscard]] GpuMemoryBackendType type() const noexcept override
  {
    return GpuMemoryBackendType::Unknown;
  }
  [[nodiscard]] bool is_supported() const noexcept override { return false; }
  [[nodiscard]] MappedGpuRegion create_region(uint32_t, uint32_t) override { return {}; }
  [[nodiscard]] std::optional<GpuRegionExport> export_for(
    const MappedGpuRegion &, agnocast::topic_local_id_t) override
  {
    return std::nullopt;
  }
  [[nodiscard]] MappedGpuRegion import_region(const GpuRegionExport &) override { return {}; }

private:
  void release_region(void *, const GpuRegionGeometry &, uint64_t) noexcept override {}
};

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

// Geometry arrives from another process, so this is the check that stands
// between a corrupt message and an out-of-bounds device address.
TEST(GpuRegionGeometryTest, RejectsSlotsThatDoNotFitTheMapping)
{
  EXPECT_TRUE(is_consistent(GpuRegionGeometry{2048, 4, 8192, {}}));
  EXPECT_TRUE(is_consistent(GpuRegionGeometry{2048, 4, 16384, {}}));
  EXPECT_FALSE(is_consistent(GpuRegionGeometry{2048, 4, 8191, {}}));
  EXPECT_FALSE(is_consistent(GpuRegionGeometry{0, 4, 8192, {}}));
  EXPECT_FALSE(is_consistent(GpuRegionGeometry{2048, 0, 8192, {}}));
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

// The slot index arrives from another process, so out-of-range indices and
// oversized payloads must resolve to nothing rather than to a wild address.
TEST(MappedGpuRegionTest, SlotAddressBoundsIndexAndPayload)
{
  NullBackend backend;
  std::array<uint8_t, 8192> buffer{};
  const MappedGpuRegion region(backend, buffer.data(), GpuRegionGeometry{2048, 4, 8192, {}}, 0);

  EXPECT_EQ(region.slot_address(3, 2048), buffer.data() + 6144);
  EXPECT_EQ(region.slot_address(4, 1), nullptr);
  EXPECT_EQ(region.slot_address(0, 2049), nullptr);
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

// An importer is granted read access only, so this checks the grant is both
// accepted by the driver and sufficient: the bytes the exporter wrote must be
// readable through the imported mapping. A write through it is deliberately not
// tested, since a device-side access violation poisons the context for the rest
// of the run.
TEST(VmmBackendGpuTest, ImportedMappingReadsWhatTheExporterWrote)
{
  auto * backend = gpu_backend_or_skip();
  if (backend == nullptr) GTEST_SKIP() << "no VMM-capable GPU";

  constexpr uint32_t kSlotSize = 1U << 20;
  const MappedGpuRegion region = backend->create_region(kSlotSize, 2);
  ASSERT_TRUE(region.valid());

  std::vector<uint8_t> pattern(kPayload);
  for (size_t i = 0; i < pattern.size(); i++) pattern[i] = static_cast<uint8_t>(i % 251);
  ASSERT_EQ(
    cudaMemcpy(region.slot_address(1, kPayload), pattern.data(), kPayload, cudaMemcpyHostToDevice),
    cudaSuccess);

  const auto exported = backend->export_for(region, 0);
  ASSERT_TRUE(exported.has_value());
  const MappedGpuRegion imported = backend->import_region(*exported);
  ASSERT_TRUE(imported.valid());

  // A distinct mapping of the same memory, so the addresses differ but the bytes
  // do not.
  EXPECT_NE(imported.slot_address(1, kPayload), region.slot_address(1, kPayload));

  std::vector<uint8_t> readback(kPayload, 0);
  ASSERT_EQ(
    cudaMemcpy(
      readback.data(), imported.slot_address(1, kPayload), kPayload, cudaMemcpyDeviceToHost),
    cudaSuccess);
  EXPECT_EQ(readback, pattern);

  // The offset is honoured: a slot the exporter did not write must not match.
  std::vector<uint8_t> other(kPayload, 0);
  ASSERT_EQ(
    cudaMemcpy(other.data(), imported.slot_address(0, kPayload), kPayload, cudaMemcpyDeviceToHost),
    cudaSuccess);
  EXPECT_NE(other, pattern);
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
