#include "agnocast/internal/gpu_slot_pool.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"
#include "rcpputils/scope_exit.hpp"

#include <cstdio>
#include <limits>
#include <unordered_map>

namespace agnocast::internal
{

namespace
{

// Leaked deliberately: a message destructor can reach this after static
// destruction would have run.
std::mutex & pool_table_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, GpuSlotPool *> & pool_table()
{
  static auto * pools =
    new std::unordered_map<uint32_t, GpuSlotPool *>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *pools;
}

// The alignment cudaMalloc guarantees, so that a slot address is as aligned as
// a device pointer the user would have allocated themselves.
constexpr uint32_t kMinSlotSize = 256;

}  // namespace

uint32_t gpu_slot_size_for(const uint64_t capacity) noexcept
{
  if (capacity <= kMinSlotSize) return kMinSlotSize;

  // Above 2 GiB the next power of two does not fit in a slot size, so the
  // capacity stands close to as asked -- but still rounded up to the floor. Slot
  // k begins at k * slot_size, so an unrounded size would put every slot after
  // the first at an arbitrary offset and lose the alignment the floor exists
  // for. A capacity too large to round is refused by the caller, so the rounding
  // below cannot overflow.
  if (capacity > (1ULL << 31)) {
    return static_cast<uint32_t>((capacity + kMinSlotSize - 1) & ~(uint64_t{kMinSlotSize} - 1));
  }

  uint32_t rounded = kMinSlotSize;
  while (rounded < capacity) rounded *= 2;
  return rounded;
}

GpuSlotPool::GpuSlotPool(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint32_t region_id,
  const uint32_t slot_size, const uint32_t slot_count)
: topic_name_(topic_name),
  publisher_id_(publisher_id),
  region_id_(region_id),
  slot_size_(slot_size),
  slot_count_(slot_count)
{
  free_slots_.reserve(slot_count);
  for (uint32_t i = slot_count; i > 0; i--) {
    free_slots_.push_back(i - 1);
  }
  // Sized once here, so neither container can reallocate later -- a release can
  // happen between a borrow and its publish, where an allocation would come out
  // of the shared-memory mempool.
  slot_is_out_.assign(slot_count, false);
}

std::unique_ptr<GpuSlotPool> GpuSlotPool::create(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint64_t capacity,
  const uint32_t slot_count)
{
  // None of what this function allocates belongs to a message -- the pool, its
  // free list, the registry entry -- so none of it should land in the mempool if
  // a borrow happens to be open on this thread. The registry suspends the window
  // for its own driver calls; this covers the rest.
  const SuspendedBorrowWindow suspended;

  const uint32_t slot_size = gpu_slot_size_for(capacity);

  const uint32_t region_id =
    GpuRegionRegistry::instance().create(topic_name, publisher_id, slot_size, slot_count);
  if (region_id == 0) return nullptr;

  // The region now exists -- device memory allocated, its liveness reference
  // handed to the kmod, its mapping in this process -- but nothing owns it yet,
  // and constructing the owner allocates. A throw here would strand all of that
  // for the life of the process, along with one of the publisher's
  // MAX_GPU_REGION_NUM_PER_PUBLISHER slots, with the id needed to release it
  // known to nobody. Armed only until the pool exists, because from then on
  // ~GpuSlotPool is what releases the region.
  auto unowned = rcpputils::make_scope_exit([topic_name, publisher_id, region_id]() noexcept {
    try {
      GpuRegionRegistry::instance().destroy(topic_name, publisher_id, region_id);
    } catch (...) {  // NOLINT(bugprone-empty-catch)
      // Runs while another exception unwinds, where terminating over a failed
      // log line would be worse than the leak the line would have reported.
    }
  });

  auto pool = std::unique_ptr<GpuSlotPool>(
    new GpuSlotPool(topic_name, publisher_id, region_id, slot_size, slot_count));
  unowned.cancel();

  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  pool_table()[region_id] = pool.get();
  return pool;
}

GpuSlotPool::~GpuSlotPool()
{
  {
    // Unregistered first, and under the same lock release_gpu_slot holds across
    // its call, so an in-flight release finishes before the check below and none
    // starts after it.
    const std::lock_guard<std::mutex> lock(pool_table_mutex());
    pool_table().erase(region_id_);
  }

  // Every slot free means nothing refers to this region: the kmod released each
  // entry, which is what let the publisher delete the message holding the slot.
  if (is_idle()) {
    GpuRegionRegistry::instance().destroy(topic_name_, publisher_id_, region_id_);
    return;
  }

  // A slot is still out, so the kmod must keep the region for whatever is still
  // in flight -- but this process will never write it again, and the messages
  // holding those slots may never be deleted at all: the kmod reports released
  // addresses only from publish(), so anything QoS was still retaining at
  // teardown is never handed back. Waiting for idleness would therefore pin this
  // mapping, and a share of device memory, for the life of the process.
  GpuRegionRegistry::instance().unmap_local(region_id_);
}

bool GpuSlotPool::is_idle() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size() == slot_count_;
}

bool GpuSlotPool::acquire(const uint64_t capacity, uint32_t & out_slot_index)
{
  if (capacity > slot_size_) return false;

  const std::lock_guard<std::mutex> lock(mutex_);
  if (free_slots_.empty()) return false;
  out_slot_index = free_slots_.back();
  free_slots_.pop_back();
  slot_is_out_[out_slot_index] = true;
  return true;
}

void GpuSlotPool::release(const uint32_t slot_index)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  // Bounded on both sides rather than trusted, because the index comes from a
  // handle in shared memory. Releasing a slot that is not out would hand one
  // slot to two messages, and would also make is_idle() true while a message
  // still points into the region, which is the predicate the publisher uses to
  // decide it may release the region altogether.
  if (slot_index >= slot_count_ || !slot_is_out_[slot_index]) {
    RCLCPP_ERROR(
      logger, "ignoring the release of GPU slot %u of region %u: not an outstanding slot",
      slot_index, region_id_);
    return;
  }
  slot_is_out_[slot_index] = false;
  free_slots_.push_back(slot_index);
}

size_t GpuSlotPool::available() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size();
}

// noexcept because the caller is a message destructor, while locking can throw
// and so can the RCLCPP_ERROR that release() reaches on a corrupted slot index --
// terminating there would be worse than the index it was reporting. A
// function-try-block, as on UniqueFd::reset and VmmBackend::release_region.
void release_gpu_slot(const uint32_t region_id, const uint32_t slot_index) noexcept
try {
  // The lock is held across the release, not just the lookup: the message being
  // destroyed here may be the last one outliving its publisher, so dropping it
  // first would let ~GpuSlotPool free the pool between the two. Nothing takes
  // the table lock while holding a pool's own lock, so this is the only nesting.
  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  const auto it = pool_table().find(region_id);
  if (it == pool_table().end()) return;  // not ours: a peer's region
  it->second->release(slot_index);
} catch (...) {
  // The slot never returns to its pool, so the region can no longer reach the
  // idle state it needs to be retired -- a bounded leak, and the safe direction.
  std::fprintf(
    stderr, "[agnocast] failed to release GPU slot %u of region %u\n", slot_index, region_id);
}

}  // namespace agnocast::internal
