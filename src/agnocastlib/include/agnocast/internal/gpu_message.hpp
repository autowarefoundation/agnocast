#pragma once

// What a message carries, and how a process turns it back into a device
// address. Deliberately free of the backend interface so that including
// agnocast.hpp does not drag the GPU SPI into every translation unit.
// docs/gpu_memory.md explains the addressing scheme.

#include "agnocast/agnocast_ioctl.hpp"

#include <cstdint>
#include <limits>
#include <string_view>
#include <type_traits>
#include <utility>

namespace agnocast::internal
{

// Names a region to the kmod. `subscriber_id` is the receiving endpoint, which
// the kmod checks against the calling process before handing out a descriptor.
// `region_id` is the id read out of the message being resolved, or 0 for "any",
// which is what a caller that has not yet seen a message asks for.
struct GpuRegionRef
{
  std::string_view topic_name;
  topic_local_id_t publisher_id = -1;
  topic_local_id_t subscriber_id = -1;
  uint32_t region_id = 0;
};

// Every region this process has mapped, keyed by the id the kmod assigned it.
//
// A mapping lives as long as anything in this process can still refer to it.
// For a region this process created, that is until its slot pool is destroyed
// with no slot outstanding. An imported one lives until the kmod no longer knows
// the publisher that exported it: while this process holds a received message,
// it holds a reference on that message's kmod entry, so the publisher cannot
// have been forgotten; once it has been, no handle to any of its messages exists
// here and none can appear, because region ids are never reused. Without that
// reclamation a subscriber would accumulate a mapping -- and a driver reference
// on the memory behind it -- for every region of every publisher it ever saw.
class GpuRegionRegistry
{
public:
  static GpuRegionRegistry & instance();

  [[nodiscard]] bool is_mapped(uint32_t region_id) const;

  // nullptr when the region is not mapped here, or the slot does not hold
  // `bytes`. Hot path: called for every access to a message's payload.
  [[nodiscard]] void * resolve(uint32_t region_id, uint32_t slot_index, uint64_t bytes) const;

  // Subscriber side. Idempotent: returns immediately when the region the
  // message refers to is already mapped. Also the point at which imported
  // regions nothing can refer to any more are released, so that growth in one
  // publisher's regions pays for reclaiming a departed publisher's.
  [[nodiscard]] bool ensure_mapped(const GpuRegionRef & ref);

  // Publisher side. Allocates the region and hands its liveness reference to the
  // kmod, which holds it so the memory outlives this process. Returns the id the
  // kmod assigned, or 0 on failure.
  [[nodiscard]] uint32_t create(
    std::string_view topic_name, topic_local_id_t publisher_id, uint32_t slot_size,
    uint32_t slot_count);

  // Publisher side. Releases the mapping and the kmod's reference. The caller
  // must know that no message refers to the region: a subscriber that already
  // imported it keeps its own reference and reads on, but one that has not will
  // no longer be able to.
  void destroy(std::string_view topic_name, topic_local_id_t publisher_id, uint32_t region_id);

  // Drops only this process's mapping, leaving the kmod's reference in place for
  // peers. What a publisher does with a region it will never write again but
  // cannot declare unreferenced -- the kmod releases its own share once the
  // messages still in flight drain.
  void unmap_local(uint32_t region_id);

private:
  GpuRegionRegistry() = default;
};

// nullptr when the region is not mapped in this process, or when the slot does
// not hold `bytes`.
[[nodiscard]] void * resolve_gpu_slot(
  uint32_t region_id, uint32_t slot_index, uint64_t bytes) noexcept;

// Returns a slot to the pool that owns it. A no-op in a process that does not
// own the region, so a subscriber dropping a handle frees nothing.
void release_gpu_slot(uint32_t region_id, uint32_t slot_index) noexcept;

// Records that one received message handle refers to a region, so it is not
// released while that handle lives. Paired across the lifetime of a
// subscriber-side control block; both are no-ops for region id 0.
void ref_gpu_region(uint32_t region_id) noexcept;
void unref_gpu_region(uint32_t region_id) noexcept;

// Marks a message whose payload lives in GPU device memory, so the publisher
// knows that borrowing must also reserve a slot.
struct gpu_message_tag
{
};

template <typename T>
inline constexpr bool is_gpu_message_v = std::is_base_of_v<gpu_message_tag, std::remove_const_t<T>>;

// A device buffer as it appears inside a message: the publisher fills one in
// when it borrows, and a subscriber resolves it back to an address of its own.
// It lives in host shared memory, so it holds only values that mean the same
// thing in every process -- never a device address.
template <typename T>
class gpu_array
{
public:
  gpu_array() = default;
  gpu_array(uint32_t region_id, uint32_t slot_index, uint64_t count, topic_local_id_t publisher_id)
  : region_id_(region_id), slot_index_(slot_index), count_(count), publisher_id_(publisher_id)
  {
  }

  // The slot returns when the message is destroyed, which is the rule the host
  // payload already follows: publish() hands released messages back for
  // deletion, and a borrow dropped without publishing deletes its message too.
  ~gpu_array() { release(); }

  // Move-only: two handles to one slot would release it twice.
  gpu_array(const gpu_array &) = delete;
  gpu_array & operator=(const gpu_array &) = delete;

  gpu_array(gpu_array && other) noexcept { swap(other); }
  gpu_array & operator=(gpu_array && other) noexcept
  {
    if (this != &other) {
      release();
      swap(other);
    }
    return *this;
  }

  // Const-qualified on the message, const on the way out. A subscriber holds an
  // ipc_shared_ptr<const MessageT>, so this is the overload it reaches, and its
  // mapping of the region is read-only: a device pointer that let it write would
  // fault rather than corrupt, but the type is what says so at the call site.
  [[nodiscard]] const T * get() const noexcept
  {
    return static_cast<const T *>(resolve_gpu_slot(region_id_, slot_index_, byte_count()));
  }

  // The publisher's own message is non-const, and its mapping is writable.
  [[nodiscard]] T * get() noexcept
  {
    return static_cast<T *>(resolve_gpu_slot(region_id_, slot_index_, byte_count()));
  }

  [[nodiscard]] uint64_t size() const noexcept { return count_; }
  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }
  [[nodiscard]] uint32_t slot_index() const noexcept { return slot_index_; }
  [[nodiscard]] bool valid() const noexcept { return region_id_ != 0; }

  // Scaffolding. The kmod keys a region on its publisher, and the receive path
  // does not yet hand a subscriber the publisher of the message it holds. Once
  // it does, the region id alone suffices and this goes away.
  [[nodiscard]] topic_local_id_t publisher_id() const noexcept { return publisher_id_; }

private:
  // count_ is a value a peer wrote into shared memory, so the product is
  // computed with a guard: an overflowing one would wrap to a small number and
  // pass the slot bound. The saturated value fails it instead.
  [[nodiscard]] uint64_t byte_count() const noexcept
  {
    constexpr uint64_t limit = std::numeric_limits<uint64_t>::max();
    if (sizeof(T) > 1 && count_ > limit / sizeof(T)) return limit;
    return count_ * sizeof(T);
  }

  void release() noexcept
  {
    if (region_id_ != 0) {
      release_gpu_slot(region_id_, slot_index_);
      region_id_ = 0;
    }
  }

  void swap(gpu_array & other) noexcept
  {
    std::swap(region_id_, other.region_id_);
    std::swap(slot_index_, other.slot_index_);
    std::swap(count_, other.count_);
    std::swap(publisher_id_, other.publisher_id_);
  }

  // Zero is never a valid region id, so a default-constructed handle resolves to
  // nothing rather than to region 0.
  uint32_t region_id_ = 0;
  uint32_t slot_index_ = 0;
  uint64_t count_ = 0;
  topic_local_id_t publisher_id_ = -1;
};

}  // namespace agnocast::internal
