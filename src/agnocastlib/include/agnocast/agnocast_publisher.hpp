#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"
#include "agnocast/internal/gpu_slot_pool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <mqueue.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <mutex>

namespace agnocast
{
class Node;

const void * get_node_base_address(Node * node);

// These are cut out of the class for information hiding.
topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos,
  const bool is_bridge, const std::string & type_name, std::string & out_mq_topic_name);
union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle, /* for CARET */ const std::string & topic_name,
  const std::string & mq_topic_name, const topic_local_id_t publisher_id,
  const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs);
// Releases the caller's own entries that QoS depth no longer retains, without
// publishing anything, and reports their addresses for the caller to free. The
// GPU borrow path uses it to recover slots when it has none left; the host path
// never needs it, because a borrow there cannot fail for want of room.
union ioctl_reclaim_msgs_args reclaim_msgs_core(
  const std::string & topic_name, const topic_local_id_t publisher_id);
uint32_t get_subscription_count_core(const std::string & topic_name);
uint32_t get_intra_subscription_count_core(const std::string & topic_name);
void increment_borrowed_publisher_num();
void decrement_borrowed_publisher_num();

extern int agnocast_fd;
extern "C" uint32_t agnocast_get_borrowed_publisher_num();

// Defined in agnocast_publisher.cpp. Declared here with the same TLS model as
// the definition on purpose: a global-dynamic reference from another
// translation unit would resolve through __tls_get_addr, which can allocate,
// which is the recursion the definition's comment warns about.
extern __attribute__((tls_model("initial-exec"))) thread_local uint32_t borrowed_publisher_num;

namespace internal
{

// Suspends this thread's borrow window for as long as it is alive, so that
// allocations made meanwhile come from the process heap instead of the
// shared-memory mempool.
//
// The window exists so that a message's payload lands in shared memory. But
// anything else allocated while it is open lands there too, and the GPU driver
// allocates host memory of its own on paths the library has to call from inside
// it -- creating a stream, importing a region. Left alone, that bookkeeping
// becomes a permanent resident of the segment every subscriber maps.
//
// Only code that allocates nothing belonging to the message may be wrapped:
// suspending the window around a step that does would put part of a message in
// this process's heap, where a peer resolving it would find nothing. That rules
// out wrapping user callbacks, and makes this an internal tool rather than an
// API.
class SuspendedBorrowWindow
{
public:
  SuspendedBorrowWindow() : saved_(borrowed_publisher_num) { borrowed_publisher_num = 0; }
  ~SuspendedBorrowWindow() { borrowed_publisher_num = saved_; }

  SuspendedBorrowWindow(const SuspendedBorrowWindow &) = delete;
  SuspendedBorrowWindow & operator=(const SuspendedBorrowWindow &) = delete;

private:
  uint32_t saved_;
};

}  // namespace internal

/**
 * @brief Options for configuring an Agnocast publisher.
 */
AGNOCAST_PUBLIC
struct PublisherOptions
{
  /// @deprecated Use the `AGNOCAST_BRIDGE_MODE` environment variable instead.
  bool do_always_ros2_publish = false;
  /// QoS parameter override options (same semantics as rclcpp).
  rclcpp::QosOverridingOptions qos_overriding_options{};
};

/**
 * @brief Role of a publisher with respect to the Agnocast<->ROS bridge.
 *
 * Encodes two properties of a publisher:
 *   - whether it is used by the bridge implementation itself
 *   - whether it should issue an A2R bridge request on construction
 *
 *   | Role            | kmod `is_bridge` | bridge request issued |
 *   |-----------------|------------------|-----------------------|
 *   | Default         | false            | yes (A2R)             |
 *   | AgnocastOnly    | false            | no                    |
 *   | BridgeInternal  | true             | no                    |
 */
enum class PublisherRole : uint8_t {
  /// User-created publisher; issues an A2R bridge request.
  Default,
  /// Used internally; no bridge request is issued.
  /// Not intended for direct use by application code.
  AgnocastOnly,
  /// Used by the bridge implementation itself; marked as bridge in kmod and
  /// issues no bridge request.
  /// Not intended for direct use by application code.
  BridgeInternal,
};

// Base class for Agnocast publishers. This class handles the common operations
// shared with all Agnocast publishers, such as kernel registration and message queue management.
class PublisherBase
{
  void generate_gid();

protected:
  topic_local_id_t id_ = -1;
  uint32_t qos_depth_ = 1;
  // Grown on demand rather than fixed. The first is created on the first GPU
  // borrow.
  mutable std::mutex gpu_pools_mtx_;
  std::vector<std::unique_ptr<internal::GpuSlotPool>> gpu_pools_;

  // One slot per message that may be in flight, plus one being filled. KeepAll
  // reports a depth of 0, which would size a region for a single message. This
  // sizes a region rather than bounding the publisher: more regions are created
  // when subscribers hold messages past it.
  uint32_t gpu_slot_count() const { return std::max(qos_depth_, 1U) + 1; }

  // The first region whose slots fit `capacity` and has one free, or nullptr.
  // Costs nothing but a scan of a list that is at most
  // MAX_GPU_REGION_NUM_PER_PUBLISHER long. Caller holds gpu_pools_mtx_.
  internal::GpuSlotPool * try_acquire_gpu_slot(const size_t capacity, uint32_t & slot_index)
  {
    for (const auto & candidate : gpu_pools_) {
      if (candidate->acquire(capacity, slot_index)) return candidate.get();
    }
    return nullptr;
  }

  // Adds a region sized for `capacity` and takes a slot in it, or nullptr.
  // Caller holds gpu_pools_mtx_ and has checked the region cap.
  internal::GpuSlotPool * grow_gpu_pool(const size_t capacity, uint32_t & slot_index)
  {
    auto grown = internal::GpuSlotPool::create(topic_name_, id_, capacity, gpu_slot_count());
    if (grown == nullptr) {
      RCLCPP_ERROR(
        logger,
        "could not allocate a GPU region for a %zu byte payload on topic '%s' (%zu region(s) "
        "held). See the GPU backend errors above for the cause.",
        capacity, topic_name_.c_str(), gpu_pools_.size());
      return nullptr;
    }

    // Handed to the vector before a slot is taken. push_back allocates, and a
    // throw with a slot already out would destroy a pool that is not idle --
    // which takes ~GpuSlotPool's unmap_local branch rather than destroy, leaving
    // the kmod holding the region and one of this publisher's region slots for
    // the life of the process.
    internal::GpuSlotPool * pool = grown.get();
    gpu_pools_.push_back(std::move(grown));
    if (!pool->acquire(capacity, slot_index)) {
      // A fresh region is sized for this payload and has every slot free, so
      // this cannot happen; retiring it keeps the vector honest if it ever does.
      gpu_pools_.pop_back();
      return nullptr;
    }
    return pool;
  }

  // Drops a region that holds no message and is too small for `capacity`, to
  // make room under MAX_GPU_REGION_NUM_PER_PUBLISHER. The smallest such region
  // goes, since its device memory buys the least. Caller holds gpu_pools_mtx_.
  void retire_idle_gpu_pool(const size_t capacity)
  {
    auto victim = gpu_pools_.end();
    for (auto it = gpu_pools_.begin(); it != gpu_pools_.end(); ++it) {
      if ((*it)->slot_size() >= capacity || !(*it)->is_idle()) continue;
      if (victim == gpu_pools_.end() || (*it)->slot_size() < (*victim)->slot_size()) {
        victim = it;
      }
    }
    if (victim == gpu_pools_.end()) return;

    RCLCPP_INFO(
      logger,
      "releasing an unused %u byte GPU region of topic '%s' to make room for a %zu byte payload",
      (*victim)->slot_size(), topic_name_.c_str(), capacity);
    gpu_pools_.erase(victim);  // the pool's destructor releases the region
  }
  std::string topic_name_;
  // Topic name for the publish-notification MQ (returned by the kmod). Differs from topic_name_
  // only for a domain-bridged/renamed topic, where it is the pair's canonical name so a publisher
  // and a renamed subscriber derive the same MQ name.
  std::string mq_topic_name_;
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> opened_mqs_;
  std::mutex opened_mqs_mtx_;
  rmw_gid_t gid_;

  template <typename NodeT>
  rclcpp::QoS init_base(
    NodeT * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, const PublisherOptions & options, const PublisherRole role);

public:
  PublisherBase() = default;
  virtual ~PublisherBase();

  /**
   * @brief Return the fully-resolved topic name.
   * @return Null-terminated topic name string.
   */
  AGNOCAST_PUBLIC
  const char * get_topic_name() const { return topic_name_.c_str(); }

  /**
   * @brief Return the GID of this publisher, unique across both Agnocast and ROS 2.
   * @return Publisher GID.
   */
  AGNOCAST_PUBLIC
  const rmw_gid_t & get_gid() const { return gid_; }

  /**
   * @brief Return the total subscriber count for this topic (Agnocast + ROS 2 via bridge).
   * @return Total subscriber count.
   */
  AGNOCAST_PUBLIC
  uint32_t get_subscription_count() const { return get_subscription_count_core(topic_name_); }

  /**
   * @brief Return the number of Agnocast intra-process subscribers only (excludes ROS 2).
   * @return Agnocast subscriber count.
   */
  AGNOCAST_PUBLIC
  uint32_t get_intra_subscription_count() const
  {
    return get_intra_subscription_count_core(topic_name_);
  }
};

/**
 * @brief Mirrors `rclcpp::Publisher` semantics: the topic type is supplied as a template
 * type argument `MessageT`. It allocates a memory region for a message using
 * borrow_loaned_message() and publishes it via zero-copy IPC using publish().
 *
 * @tparam MessageT ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class Publisher : public PublisherBase
{
  template <typename NodeT>
  rclcpp::QoS constructor_impl(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options, const PublisherRole role)
  {
    // Gated to message types only — service types pulled in by
    // BasicService<ServiceT> have no rosidl message name. The empty string
    // signals "skip registry" to initialize_publisher.
    std::string type_name;
    if constexpr (rosidl_generator_traits::is_message<MessageT>::value) {
      type_name = rosidl_generator_traits::name<MessageT>();
    } else if constexpr (internal::is_gpu_message_v<MessageT>) {
      // A GPU message type is not a generated ROS type, so it has no name to
      // register and everything keyed on one skips it. Said out loud because the
      // symptom otherwise is a topic that simply never reaches ROS 2.
      RCLCPP_WARN_ONCE(
        logger,
        "topic '%s' carries a GPU message type, which has no ROS type name: it will not appear "
        "with a type in 'ros2 topic info_agnocast' and the Agnocast-ROS 2 bridge cannot carry it. "
        "Only Agnocast subscribers on the same GPU will receive it.",
        topic_name.c_str());
    }

    return this->init_base(node, topic_name, type_name, qos, options, role);
  }

public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;

  Publisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options, const PublisherRole role = PublisherRole::Default)
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, qos, options, role);
    qos_depth_ = static_cast<uint32_t>(actual_qos.depth());

    TRACEPOINT(
      agnocast_publisher_init, static_cast<const void *>(this),
      static_cast<const void *>(
        node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
      topic_name_.c_str(), actual_qos.depth());
  }

  Publisher(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options = PublisherOptions{},
    const PublisherRole role = PublisherRole::Default)
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, qos, options, role);
    qos_depth_ = static_cast<uint32_t>(actual_qos.depth());

    TRACEPOINT(
      agnocast_publisher_init, static_cast<const void *>(this),
      static_cast<const void *>(get_node_base_address(node)), topic_name_.c_str(),
      actual_qos.depth());
  }

  /**
   * @brief Allocate a new default-constructed message in shared memory. The caller must either
   * pass the returned pointer to publish() or let it go out of scope (which frees the memory).
   *
   * @return Owned pointer to the newly allocated message in shared memory.
   */
  AGNOCAST_PUBLIC
  ipc_shared_ptr<MessageT> borrow_loaned_message()
  {
    static_assert(
      !internal::is_gpu_message_v<MessageT>,
      "a message whose payload lives in GPU memory must be borrowed with the capacity overload: "
      "without it no slot is reserved and the payload resolves to nothing");

    increment_borrowed_publisher_num();
    MessageT * ptr = new MessageT();
    return ipc_shared_ptr<MessageT>(ptr, topic_name_.c_str(), id_);
  }

  /**
   * @brief Borrow a message whose payload lives in GPU device memory.
   * @param capacity Payload size in bytes.
   * @return A message whose `data` already refers to a reserved slot, or an
   * empty pointer when no region could be provided. **Unlike the no-argument
   * overload this can fail, and dereferencing the result without checking it is
   * a null dereference.**
   *
   * Reserves a slot in one of this publisher's GPU regions and records the
   * region id and slot index in the message, which is how a subscriber finds the
   * payload. A region is allocated on the first borrow, sized from the QoS
   * depth; later borrows only reserve a slot, so in steady state no GPU
   * allocation happens on the message path.
   *
   * A borrow that finds no free slot -- because the payload outgrew every
   * region, or because subscribers are still holding every message -- allocates
   * another region, bounded by MAX_GPU_REGION_NUM_PER_PUBLISHER, and failing
   * that releases whatever the QoS depth no longer retains. It returns empty
   * only when neither recovers a slot, which costs that one frame and not the
   * topic. See docs/gpu_ipc.md.
   */
  ipc_shared_ptr<MessageT> borrow_loaned_message(const size_t capacity)
  {
    static_assert(
      internal::is_gpu_message_v<MessageT>,
      "the capacity overload is for messages whose payload lives in GPU memory");

    if (capacity == 0 || capacity > internal::kMaxGpuPayloadCapacity) {
      RCLCPP_ERROR(
        logger, "GPU payload capacity %zu is out of range for topic '%s' (max %llu)", capacity,
        topic_name_.c_str(), static_cast<unsigned long long>(internal::kMaxGpuPayloadCapacity));
      return ipc_shared_ptr<MessageT>();
    }

    uint32_t region_id = 0;
    uint32_t slot_index = 0;
    {
      const std::lock_guard<std::mutex> lock(gpu_pools_mtx_);

      // Steps in cost order. A borrow that fails only because slots are in
      // flight takes at most one call into the module: growth, or the release
      // below. Retiring first costs one as well, but only on the path where the
      // payload has outgrown every region this publisher holds, which is where
      // that call already belonged.
      //
      // Host publishing cannot fail for want of room: a borrow allocates from
      // the process mempool, so QoS depth is a retention target the module
      // applies lazily at each publish, never a bound on borrowing. GPU slots
      // are preallocated per region, so the same transient excess -- a
      // subscriber still referencing the oldest entry when the next message is
      // published -- would exhaust them. Growth and release below exist to give
      // that excess the same outcome it has on the host side.
      internal::GpuSlotPool * pool = try_acquire_gpu_slot(capacity, slot_index);

      // At the cap, a region holding no message and too small for this payload
      // can go. That is the only way a publisher whose payloads grow ever gets a
      // region that fits, and it costs no device memory overall.
      if (
        pool == nullptr &&
        gpu_pools_.size() >= static_cast<size_t>(MAX_GPU_REGION_NUM_PER_PUBLISHER)) {
        retire_idle_gpu_pool(capacity);
      }

      // Growth, which is what the mempool does under the same pressure, bounded
      // by MAX_GPU_REGION_NUM_PER_PUBLISHER instead of the mempool size. It also
      // restores the ability to publish, and publishing is what drains the
      // backlog.
      if (
        pool == nullptr &&
        gpu_pools_.size() < static_cast<size_t>(MAX_GPU_REGION_NUM_PER_PUBLISHER)) {
        pool = grow_gpu_pool(capacity, slot_index);
      }

      // At the region cap, or growth failed. Ask the module to release what QoS
      // depth no longer retains and free those messages, which returns their
      // slots. Without this the failure would be permanent rather than a dropped
      // frame: a slot is returned only by destroying its message, the module
      // names releasable messages only when something is published, and with no
      // slot there is nothing to publish.
      if (pool == nullptr) {
        const union ioctl_reclaim_msgs_args reclaimed = reclaim_msgs_core(topic_name_, id_);
        for (uint32_t i = 0; i < reclaimed.ret_released_num; i++) {
          delete reinterpret_cast<MessageT *>(reclaimed.ret_released_addrs[i]);
        }
        pool = try_acquire_gpu_slot(capacity, slot_index);
      }

      if (pool == nullptr) {
        // Which of the two got us here decides what the reader should do about
        // it, and only one of them is about consumers keeping up.
        if (gpu_pools_.size() >= static_cast<size_t>(MAX_GPU_REGION_NUM_PER_PUBLISHER)) {
          RCLCPP_ERROR(
            logger,
            "no GPU slot for a %zu byte payload on topic '%s': this publisher holds the maximum of "
            "%d regions and every slot of each is still held by a message in flight, so none can "
            "be grown or retired. Subscribers are not releasing messages as fast as they are "
            "published; this frame is dropped and publishing resumes once they do.",
            capacity, topic_name_.c_str(), MAX_GPU_REGION_NUM_PER_PUBLISHER);
        } else {
          RCLCPP_ERROR(
            logger,
            "no GPU slot for a %zu byte payload on topic '%s': allocating another region failed "
            "with %zu of %d held, and releasing messages the QoS depth no longer retains freed "
            "none. See the GPU backend errors above for why the allocation failed.",
            capacity, topic_name_.c_str(), gpu_pools_.size(), MAX_GPU_REGION_NUM_PER_PUBLISHER);
        }
        return ipc_shared_ptr<MessageT>();
      }
      region_id = pool->region_id();
    }

    // Everything from here can throw -- the message, its control block and the
    // handle's topic name all allocate, and inside the borrow window that is the
    // shared-memory mempool, which returns null when exhausted. Both the slot and
    // the window are therefore held by guards: a stranded slot is never returned
    // to its pool, which leaves the region unable to reach the idle state it
    // needs to be retired or released, and a stranded window sends every later
    // allocation in the process to the mempool.
    auto slot_guard = rcpputils::make_scope_exit(
      [region_id, slot_index]() noexcept { internal::release_gpu_slot(region_id, slot_index); });
    increment_borrowed_publisher_num();
    auto window_guard =
      rcpputils::make_scope_exit([]() noexcept { decrement_borrowed_publisher_num(); });

    MessageT * ptr = new MessageT();
    // Guarded too: constructing the handle allocates its control block, and a
    // throw there would otherwise leave the message itself in the mempool with
    // nothing owning it -- each retry under the same pressure leaking another.
    auto message_guard = rcpputils::make_scope_exit([ptr]() noexcept { delete ptr; });
    ptr->data = internal::gpu_array<uint8_t>(region_id, slot_index, capacity, id_);
    // The handle owns the message from here, and the message owns the slot.
    ipc_shared_ptr<MessageT> message(ptr, topic_name_.c_str(), id_);
    message_guard.cancel();
    window_guard.cancel();
    slot_guard.cancel();
    return message;
  }

  /**
   * @brief Publish a message via zero-copy IPC. Ownership is transferred: after this call, the
   * passed-in ipc_shared_ptr and all copies sharing its control block are invalidated —
   * dereferencing them calls std::terminate().
   *
   * @param message Message obtained from borrow_loaned_message(). Must be moved in.
   */
  AGNOCAST_PUBLIC
  void publish(ipc_shared_ptr<MessageT> && message)
  {
    if (!message || topic_name_ != message.get_topic_name()) {
      RCLCPP_ERROR(logger, "Invalid message to publish.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    // Capture raw pointer BEFORE invalidation (get() returns nullptr after invalidation).
    const uint64_t msg_virtual_address = reinterpret_cast<uint64_t>(message.get());

    // Invalidate all references sharing this handle's control block.
    // Any remaining copies held elsewhere will fail-fast on dereference.
    message.invalidate_all_references();

    decrement_borrowed_publisher_num();

    union ioctl_publish_msg_args publish_msg_args;
    {
      std::lock_guard<std::mutex> lock(opened_mqs_mtx_);
      publish_msg_args =
        publish_core(this, topic_name_, mq_topic_name_, id_, msg_virtual_address, opened_mqs_);
    }

    for (uint32_t i = 0; i < publish_msg_args.ret_released_num; i++) {
      MessageT * release_ptr = reinterpret_cast<MessageT *>(publish_msg_args.ret_released_addrs[i]);
      // Deleting the message returns its GPU slot along with its host payload:
      // the kmod reports an entry here only once nothing references it, so the
      // slot is free by the same fact that made the memory reclaimable.
      //
      // "Nothing references it" is the kmod's accounting, not a statement about
      // handles: REMOVE_SUBSCRIBER clears a subscriber's bit on every entry of
      // the topic whether or not userspace still holds the message. A node that
      // keeps a received handle past its own subscription therefore reads a slot
      // this publisher may already have refilled. That predates GPU payloads --
      // the same sequence frees host message memory under a live handle -- and
      // is not something this loop can decide.
      delete release_ptr;
    }

    message.reset();
  }
};

/**
 * @brief A type-erased Agnocast publisher.
 *
 * There are four differences between this and Publisher:
 *
 * 1. borrow_loaned_message() takes a size and returns an ipc_shared_ptr<void> that points to a
 *    shared memory block of the requested size.
 * 2. publish() takes a deleter as well as the message. Because the message is type-erased, the
 *    publisher cannot know how to free it, so the caller must provide a deleter.
 * 3. If a user decides not to publish a borrowed message, they must call cancel_message() with a
 *    deleter to free the memory.
 * 4. The constructor is just a thin wrapper around init_base(). As this class is intended for
 *    internal use only, flexibility is prioritized.
 *
 * NOTE: This is the publisher counterpart of Subscription<void>. This is not implemented as
 * Publisher<void> because there is a clear semantic difference between a typed and type-erased
 * publisher as described above.
 */
class TypeErasedPublisher : public PublisherBase
{
public:
  using SharedPtr = std::shared_ptr<TypeErasedPublisher>;

  TypeErasedPublisher(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const PublisherOptions & options, const PublisherRole role);

  TypeErasedPublisher(
    agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const PublisherOptions & options, const PublisherRole role);

  ipc_shared_ptr<void> borrow_loaned_message(size_t size);

  template <typename Deleter>
  void cancel_message(ipc_shared_ptr<void> && message, Deleter && deleter)
  {
    if (!message || topic_name_ != message.get_topic_name()) {
      RCLCPP_ERROR(logger, "Invalid message to cancel.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    void * delete_ptr = message.get();

    message.invalidate_all_references();

    decrement_borrowed_publisher_num();

    deleter(delete_ptr);

    message.reset();
  }

  template <typename Deleter>
  void publish(ipc_shared_ptr<void> && message, Deleter && deleter)
  {
    if (!message || topic_name_ != message.get_topic_name()) {
      RCLCPP_ERROR(logger, "Invalid message to publish.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    const uint64_t msg_virtual_address = reinterpret_cast<uint64_t>(message.get());

    message.invalidate_all_references();

    decrement_borrowed_publisher_num();

    union ioctl_publish_msg_args publish_msg_args;
    {
      std::lock_guard<std::mutex> lock(opened_mqs_mtx_);
      publish_msg_args =
        publish_core(this, topic_name_, mq_topic_name_, id_, msg_virtual_address, opened_mqs_);
    }

    for (uint32_t i = 0; i < publish_msg_args.ret_released_num; i++) {
      void * release_ptr = reinterpret_cast<void *>(publish_msg_args.ret_released_addrs[i]);
      deleter(release_ptr);
    }

    message.reset();
  }
};

/**
 * @brief Mirrors `rclcpp::GenericPublisher` semantics: the topic type is supplied as a
 * runtime string (e.g. "std_msgs/msg/String") rather than a compile-time
 * template argument. The typesupport library is loaded eagerly in the
 * constructor and held for the publisher's lifetime.
 *
 * Messages are passed to `publish()` as `rclcpp::SerializedMessage` objects
 * and are deserialized into Agnocast shared memory within the `publish()` call.
 */
AGNOCAST_PUBLIC
class GenericPublisher : public TypeErasedPublisher
{
  // Keeps the dynamically loaded typesupport and introspection shared libraries
  // (.so) alongside their handles for the lifetime of the publisher.
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * type_support_handle_{nullptr};
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_{nullptr};

  void load_type_support(const std::string & topic_type);

public:
  using SharedPtr = std::shared_ptr<GenericPublisher>;

  AGNOCAST_PUBLIC
  GenericPublisher(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const PublisherOptions & options = PublisherOptions{},
    PublisherRole role = PublisherRole::Default);

  AGNOCAST_PUBLIC
  GenericPublisher(
    agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const PublisherOptions & options = PublisherOptions{},
    PublisherRole role = PublisherRole::Default);

  /**
   * @brief Deserialize a serialized message into Agnocast shared memory and
   * publish it via zero-copy IPC.
   *
   * @param serialized_msg Serialized ROS 2 message to deserialize and publish.
   */
  AGNOCAST_PUBLIC
  void publish(const rclcpp::SerializedMessage & serialized_msg);
};

}  // namespace agnocast
