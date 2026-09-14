#include "agnocast/agnocast_publisher.hpp"

#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/internal/type_registry_writer.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/detail/qos_parameters.hpp"

#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <sys/types.h>

#include <algorithm>
#include <new>

namespace agnocast
{

// Keep the initial-exec TLS model here: it avoids the following infinite recursion that causes a
// SIGSEGV:
// 1. heaphook malloc() is called.
// 2. agnocast_get_borrowed_publisher_num() is called and accesses a thread_local variable.
// 3. __tls_get_addr() is called to resolve the address.
// 4. _dl_resize_dtv() is called to resize the DTV region. This occurs when new .so libraries are
//    loaded via dlopen() and the number of managed TLS variables increases.
// 5. _dl_resize_dtv() calls malloc(), which loops back to step 1.
__attribute__((tls_model("initial-exec"))) thread_local uint32_t borrowed_publisher_num = 0;

extern "C" uint32_t agnocast_get_borrowed_publisher_num()
{
  return borrowed_publisher_num;
}

void increment_borrowed_publisher_num()
{
  borrowed_publisher_num++;
}

void decrement_borrowed_publisher_num()
{
  if (borrowed_publisher_num == 0) {
    RCLCPP_ERROR(
      logger,
      "The number of publish() called exceeds the number of borrow_loaned_message() called.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  borrowed_publisher_num--;
}

topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos,
  const bool is_bridge, const std::string & type_name)
{
  validate_ld_preload();

  // Announce to the per-IPC-namespace discovery agent before the kmod call so
  // the registry line is in place whenever a later snapshot sees the
  // ioctl-side endpoint. Empty `type_name` (e.g. service types) skips this.
  if (!type_name.empty()) {
    internal::TypeRegistryWriter::instance().register_type(topic_name, type_name, "pub", node_name);
  }

  union ioctl_add_publisher_args pub_args = {};
  pub_args.topic_name = {topic_name.c_str(), topic_name.size()};
  pub_args.node_name = {node_name.c_str(), node_name.size()};
  pub_args.qos_depth = qos.depth();
  pub_args.qos_is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  pub_args.is_bridge = is_bridge;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, &pub_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PUBLISHER_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return pub_args.ret_id;
}

union ioctl_reclaim_msgs_args reclaim_msgs_core(
  const std::string & topic_name, const topic_local_id_t publisher_id)
{
  union ioctl_reclaim_msgs_args reclaim_args = {};
  reclaim_args.topic_name = {topic_name.c_str(), topic_name.size()};
  reclaim_args.publisher_id = publisher_id;

  if (ioctl(agnocast_fd, AGNOCAST_RECLAIM_MSGS_CMD, &reclaim_args) < 0) {
    // Reported rather than fatal, unlike the publish path: the caller is trying
    // to recover from having no slot, and failing to means one dropped frame.
    RCLCPP_ERROR(
      logger, "AGNOCAST_RECLAIM_MSGS_CMD failed for topic '%s': %s", topic_name.c_str(),
      strerror(errno));
    reclaim_args.ret_released_num = 0;
  }
  return reclaim_args;
}

union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle /* for CARET */, const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address)
{
  union ioctl_publish_msg_args publish_msg_args = {};
  publish_msg_args.topic_name = {topic_name.c_str(), topic_name.size()};
  publish_msg_args.publisher_id = publisher_id;
  publish_msg_args.msg_virtual_address = msg_virtual_address;

  if (ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, &publish_msg_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_PUBLISH_MSG_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  TRACEPOINT(agnocast_publish, publisher_handle, publish_msg_args.ret_entry_id);

  return publish_msg_args;
}

uint32_t get_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  uint32_t inter_count = args.ret_other_process_subscriber_num;
  // If an A2R bridge exists, exclude the agnocast subscriber created by the bridge. The bridge
  // runs in a forked process, so it is never part of the same-process count.
  if (args.ret_a2r_bridge_exist && inter_count > 0) {
    inter_count--;
  }

  uint32_t ros2_count = args.ret_ros2_subscriber_num;
  // If an R2A bridge exists, exclude the ROS 2 subscriber created by the bridge
  if (args.ret_r2a_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  return inter_count + args.ret_same_process_subscriber_num + ros2_count;
}

uint32_t get_same_process_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args get_subscriber_count_args = {};
  get_subscriber_count_args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &get_subscriber_count_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return get_subscriber_count_args.ret_same_process_subscriber_num;
}

template <typename NodeT>
void PublisherBase::init_base(
  NodeT * node, const std::string & topic_name, const std::string & type_name,
  const rclcpp::QoS & qos, const PublisherOptions & options, const PublisherRole role)
{
  if (options.do_always_ros2_publish) {
    RCLCPP_ERROR(
      logger,
      "The 'do_always_ros2_publish' option is deprecated. "
      "Use the AGNOCAST_BRIDGE_MODE environment variable instead.");
  }

  topic_name_ = node->get_node_topics_interface()->resolve_topic_name(topic_name);

  auto node_parameters = node->get_node_parameters_interface();
  actual_qos_ = !options.qos_overriding_options.get_policy_kinds().empty()
                  ? rclcpp::detail::declare_qos_parameters(
                      options.qos_overriding_options, node_parameters, topic_name_, qos,
                      rclcpp::detail::PublisherQosParametersTraits{})
                  : qos;

  validate_publisher_qos(actual_qos_);

  const bool is_bridge = (role == PublisherRole::BridgeInternal);
  const std::string node_name = node->get_fully_qualified_name();
  id_ = initialize_publisher(topic_name_, node_name, actual_qos_, is_bridge, type_name);
  generate_gid();

  if (role == PublisherRole::Default) {
    if (!type_name.empty()) {
      register_pubsub_bridge_by_type_name(
        topic_name_, id_, type_name, BridgeDirection::AGNOCAST_TO_ROS2);
    } else {
      RCLCPP_ERROR(
        logger,
        "A2R bridge registration is skipped because the type_name is empty (topic: '%s'). "
        "Please make sure to specify the valid message type in normal use case.",
        topic_name_.c_str());
    }
  }
}

template void PublisherBase::init_base<rclcpp::Node>(
  rclcpp::Node *, const std::string &, const std::string &, const rclcpp::QoS &,
  const PublisherOptions &, PublisherRole);
template void PublisherBase::init_base<agnocast::Node>(
  agnocast::Node *, const std::string &, const std::string &, const rclcpp::QoS &,
  const PublisherOptions &, PublisherRole);

namespace
{

// One slot per message that may be in flight, plus one being filled. KeepAll
// reports a depth of 0, which would size a region for a single message. This is
// the bound on messages in flight, not merely a starting size: regions are added
// when the payload outgrows them, never to hold more messages at once.
uint32_t gpu_slot_count(const uint32_t qos_depth)
{
  return std::max(qos_depth, 1U) + 1;
}

}  // namespace

bool PublisherBase::has_gpu_region_fitting(const size_t capacity) const
{
  return std::any_of(gpu_pools_.begin(), gpu_pools_.end(), [capacity](const auto & pool) {
    return pool->slot_size() >= capacity;
  });
}

internal::GpuSlotPool * PublisherBase::acquire_gpu_slot(
  const size_t capacity, uint32_t & slot_index)
{
  const std::lock_guard<std::mutex> lock(gpu_pools_mtx_);

  for (const auto & candidate : gpu_pools_) {
    if (candidate->acquire(capacity, slot_index)) return candidate.get();
  }

  // A region fits but has no free slot: the publisher already has as many
  // messages in flight as its QoS depth allows, and growing would overrule that
  // depth by taking more of a resource the whole machine shares. Growth is for
  // payloads that have outgrown every region, and nothing else. See
  // docs/gpu_ipc.md.
  if (has_gpu_region_fitting(capacity)) return nullptr;

  // At the cap, a region holding no message can go -- every one of them is too
  // small for this payload, or the test above would have returned. The smallest
  // goes, since its device memory buys the least. That is the only way a
  // publisher whose payloads grow ever gets a region that fits, and it costs no
  // device memory overall.
  if (gpu_pools_.size() >= static_cast<size_t>(MAX_GPU_REGION_NUM_PER_PUBLISHER)) {
    auto victim = gpu_pools_.end();
    for (auto it = gpu_pools_.begin(); it != gpu_pools_.end(); ++it) {
      if (!(*it)->is_idle()) continue;
      if (victim == gpu_pools_.end() || (*it)->slot_size() < (*victim)->slot_size()) victim = it;
    }
    if (victim == gpu_pools_.end()) return nullptr;

    RCLCPP_INFO(
      logger,
      "releasing an unused %u byte GPU region of topic '%s' to make room for a %zu byte payload",
      (*victim)->slot_size(), topic_name_.c_str(), capacity);
    gpu_pools_.erase(victim);  // the pool's destructor releases the region
  }

  auto grown = internal::GpuSlotPool::create(
    topic_name_, id_, capacity, gpu_slot_count(static_cast<uint32_t>(actual_qos_.depth())));
  if (grown == nullptr) {
    RCLCPP_ERROR(
      logger,
      "could not allocate a GPU region for a %zu byte payload on topic '%s' (%zu region(s) held). "
      "See the GPU backend errors above for the cause.",
      capacity, topic_name_.c_str(), gpu_pools_.size());
    return nullptr;
  }

  // Handed to the vector before a slot is taken. push_back allocates, and a
  // throw with a slot already out would destroy a pool that is not idle, leaving
  // the kmod holding the region and one of this publisher's region slots for the
  // life of the process.
  internal::GpuSlotPool * pool = grown.get();
  gpu_pools_.push_back(std::move(grown));
  if (!pool->acquire(capacity, slot_index)) {
    // A fresh region is sized for this payload and has every slot free, so this
    // cannot happen; retiring it keeps the vector honest if it ever does.
    gpu_pools_.pop_back();
    return nullptr;
  }
  return pool;
}

void PublisherBase::report_gpu_borrow_failure(const size_t capacity) const
{
  const std::lock_guard<std::mutex> lock(gpu_pools_mtx_);
  if (has_gpu_region_fitting(capacity)) {
    RCLCPP_ERROR(
      logger,
      "no free GPU slot for topic '%s': every slot fitting a %zu byte payload is held by a message "
      "in flight, and the QoS depth released none. This frame is dropped; raise the depth if "
      "subscribers are expected to lag.",
      topic_name_.c_str(), capacity);
    return;
  }
  RCLCPP_ERROR(
    logger,
    "no GPU region for topic '%s': a %zu byte payload exceeds every slot this publisher has, and "
    "another region could not be allocated (%zu of %d held). A stable payload size needs far fewer "
    "regions.",
    topic_name_.c_str(), capacity, gpu_pools_.size(), MAX_GPU_REGION_NUM_PER_PUBLISHER);
}

void PublisherBase::generate_gid()
{
  constexpr size_t kPidOffset = 2;
  constexpr size_t kHashOffset = 6;
  constexpr size_t kHashSize = 6;
  constexpr size_t kPubIdOffset = 12;

  std::memset(static_cast<void *>(&gid_.data[0]), 0, RMW_GID_STORAGE_SIZE);

  // [0-1]: Agnocast identifier
  gid_.data[0] = 'A';
  gid_.data[1] = 'G';

  // [2-5]: Process ID
  const auto pid = static_cast<uint32_t>(getpid());
  std::memcpy(static_cast<void *>(&gid_.data[kPidOffset]), &pid, sizeof(pid));

  // [6-11]: topic_name hash (upper 6 bytes)
  const uint64_t topic_hash = static_cast<uint64_t>(std::hash<std::string>{}(topic_name_));
  std::memcpy(static_cast<void *>(&gid_.data[kHashOffset]), &topic_hash, kHashSize);

  // [12-15]: publisher id
  std::memcpy(static_cast<void *>(&gid_.data[kPubIdOffset]), &id_, sizeof(id_));

  // [16-23]: reserved

  gid_.implementation_identifier = "agnocast";
}

PublisherBase::~PublisherBase()
{
  // Before REMOVE_PUBLISHER below, and before the members are destroyed: the
  // kmod keys region removal on the publisher, so a region released afterwards
  // would be refused -- the publisher's own teardown frees its regions once its
  // last entry is gone. Releasing them here means a region this publisher has
  // proven idle is handed back while it can still be named.
  gpu_pools_.clear();

  if (id_ >= 0) {
    // NOTE: When a publisher is destroyed, subscribers should unmap its memory, but this is not yet
    // implemented. Since multiple publishers in the same process share a mempool, process-level
    // reference counting in kmod is needed. Leaving memory mapped causes no functional issues, so
    // this is left as future work.
    struct ioctl_remove_publisher_args remove_publisher_args
    {
    };
    remove_publisher_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    remove_publisher_args.publisher_id = id_;
    if (ioctl(agnocast_fd, AGNOCAST_REMOVE_PUBLISHER_CMD, &remove_publisher_args) < 0) {
      RCLCPP_WARN(logger, "Failed to remove publisher (id=%d) from kernel.", id_);
    }
  }
}

TypeErasedPublisher::TypeErasedPublisher(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const agnocast::PublisherOptions & options, const PublisherRole role)
{
  this->init_base(node, topic_name, topic_type, qos, options, role);

  TRACEPOINT(
    agnocast_publisher_init, static_cast<const void *>(this),
    static_cast<const void *>(node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
    topic_name_.c_str(), actual_qos_.depth());
}

TypeErasedPublisher::TypeErasedPublisher(
  agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const agnocast::PublisherOptions & options, const PublisherRole role)
{
  this->init_base(node, topic_name, topic_type, qos, options, role);

  TRACEPOINT(
    agnocast_publisher_init, static_cast<const void *>(this),
    static_cast<const void *>(get_node_base_address(node)), topic_name_.c_str(),
    actual_qos_.depth());
}

ipc_shared_ptr<void> TypeErasedPublisher::borrow_loaned_message(size_t size)
{
  increment_borrowed_publisher_num();
  void * ptr = ::operator new(size);
  return ipc_shared_ptr<void>(ptr, topic_name_, id_);
}

void GenericPublisher::load_type_support(const std::string & topic_type)
{
  // The typesupport functions may throw exceptions if the shared libraries
  // fail to load or an invalid message type name is provided. These
  // exceptions are not handled here, causing the constructor to exit
  // immediately.
  ts_lib_ = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  ts_lib_introspection_ =
    rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_introspection_cpp");
#if RCLCPP_VERSION_MAJOR >= 28
  type_support_handle_ =
    rclcpp::get_message_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib_);
  const rosidl_message_type_support_t * introspection_handle =
    rclcpp::get_message_typesupport_handle(
      topic_type, "rosidl_typesupport_introspection_cpp", *ts_lib_introspection_);
#else
  type_support_handle_ =
    rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib_);
  const rosidl_message_type_support_t * introspection_handle = rclcpp::get_typesupport_handle(
    topic_type, "rosidl_typesupport_introspection_cpp", *ts_lib_introspection_);
#endif
  members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_handle->data);
}

GenericPublisher::GenericPublisher(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const PublisherOptions & options, PublisherRole role)
: TypeErasedPublisher(node, topic_name, topic_type, qos, options, role)
{
  load_type_support(topic_type);
}

GenericPublisher::GenericPublisher(
  agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const PublisherOptions & options, PublisherRole role)
: TypeErasedPublisher(node, topic_name, topic_type, qos, options, role)
{
  load_type_support(topic_type);
}

void GenericPublisher::publish(const rclcpp::SerializedMessage & serialized_msg)
{
  // Mirror the pre-conditions checked by rclcpp::SerializationBase::deserialize_message
  // to avoid a SIGSEGV inside rmw_deserialize on malformed input.
  if (serialized_msg.capacity() == 0) {
    RCLCPP_ERROR(
      logger,
      "GenericPublisher::publish: serialized message has capacity of zero; dropping message");
    return;
  }
  if (serialized_msg.size() == 0) {
    RCLCPP_ERROR(
      logger, "GenericPublisher::publish: serialized message has size of zero; dropping message");
    return;
  }

  ipc_shared_ptr<void> message = borrow_loaned_message(members_->size_of_);
  void * ptr = message.get();

  // Invoke the constructor of the message type at ptr.
  // Type-specific initialization is unnecessary because the message object
  // will be immediately populated with data by rmw_deserialize.
  // Perform minimal initialization only.
  members_->init_function(ptr, rosidl_runtime_cpp::MessageInitialization::SKIP);

  const rmw_ret_t ret =
    rmw_deserialize(&serialized_msg.get_rcl_serialized_message(), type_support_handle_, ptr);

  auto deleter = [this](void * release_ptr) {
    members_->fini_function(release_ptr);
    ::operator delete(release_ptr);
  };

  if (ret != RMW_RET_OK) {
    cancel_message(std::move(message), deleter);
    RCLCPP_ERROR(
      logger, "rmw_deserialize failed in GenericPublisher (rmw_ret=%d); dropping message",
      static_cast<int>(ret));
    return;
  }

  TypeErasedPublisher::publish(std::move(message), deleter);
}

}  // namespace agnocast
