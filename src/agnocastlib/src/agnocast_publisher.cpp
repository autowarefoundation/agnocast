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

#include <array>
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

union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle /* for CARET */, const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs)
{
  std::array<topic_local_id_t, MAX_SUBSCRIBER_NUM> subscriber_ids_buffer{};

  union ioctl_publish_msg_args publish_msg_args = {};
  publish_msg_args.topic_name = {topic_name.c_str(), topic_name.size()};
  publish_msg_args.publisher_id = publisher_id;
  publish_msg_args.msg_virtual_address = msg_virtual_address;
  // The kernel writes subscriber IDs directly to this buffer via copy_to_user,
  // unlike ret_* fields which are copied back through the union.
  publish_msg_args.subscriber_ids_buffer_addr =
    reinterpret_cast<uint64_t>(subscriber_ids_buffer.data());
  publish_msg_args.subscriber_ids_buffer_size = MAX_SUBSCRIBER_NUM;

  if (ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, &publish_msg_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_PUBLISH_MSG_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  TRACEPOINT(agnocast_publish, publisher_handle, publish_msg_args.ret_entry_id);

  for (uint32_t i = 0; i < publish_msg_args.ret_subscriber_num; i++) {
    const topic_local_id_t subscriber_id = subscriber_ids_buffer[i];
    mqd_t mq = 0;
    if (opened_mqs.find(subscriber_id) != opened_mqs.end()) {
      std::tuple<mqd_t, bool> & t = opened_mqs[subscriber_id];
      mq = std::get<0>(t);
      // The boolean in the tuple indicates whether the mq is used in this publication round.
      // An unused mq means that its corresponding subscribers have exited, so we close such mqs
      // later.
      std::get<1>(t) = true;
    } else {
      const std::string mq_name = create_mq_name_for_agnocast_publish(topic_name, subscriber_id);
      mq = mq_open(mq_name.c_str(), O_WRONLY | O_NONBLOCK);
      if (mq == -1) {
        // Right after a subscriber is added, its message queue has not been created yet. Therefore,
        // the `mq_open` call above might fail. In that case, we just continue.
        RCLCPP_DEBUG_STREAM(
          logger, "mq_open failed for topic '" << topic_name << "' (subscriber_id=" << subscriber_id
                                               << ", mq_name='" << mq_name
                                               << "'): " << strerror(errno));
        continue;
      }
      opened_mqs.insert({subscriber_id, {mq, true}});
    }

    struct MqMsgAgnocast mq_msg = {};
    // Although the size of the struct is 1, we deliberately send a zero-length message
    if (mq_send(mq, reinterpret_cast<char *>(&mq_msg), 0 /*msg_len*/, 0) == -1) {
      // If it returns EAGAIN, it means mq_send has already been executed, but the subscriber
      // hasn't received it yet. Thus, there's no need to send it again since the notification has
      // already been sent.
      if (errno != EAGAIN) {
        RCLCPP_ERROR_STREAM(
          logger, "mq_send failed for topic '" << topic_name << "' (subscriber_id=" << subscriber_id
                                               << "): " << strerror(errno));
      }
    }
  }

  // Close mqs that are no longer needed and update `opened_mqs`
  for (auto it = opened_mqs.begin(); it != opened_mqs.end();) {
    bool & keep = std::get<1>(it->second);
    if (!keep) {
      mqd_t mq = std::get<0>(it->second);
      if (mq_close(mq) == -1) {
        RCLCPP_ERROR_STREAM(
          logger, "mq_close failed for topic '" << topic_name << "' (subscriber_id=" << it->first
                                                << "): " << strerror(errno));
      }
      it = opened_mqs.erase(it);
    } else {
      // Update the value for the next publication round
      keep = false;
      ++it;
    }
  }

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
  // If an A2R bridge exists, exclude the agnocast subscriber created by the bridge
  if (args.ret_a2r_bridge_exist && inter_count > 0) {
    inter_count--;
  }

  uint32_t ros2_count = args.ret_ros2_subscriber_num;
  // If an R2A bridge exists, exclude the ROS 2 subscriber created by the bridge
  if (args.ret_r2a_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  return inter_count + ros2_count;
}

uint32_t get_intra_subscription_count_core(const std::string & topic_name)
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
rclcpp::QoS PublisherBase::init_base(
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
  const rclcpp::QoS actual_qos = !options.qos_overriding_options.get_policy_kinds().empty()
                                   ? rclcpp::detail::declare_qos_parameters(
                                       options.qos_overriding_options, node_parameters, topic_name_,
                                       qos, rclcpp::detail::PublisherQosParametersTraits{})
                                   : qos;

  validate_publisher_qos(actual_qos);

  const bool is_bridge = (role == PublisherRole::BridgeInternal);
  const std::string node_name = node->get_fully_qualified_name();
  id_ = initialize_publisher(topic_name_, node_name, actual_qos, is_bridge, type_name);
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

  return actual_qos;
}

template rclcpp::QoS PublisherBase::init_base<rclcpp::Node>(
  rclcpp::Node *, const std::string &, const std::string &, const rclcpp::QoS &,
  const PublisherOptions &, PublisherRole);
template rclcpp::QoS PublisherBase::init_base<agnocast::Node>(
  agnocast::Node *, const std::string &, const std::string &, const rclcpp::QoS &,
  const PublisherOptions &, PublisherRole);

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
  {
    std::lock_guard<std::mutex> lock(opened_mqs_mtx_);
    for (auto & [_, t] : opened_mqs_) {
      mqd_t mq = std::get<0>(t);
      if (mq_close(mq) == -1) {
        RCLCPP_ERROR_STREAM(
          logger, "mq_close failed for topic '" << topic_name_ << "': " << strerror(errno));
      }
    }
  }

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

template <typename NodeT>
rclcpp::QoS GenericPublisher::constructor_impl(
  NodeT * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const agnocast::PublisherOptions & options, const PublisherRole role)
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

  return this->init_base(node, topic_name, topic_type, qos, options, role);
}

GenericPublisher::GenericPublisher(
  rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const PublisherOptions & options, PublisherRole role)
{
  const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, topic_type, qos, options, role);

  TRACEPOINT(
    agnocast_publisher_init, static_cast<const void *>(this),
    static_cast<const void *>(node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
    topic_name_.c_str(), actual_qos.depth());
}

GenericPublisher::GenericPublisher(
  agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
  const rclcpp::QoS & qos, const PublisherOptions & options, PublisherRole role)
{
  const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, topic_type, qos, options, role);

  TRACEPOINT(
    agnocast_publisher_init, static_cast<const void *>(this),
    static_cast<const void *>(get_node_base_address(node)), topic_name_.c_str(),
    actual_qos.depth());
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

  increment_borrowed_publisher_num();
  void * ptr = ::operator new(members_->size_of_);

  // Invoke the constructor of the message type at ptr.
  // Type-specific initialization is unnecessary because the message object
  // will be immediately populated with data by rmw_deserialize.
  // Perform minimal initialization only.
  members_->init_function(ptr, rosidl_runtime_cpp::MessageInitialization::SKIP);

  const rmw_ret_t ret =
    rmw_deserialize(&serialized_msg.get_rcl_serialized_message(), type_support_handle_, ptr);

  decrement_borrowed_publisher_num();

  if (ret != RMW_RET_OK) {
    members_->fini_function(ptr);
    ::operator delete(ptr);
    RCLCPP_ERROR(
      logger, "rmw_deserialize failed in GenericPublisher (rmw_ret=%d); dropping message",
      static_cast<int>(ret));
    return;
  }

  const auto va = reinterpret_cast<uint64_t>(ptr);
  union ioctl_publish_msg_args publish_msg_args {
  };
  {
    std::lock_guard<std::mutex> lock(opened_mqs_mtx_);
    publish_msg_args = publish_core(this, topic_name_, id_, va, opened_mqs_);
  }

  // Release entries that all subscribers have finished reading.
  // Only the addresses previously passed to the kernel by this publisher are returned here.
  // Therefore, the message type is guaranteed to match members_ and it's safe to call
  // fini_function on the pointer.
  for (uint32_t i = 0; i < publish_msg_args.ret_released_num; i++) {
    void * rptr = reinterpret_cast<void *>(publish_msg_args.ret_released_addrs[i]);
    members_->fini_function(rptr);
    ::operator delete(rptr);
  }
}

}  // namespace agnocast
