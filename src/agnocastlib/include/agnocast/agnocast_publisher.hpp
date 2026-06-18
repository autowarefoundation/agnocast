#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace agnocast
{
class Node;

const void * get_node_base_address(Node * node);

// These are cut out of the class for information hiding.
topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos,
  const bool is_bridge, const std::string & type_name);
union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle, /* for CARET */ const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs);
uint32_t get_subscription_count_core(const std::string & topic_name);
uint32_t get_intra_subscription_count_core(const std::string & topic_name);
void increment_borrowed_publisher_num();
void decrement_borrowed_publisher_num();

extern int agnocast_fd;
extern "C" uint32_t agnocast_get_borrowed_publisher_num();

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
  std::string topic_name_;
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> opened_mqs_;
  std::mutex opened_mqs_mtx_;
  rmw_gid_t gid_;

  template <typename NodeT>
  rclcpp::QoS init_base(
    NodeT * node, const std::string & topic_name, const std::string & type_name,
    const rclcpp::QoS & qos, const PublisherOptions & options, const bool is_bridge)
  {
    if (options.do_always_ros2_publish) {
      RCLCPP_ERROR(
        logger,
        "The 'do_always_ros2_publish' option is deprecated. "
        "Use the AGNOCAST_BRIDGE_MODE environment variable instead.");
    }

    topic_name_ = node->get_node_topics_interface()->resolve_topic_name(topic_name);

    auto node_parameters = node->get_node_parameters_interface();
    const rclcpp::QoS actual_qos =
      options.qos_overriding_options.get_policy_kinds().size()
        ? rclcpp::detail::declare_qos_parameters(
            options.qos_overriding_options, node_parameters, topic_name_, qos,
            rclcpp::detail::PublisherQosParametersTraits{})
        : qos;

    validate_publisher_qos(actual_qos);

    const std::string node_name = node->get_fully_qualified_name();
    id_ = initialize_publisher(topic_name_, node_name, actual_qos, is_bridge, type_name);
    generate_gid();

    return actual_qos;
  }

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

// Internal implementation — users should use agnocast::Publisher<MessageT> instead.
template <typename MessageT, typename BridgeRegistrationPolicy>
class BasicPublisher : public PublisherBase
{
  template <typename NodeT>
  rclcpp::QoS constructor_impl(
    NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options, const bool is_bridge)
  {
    // Gated to message types only — service types pulled in by
    // BasicService<ServiceT> have no rosidl message name. The empty string
    // signals "skip registry" to initialize_publisher.
    std::string type_name;
    if constexpr (rosidl_generator_traits::is_message<MessageT>::value) {
      type_name = rosidl_generator_traits::name<MessageT>();
    }

    const rclcpp::QoS actual_qos =
      this->init_base(node, topic_name, type_name, qos, options, is_bridge);

    BridgeRegistrationPolicy::template register_bridge<MessageT>(topic_name_, id_);

    return actual_qos;
  }

public:
  using SharedPtr = std::shared_ptr<BasicPublisher<MessageT, BridgeRegistrationPolicy>>;

  BasicPublisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options, const bool is_bridge = false)
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, qos, options, is_bridge);

    TRACEPOINT(
      agnocast_publisher_init, static_cast<const void *>(this),
      static_cast<const void *>(
        node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
      topic_name_.c_str(), actual_qos.depth());
  }

  BasicPublisher(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & options = PublisherOptions{})
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, topic_name, qos, options, false);

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
    increment_borrowed_publisher_num();
    MessageT * ptr = new MessageT();
    return ipc_shared_ptr<MessageT>(ptr, topic_name_.c_str(), id_);
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
      publish_msg_args = publish_core(this, topic_name_, id_, msg_virtual_address, opened_mqs_);
    }

    for (uint32_t i = 0; i < publish_msg_args.ret_released_num; i++) {
      MessageT * release_ptr = reinterpret_cast<MessageT *>(publish_msg_args.ret_released_addrs[i]);
      delete release_ptr;
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
class GenericPublisher : public PublisherBase
{
  // Keeps the dynamically loaded typesupport and introspection shared libraries
  // (.so) alongside their handles for the lifetime of the publisher.
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * type_support_handle_{nullptr};
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_{nullptr};

  template <typename NodeT>
  rclcpp::QoS constructor_impl(
    NodeT * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, const PublisherOptions & options, PublisherRole role);

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

struct AgnocastToRosPubsubRegistrationPolicy;

/**
 * @brief The user-facing Agnocast publisher type.
 * Alias for `BasicPublisher<MessageT>`. Use this type (not BasicPublisher directly) when declaring
 * publisher variables.
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
using Publisher =
  agnocast::BasicPublisher<MessageT, agnocast::AgnocastToRosPubsubRegistrationPolicy>;

}  // namespace agnocast
