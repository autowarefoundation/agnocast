#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace agnocast
{

inline constexpr std::string_view SUFFIX_PUBSUB_R2A = "_P_R2A";
inline constexpr std::string_view SUFFIX_PUBSUB_A2R = "_P_A2R";
inline constexpr std::string_view SUFFIX_SERVICE_R2A = "_S_R2A";
inline constexpr std::string_view SUFFIX_SERVICE_A2R = "_S_A2R";

inline constexpr size_t SUFFIX_LEN = 6;
static_assert(SUFFIX_PUBSUB_R2A.length() == SUFFIX_LEN);
static_assert(SUFFIX_PUBSUB_A2R.length() == SUFFIX_LEN);
static_assert(SUFFIX_SERVICE_R2A.length() == SUFFIX_LEN);
static_assert(SUFFIX_SERVICE_A2R.length() == SUFFIX_LEN);

enum class BridgeMode : int { Off = 0, Standard = 1, Performance = 2 };

class PubsubBridgeBase
{
public:
  virtual ~PubsubBridgeBase() = default;
  virtual rclcpp::CallbackGroup::SharedPtr get_callback_group() const = 0;
};

class ServiceBridgeBase
{
public:
  virtual ~ServiceBridgeBase() = default;
  virtual std::pair<rclcpp::CallbackGroup::SharedPtr, rclcpp::CallbackGroup::SharedPtr>
  get_callback_groups() const = 0;
};

struct SubscriberCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if A2R bridge exists
};

struct PublisherCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if R2A bridge exists
};

BridgeMode get_bridge_mode();
rclcpp::QoS get_subscriber_qos(const std::string & topic_name, topic_local_id_t subscriber_id);
rclcpp::QoS get_publisher_qos(const std::string & topic_name, topic_local_id_t publisher_id);
PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name);
SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name);
bool update_ros2_subscriber_num(const rclcpp::Node * node, const std::string & topic_name);
bool update_ros2_publisher_num(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name);
rclcpp::QoS get_service_qos(const std::string & service_name);
bool is_agnocast_service_alive(const std::string & service_name, std::string & reason);

/// @brief A builder class for creating bridge registration messages.
///
/// It handles errors such as setting non-existent fields or invalid values for each field, and the
/// `build_*` member functions return an error reason. However, it does not check whether the
/// computed message as a whole is valid. It's the caller's responsibility to invoke a correct
/// set of setters to build a valid message.
class BridgeRegistrationMsgBuilder
{
  std::variant<MqMsgBridge, MqMsgPerformanceBridge> msg_;
  rclcpp::Logger logger_;
  bool failed_;
  std::string reason_;

  BridgeRegistrationMsgBuilder & fail(const char * format, ...);
  int checked_snprintf(
    const std::string & member, char * buffer, size_t size, const char * format, ...);

public:
  enum class Mode {
    Standard,
    Performance,
  };

  explicit BridgeRegistrationMsgBuilder(Mode mode, const rclcpp::Logger & logger);

  BridgeRegistrationMsgBuilder & set_direction(BridgeDirection direction);
  BridgeRegistrationMsgBuilder & set_is_service(bool is_service);
  BridgeRegistrationMsgBuilder & set_factory(uintptr_t fn_r2a, uintptr_t fn_a2r);
  BridgeRegistrationMsgBuilder & set_message_type(const char * message_type);
  BridgeRegistrationMsgBuilder & set_topic_name(const char * topic_name);
  BridgeRegistrationMsgBuilder & set_pubsub_target_id(topic_local_id_t target_id);
  BridgeRegistrationMsgBuilder & set_service_type(const char * service_type);
  BridgeRegistrationMsgBuilder & set_service_name(const char * service_name);
  BridgeRegistrationMsgBuilder & set_shadow_node_identity(
    const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);

  std::pair<MqMsgBridge, std::string> build_standard_message();
  std::pair<MqMsgPerformanceBridge, std::string> build_performance_message();
};

template <typename MapT>
std::shared_ptr<rcl_node_t> find_or_create_shadow_node(
  const MapT & active_r2a_service_bridges, const std::string & ns, const std::string & name)
{
  for (const auto & [_, item] : active_r2a_service_bridges) {
    const std::shared_ptr<rcl_node_t> & shadow_node = item.shadow_node;
    if (
      shadow_node != nullptr && strcmp(rcl_node_get_name(shadow_node.get()), name.c_str()) == 0 &&
      strcmp(rcl_node_get_namespace(shadow_node.get()), ns.c_str()) == 0) {
      return shadow_node;
    }
  }

  rcl_context_t * rcl_ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context().get();

  rcl_node_options_t options = rcl_node_get_default_options();
  options.enable_rosout = false;
  options.use_global_arguments = false;
  if (rcl_parse_arguments(0, nullptr, options.allocator, &(options.arguments)) != RCL_RET_OK) {
    rcl_reset_error();
    throw std::runtime_error("Failed to parse arguments while creating shadow node");
  }

  auto del = [](rcl_node_t * node) {
    if (rcl_node_is_valid(node)) {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "agnocast_bridge", "Error in destruction of shadow node: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete node;
  };
  auto node = std::shared_ptr<rcl_node_t>(new rcl_node_t{}, del);

  if (rcl_node_init(node.get(), name.c_str(), ns.c_str(), rcl_ctx, &options) != RCL_RET_OK) {
    rcl_reset_error();
    throw std::runtime_error("Failed to initialize shadow node");
  }

  return node;
}

}  // namespace agnocast
