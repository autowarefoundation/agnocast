#include "agnocast/bridge/agnocast_service_bridge.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <sys/ioctl.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <map>
#include <utility>

namespace
{

// Global map from (namespace, name) pair to its shadow node's weak pointer.
std::map<std::pair<std::string, std::string>, std::weak_ptr<rcl_node_t>> g_shadow_nodes;

}  // namespace

namespace agnocast
{

void ServiceBridgeItem::set_error_string(const char * error_string)
{
  error_string_ = error_string;
}
const char * ServiceBridgeItem::get_error_string()
{
  return error_string_.c_str();
}

// Returns nullptr if an error occurs while creating the shadow node (the error string will be set).
std::shared_ptr<rcl_node_t> ServiceBridgeItem::find_or_create_shadow_node(
  const std::pair<std::string, std::string> & identity)
{
  auto checked_rcl_node_options_fini = [](rcl_node_options_t * options) {
    if (rcl_node_options_fini(options) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "agnocast_bridge", "Failed to fini node options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  };

  auto it = g_shadow_nodes.find(identity);
  if (it != g_shadow_nodes.end()) {
    auto shadow_node = it->second.lock();
    if (shadow_node != nullptr) {
      return shadow_node;
    }
  }

  rcl_context_t * rcl_ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context().get();

  rcl_node_options_t options = rcl_node_get_default_options();
  options.enable_rosout = false;
  options.use_global_arguments = false;
  if (rcl_parse_arguments(0, nullptr, options.allocator, &(options.arguments)) != RCL_RET_OK) {
    rcl_reset_error();
    set_error_string("Failed to parse arguments while creating shadow node");
    checked_rcl_node_options_fini(&options);
    return nullptr;
  }

  auto del = [](rcl_node_t * node) {
    if (rcl_node_is_valid(node)) {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "agnocast_bridge", "Error in destruction of shadow node: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete node;  // NOLINT(cppcoreguidelines-owning-memory)
  };
  auto node = std::shared_ptr<rcl_node_t>(new rcl_node_t{}, del);

  if (
    rcl_node_init(node.get(), identity.second.c_str(), identity.first.c_str(), rcl_ctx, &options) !=
    RCL_RET_OK) {
    rcl_reset_error();
    set_error_string("Failed to initialize shadow node");
    checked_rcl_node_options_fini(&options);
    return nullptr;
  }

  checked_rcl_node_options_fini(&options);
  g_shadow_nodes[identity] = node;
  return node;
}

void ServiceBridgeItem::erase_expired_shadow_node(
  const std::pair<std::string, std::string> & identity)
{
  auto it = g_shadow_nodes.find(identity);
  if (it != g_shadow_nodes.end() && it->second.expired()) {
    g_shadow_nodes.erase(it);
  }
}

// Returns 0 on success, -1 on error (the error string will be set).
int ServiceBridgeItem::get_agno_service_qos(rclcpp::QoS & qos)
{
  const std::string request_topic_name = create_service_request_topic_name(service_name_);

  auto topic_info_buffer = std::make_unique<std::array<topic_info_ret, 1>>();
  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {request_topic_name.c_str(), request_topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(topic_info_buffer->data());
  topic_info_args.topic_info_ret_buffer_size = 1;

  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    if (errno == ENOBUFS) {
      set_error_string("Multiple target agnocast services found");
    } else {
      set_error_string("Failed to fetch target service information from agnocast kernel module");
    }
    return -1;
  }

  if (topic_info_args.ret_topic_info_ret_num <= 0) {
    set_error_string("No target agnocast service found");
    return -1;
  }

  const topic_info_ret & info = (*topic_info_buffer)[0];

  // We know the durability policy is set to Volatile because this is a service.
  qos.keep_last(info.qos_depth)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .reliability(
      info.qos_is_reliable ? rclcpp::ReliabilityPolicy::Reliable
                           : rclcpp::ReliabilityPolicy::BestEffort);
  return 0;
}

// Returns false if the target ROS 2 service does not exist or if an exception occurs while checking
// it (the reason will be set in the error string).
bool ServiceBridgeItem::ros2_service_exists(const ServiceBridgeDeps & deps)
{
  try {
    bool exists = false;

#if RCLCPP_VERSION_MAJOR >= 28
    exists = deps.container_node->count_services(this->service_name_) > 0;
#else
    // NOTE(bdm-k): A potential performance enhancement would be to use the ROS 2 client in the
    // bridge entity whenever it is available.
    const auto node_names = deps.container_node->get_node_names();
    for (const auto & full_name : node_names) {
      const auto [ns, name] = split_full_node_name(full_name);
      const auto srvs = deps.container_node->get_service_names_and_types_by_node(name, ns);

      if (srvs.find(service_name_) != srvs.end()) {
        exists = true;
        break;
      }
    }
#endif

    if (!exists) {
      set_error_string("No target ROS 2 service found");
    }
    return exists;
  } catch (const std::exception & e) {
    set_error_string(e.what());
    return false;
  } catch (...) {
    set_error_string("Unknown error");
    return false;
  }
}

// Returns false if the target Agnocast service does not exist or if an error occurs while checking
// it (the reason will be set in the error string).
bool ServiceBridgeItem::agno_service_exists()
{
  // TODO(bdm-k): Add a dedicated service-liveness ioctl so we can validate target service state
  // directly without using get_service_qos() as a probe.
  rclcpp::QoS qos = rclcpp::ServicesQoS();
  return get_agno_service_qos(qos) == 0;
}

// Returns false if there is no target Agnocast client or if an error occurs while checking it (the
// reason will be set in the error string).
bool ServiceBridgeItem::agno_client_exists()
{
  const std::string request_topic_name = create_service_request_topic_name(service_name_);

  PublisherCountResult result = get_agnocast_publisher_count(request_topic_name);

  if (result.count == -1) {
    set_error_string("Failed to fetch publisher count from agnocast kernel module");
    return false;
  }
  if (result.count == 0) {
    set_error_string("No target Agnocast client found");
    return false;
  }
  return true;
}

// Creates and starts the R2A bridge. Relevant configuration members must be set beforehand.
// Returns 0 on success, -1 on error (the error string will be set). On error, it is guaranteed
// that the stateful members are not modified.
int ServiceBridgeItem::start_r2a_bridge(const ServiceBridgeDeps & deps)
{
  // Warn if the target service already exists in ROS 2.
  if (ros2_service_exists(deps)) {
    RCLCPP_WARN(
      deps.logger,
      "Found a ROS 2 service with the same name while creating the R2A service bridge: '%s'",
      service_name_.c_str());
  }

  rclcpp::QoS service_qos = rclcpp::ServicesQoS();
  if (get_agno_service_qos(service_qos) != 0) {
    return -1;
  }

  std::shared_ptr<rcl_node_t> shadow_node;
  if (shadow_node_identity_.has_value()) {
    const auto & identity = *shadow_node_identity_;
    if ((shadow_node = find_or_create_shadow_node(identity)) == nullptr) {
      return -1;
    }
  }

  ServiceBridgeEntity entity;
  if (service_type_.has_value() && deps.performance_loader != nullptr) {
    try {
      entity = deps.performance_loader->create_r2a_service_bridge(
        deps.container_node, service_name_, *service_type_, service_qos);
    } catch (const std::exception & e) {
      set_error_string(e.what());
      return -1;
    } catch (...) {
      set_error_string("Unknown error");
      return -1;
    }
  } else {
    set_error_string("missing configuration members or bridge loader");
    return -1;
  }

  if (entity.entity_handle == nullptr) {
    set_error_string("Bridge loader returned nullptr");
    return -1;
  }

  state_ = ServiceBridgeState::R2A;
  entity_ = std::move(entity);
  shadow_node_ = std::move(shadow_node);
  return 0;
}

// Creates and starts the A2R bridge. Relevant configuration members must be set beforehand.
// Returns 0 on success, -1 on error (the error string will be set). On error, it is guaranteed
// that the stateful members are not modified.
int ServiceBridgeItem::start_a2r_bridge(const ServiceBridgeDeps & deps)
{
  ServiceBridgeEntity entity;
  if (service_type_.has_value() && deps.performance_loader != nullptr) {
    try {
      entity = deps.performance_loader->create_a2r_service_bridge(
        deps.container_node, service_name_, *service_type_, rclcpp::ServicesQoS());
    } catch (const std::exception & e) {
      set_error_string(e.what());
      return -1;
    } catch (...) {
      set_error_string("Unknown error");
      return -1;
    }
  } else {
    set_error_string("missing configuration members or bridge loader");
    return -1;
  }

  if (entity.entity_handle == nullptr) {
    set_error_string("Bridge loader returned nullptr");
    return -1;
  }

  state_ = ServiceBridgeState::A2R;
  entity_ = std::move(entity);
  shadow_node_ = nullptr;
  return 0;
}

void ServiceBridgeItem::update_configuration(const BridgeMsgServicePayload & payload)
{
  if (service_name_.empty()) {
    service_name_ = static_cast<const char *>(payload.service_name);
  }
  if (!service_type_.has_value()) {
    service_type_ = static_cast<const char *>(payload.service_type);
  }
  if (!shadow_node_identity_.has_value() && payload.create_shadow_node) {
    shadow_node_identity_ = {
      static_cast<const char *>(payload.shadow_node_namespace),
      static_cast<const char *>(payload.shadow_node_name)};
  }

  if (payload.direction == BridgeDirection::ROS2_TO_AGNOCAST) {
    may_start_r2a_bridge_ = true;
  } else {
    may_start_a2r_bridge_ = true;
  }
}

void ServiceBridgeItem::check_and_update_r2a(const ServiceBridgeDeps & deps)
{
  if (agno_service_exists()) {
    return;
  }

  RCLCPP_WARN(
    deps.logger, "Removing R2A service bridge for '%s': %s", service_name_.c_str(),
    get_error_string());

  if (entity_.srv_cb_group) {
    deps.executor->stop_callback_group(entity_.srv_cb_group);
  }
  if (entity_.client_cb_group) {
    deps.executor->stop_callback_group(entity_.client_cb_group);
  }

  state_ = ServiceBridgeState::PENDING;
  entity_ = {nullptr, nullptr, nullptr};
  shadow_node_ = nullptr;

  if (shadow_node_identity_.has_value()) {
    erase_expired_shadow_node(shadow_node_identity_.value());
  }
}

void ServiceBridgeItem::check_and_update_a2r(const ServiceBridgeDeps & deps)
{
  if (ros2_service_exists(deps)) {
    return;
  }

  RCLCPP_WARN(
    deps.logger, "Removing A2R service bridge for '%s': %s", service_name_.c_str(),
    get_error_string());

  if (entity_.srv_cb_group) {
    deps.executor->stop_callback_group(entity_.srv_cb_group);
  }
  if (entity_.client_cb_group) {
    deps.executor->stop_callback_group(entity_.client_cb_group);
  }

  state_ = ServiceBridgeState::PENDING;
  entity_ = {nullptr, nullptr, nullptr};
  shadow_node_ = nullptr;
}

void ServiceBridgeItem::check_and_update_pending(const ServiceBridgeDeps & deps)
{
  if (may_start_r2a_bridge_ && agno_service_exists()) {
    if (start_r2a_bridge(deps) != 0) {
      RCLCPP_WARN(
        deps.logger, "Failed to start R2A service bridge for '%s': %s", service_name_.c_str(),
        get_error_string());
    }
    return;
  }

  if (may_start_a2r_bridge_ && ros2_service_exists(deps)) {
    if (start_a2r_bridge(deps) != 0) {
      RCLCPP_WARN(
        deps.logger, "Failed to start A2R service bridge for '%s': %s", service_name_.c_str(),
        get_error_string());
    }
    return;
  }

  if (!agno_client_exists()) {
    RCLCPP_WARN(
      deps.logger, "Removing service bridge state-machine for '%s': %s", service_name_.c_str(),
      get_error_string());

    state_ = ServiceBridgeState::NONE;
  }
}

void ServiceBridgeItem::check_and_update(const ServiceBridgeDeps & deps)
{
  switch (state_) {
    case ServiceBridgeState::PENDING:
      check_and_update_pending(deps);
      break;
    case ServiceBridgeState::R2A:
      check_and_update_r2a(deps);
      break;
    case ServiceBridgeState::A2R:
      check_and_update_a2r(deps);
      break;
    default:
      break;
  }
}

void ServiceBridgeItem::handle_request_with_direction(
  BridgeDirection direction, const ServiceBridgeDeps & deps)
{
  switch (direction) {
    case BridgeDirection::ROS2_TO_AGNOCAST:
      if (state_ == ServiceBridgeState::NONE || state_ == ServiceBridgeState::PENDING) {
        if (start_r2a_bridge(deps) != 0) {
          RCLCPP_WARN(
            deps.logger, "Failed to start R2A service bridge for '%s': %s", service_name_.c_str(),
            get_error_string());
        }
      }
      break;
    case BridgeDirection::AGNOCAST_TO_ROS2:
      if (state_ == ServiceBridgeState::NONE) {
        state_ = ServiceBridgeState::PENDING;
      }
      break;
  }
}

void ServiceBridgeItem::handle_request(
  const BridgeMsgServicePayload & payload, const ServiceBridgeDeps & deps)
{
  update_configuration(payload);
  handle_request_with_direction(payload.direction, deps);
}

}  // namespace agnocast
