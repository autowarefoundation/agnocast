#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace agnocast
{

struct ServiceBridgeDeps
{
  rclcpp::Node::SharedPtr container_node;
  std::shared_ptr<CallbackIsolatedAgnocastExecutor> executor;
  rclcpp::Logger logger;
  std::shared_ptr<PerformanceBridgeLoader> performance_loader;
};

enum class ServiceBridgeState { NONE, PENDING, A2R, R2A };

class ServiceBridgeItem
{
  std::string error_string_;

  // Stateful members.
  ServiceBridgeState state_ = ServiceBridgeState::NONE;
  ServiceBridgeEntity entity_ = {nullptr, nullptr, nullptr};
  std::shared_ptr<rcl_node_t> shadow_node_ = nullptr;

  // Configuration members; set once and never modified.
  std::string service_name_;
  std::optional<std::string> service_type_ = std::nullopt;
  std::optional<std::pair<std::string, std::string>> shadow_node_identity_ = std::nullopt;
  bool may_start_r2a_bridge_ = false;
  bool may_start_a2r_bridge_ = false;

  void set_error_string(const char * error_string);
  const char * get_error_string();

  std::shared_ptr<rcl_node_t> find_or_create_shadow_node(
    const std::pair<std::string, std::string> & identity);
  static void erase_expired_shadow_node(const std::pair<std::string, std::string> & identity);

  int get_agno_service_qos(rclcpp::QoS & qos);

  bool ros2_service_exists(const ServiceBridgeDeps & deps);
  bool agno_service_exists();
  bool agno_client_exists();

  int start_r2a_bridge(const ServiceBridgeDeps & deps);
  int start_a2r_bridge(const ServiceBridgeDeps & deps);

  void update_configuration(const BridgeMsgServicePayload & payload);

  void check_and_update_r2a(const ServiceBridgeDeps & deps);
  void check_and_update_a2r(const ServiceBridgeDeps & deps);
  void check_and_update_pending(const ServiceBridgeDeps & deps);

  void handle_request_with_direction(BridgeDirection direction, const ServiceBridgeDeps & deps);

public:
  ServiceBridgeState state() const { return state_; }
  const std::string & service_name() const { return service_name_; }

  void check_and_update(const ServiceBridgeDeps & deps);

  void handle_request(const BridgeMsgServicePayload & payload, const ServiceBridgeDeps & deps);
};

}  // namespace agnocast
