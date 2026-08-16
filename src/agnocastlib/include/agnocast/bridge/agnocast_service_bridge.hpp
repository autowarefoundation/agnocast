#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/agnocast_bridge_loader.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"

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
  std::shared_ptr<BridgeLoader> bridge_loader;
};

// Lifecycle of one service's bridge. NONE is not a stored state: the manager keeps an entry in
// active_service_bridges_ only while the machine is out of NONE, so reaching NONE destroys the
// item. Only edge (1) is request-driven; check_and_update() re-derives every other edge from the
// current world once per maintenance tick and caches nothing, so a bridge appears at most one tick
// after the demand for it does, and is reaped at most one tick after that demand goes away.
//
//                        ┌──────────────┐
//                        │  (no bridge  │
//                        │    state)    │
//                        └───┬───▲──────┘
//                            │   │
//              (1) Agnocast  │   │  (2) no Agnocast
//                  endpoint  │   │      client left
//                  registers │   │
//                      ┌─────▼───┴───┐
//           ┌──────────┤   Pending   ├──────────┐
//           │          └──▲───────▲──┘          │
//           │             │       │             │
//       (3) │         (4) │       │ (6)         │ (5)
//     ROS 2 │       ROS 2 │       │ Agnocast    │ Agnocast service
//   service │     service │       │ service or  │ and external
//  detected │        gone │       │ last client │ ROS 2 client
//           │             │       │ gone        │ both exist
//       ┌───▼───┐         │       │         ┌───▼───┐
//       │  A2R  ├─────────┘       └─────────┤  R2A  │
//       └───────┘                           └───────┘
//
// Which directions this item may build is fixed at registration and deliberately kept out of the
// state: handle_request() latches may_start_r2a_bridge_ (an Agnocast service announced itself) and
// may_start_a2r_bridge_ (an Agnocast client did), and clears neither. Both are true in the ordinary
// Agnocast-service-to-Agnocast-client topology, which is why direction is a pair of flags rather
// than a pair of Pending states.
//
// (1) An Agnocast service or client registered. handle_request() only enters Pending; every
//     creation decision is deferred to the tick below.
// (2) No Agnocast client publishes on the request topic, and neither (3) nor (5) applied.
// (3) may_start_a2r_bridge_ && ros2_service_exists().
// (4) !ros2_service_exists().
// (5) may_start_r2a_bridge_ && agno_service_exists() && ros2_client_exists(). The client check is
//     the demand gate: an R2A bridge costs a callback group, which the callback-isolated executor
//     backs with its own thread and epoll, so it is built only while a client is there to use it.
//     Waiting in Pending instead costs nothing, which is what stops N nodes' parameter services
//     from each pinning a thread whether or not anyone reads a parameter.
// (6) !agno_service_exists() || !ros2_client_exists(), the mirror of (5): the bridge and its thread
//     are reaped as soon as the last ROS 2 client leaves.
//
// A live Agnocast service excludes (3) and (2). While may_start_r2a_bridge_ &&
// agno_service_exists() holds, neither is evaluated -- not even when (5) itself does not fire for
// want of a client. This is deliberate: that service already receives every Agnocast client's
// request directly, and an A2R bridge would put a second Agnocast service on the same request
// topic, so each request would be answered twice. The item therefore waits in Pending and ignores
// any same-named ROS 2 service until the Agnocast service goes away, at which point (3) applies as
// usual.
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
  bool ros2_client_exists(const ServiceBridgeDeps & deps);
  bool agno_service_exists();
  bool agno_client_exists();

  int start_r2a_bridge(const ServiceBridgeDeps & deps);
  int start_a2r_bridge(const ServiceBridgeDeps & deps);

  void update_configuration(const BridgeMsgServicePayload & payload);

  void check_and_update_r2a(const ServiceBridgeDeps & deps);
  void check_and_update_a2r(const ServiceBridgeDeps & deps);
  void check_and_update_pending(const ServiceBridgeDeps & deps);

public:
  ServiceBridgeState state() const { return state_; }
  const std::string & service_name() const { return service_name_; }

  void check_and_update(const ServiceBridgeDeps & deps);

  void handle_request(const BridgeMsgServicePayload & payload);
};

}  // namespace agnocast
