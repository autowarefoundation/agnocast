#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/bridge/agnocast_bridge_loader.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
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

// Lifecycle of one service's bridge. NONE is not a stored state: the manager destroys the item as
// soon as the machine reaches it. Arrow (1) is taken when a bridge request arrives; every other
// arrow is checked once per maintenance tick by check_and_update(), which re-tests the numbered
// conditions below from scratch each time. A bridge therefore appears, and is torn down, within
// one tick of the demand for it changing.
//
//                        ┌──────────────┐
//                        │  (no bridge  │
//                        │    state)    │
//                        └───┬───▲──────┘
//                            │   │
//              (1) Agnocast  │   │  (2) no Agnocast
//                  endpoint  │   │      endpoint
//                  registers │   │      left
//                      ┌─────▼───┴───┐
//           ┌──────────┤   Pending   ├──────────┐
//           │          └──▲───────▲──┘          │
//           │             │       │             │
//       (3) │         (4) │       │ (6)         │ (5)
//  Agnocast │       ROS 2 │       │ Agnocast    │ Agnocast service
//    client │  service or │       │ service or  │ and external
// and ROS 2 │ last client │       │ last client │ ROS 2 client
//   service │        gone │       │ gone        │ both exist
//       ┌───▼───┐         │       │         ┌───▼───┐
//       │  A2R  ├─────────┘       └─────────┤  R2A  │
//       └───────┘                           └───────┘
//
// (1) an Agnocast service or client registered
// (2) !agno_client_exists(), (3) false, and !(may_start_r2a_bridge_ && agno_service_exists())
// (3) may_start_a2r_bridge_ && ros2_service_exists() && agno_client_exists()
// (4) !(ros2_service_exists() && agno_client_exists()) || (may_start_r2a_bridge_ &&
//     agno_service_exists())
// (5) may_start_r2a_bridge_ && agno_service_exists() && (ros2_client_exists() || daemon-forced R2A)
// (6) !agno_service_exists() || !(ros2_client_exists() || daemon-forced R2A)
//
// Direction lives in flags, not in the state. handle_request() latches may_start_r2a_bridge_ (an
// Agnocast service registered) and may_start_a2r_bridge_ (an Agnocast client registered), and
// clears neither. Both are true in the ordinary Agnocast-to-Agnocast topology, so one state value
// could not carry it.
//
// (1) does not lead straight on to (5). Registering an Agnocast service makes may_start_r2a_bridge_
// and agno_service_exists() true, but (5) needs an external ROS 2 client as well, and usually there
// is none. The item then waits in Pending, which is where most items spend their whole life: a
// parameter service nobody calls is never bridged. (5) is taken on the first tick after a client
// appears, and (6) returns the item here once the last one leaves.
//
// The shadow node is created on entry to Pending, not with the bridge: it is what makes the
// agnocast::Node visible in the graph, and a tool that resolves a node name before opening a client
// could never create the client (5) waits for. It outlives (6), so a later (5) still finds it.
//
// That wait is stable rather than transient, because may_start_r2a_bridge_ && agno_service_exists()
// on their own -- client or no client -- already stop (3) and (2) from being evaluated. This is
// deliberate. The Agnocast service answers every Agnocast client directly, and an A2R bridge would
// add a second Agnocast service to the same request topic, so each request would be answered twice.
// A same-named ROS 2 service is therefore ignored, and the item kept alive, until that Agnocast
// service goes away.
//
// Two things escape that guard, and both let (2) destroy the item while an Agnocast service is
// still running. may_start_r2a_bridge_ is set only by a ServiceRole::Default service, so a
// BridgeInternal one never guards anything; and agno_service_exists() reads at most one entry, so
// it reports false when two or more Agnocast services share the name.
//
// The daemon-forced lease in (5) and (6) exists because two IPC namespaces sharing one service
// deadlock without it: the service side's (5) wants a ROS 2 client that only the client side's A2R
// creates, and the client side's (3) wants a ROS 2 service that only the service side's R2A
// creates. Only the discovery agent sees both namespaces, so it grants the lease.
//
// The lease covers R2A only, and that is enough to break the cycle: once (5) runs here, the ROS 2
// service it publishes is the one term (3) was missing over on the client side. Leasing A2R too
// would let it run before that service exists, and a request arriving in that window could be
// neither forwarded nor refused -- an Agnocast client cannot observe a dropped request.
enum class ServiceBridgeState { NONE, PENDING, A2R, R2A };

class ServiceBridgeItem
{
  std::string error_string_;

  // Stateful members.
  ServiceBridgeState state_ = ServiceBridgeState::NONE;
  ServiceBridgeEntity entity_ = {nullptr, nullptr, nullptr};
  // Held, never read: an agnocast::Node creates no rcl_node_t of its own, so this stands in for one
  // under its name, in this process. Lifetime is decided in check_and_update_pending().
  std::shared_ptr<rcl_node_t> shadow_node_ = nullptr;

  std::optional<std::chrono::steady_clock::time_point> r2a_forced_until_ = std::nullopt;

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
  bool acquire_shadow_node();
  void release_shadow_node();

  int get_agno_service_qos(rclcpp::QoS & qos);

  bool ros2_service_exists(const ServiceBridgeDeps & deps);
  bool ros2_client_exists(const ServiceBridgeDeps & deps);

  bool r2a_forced() const;
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
  void handle_daemon_request();
};

}  // namespace agnocast
