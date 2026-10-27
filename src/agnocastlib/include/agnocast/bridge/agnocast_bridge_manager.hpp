#pragma once

#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/agnocast_bridge_loader.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"
#include "agnocast/bridge/agnocast_service_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace agnocast
{

class BridgeManager
{
public:
  BridgeManager();
  ~BridgeManager();

  void run();

private:
  using RequestMap = std::unordered_map<topic_local_id_t, BridgeMsgPubSubPayload>;

  // A cross-NS bridge request from the per-NS daemon. Unlike an intra-NS request,
  // there is no local endpoint to resolve the plugin / QoS from, so the type and
  // QoS travel in the request itself. While unexpired, the bridge is created
  // without a same-graph DDS counterpart (the peer NS's bridge creates it) and is
  // shielded from demand-based teardown.
  struct DaemonForcedRequest
  {
    std::string message_type;
    rclcpp::QoS qos;
    std::chrono::steady_clock::time_point forced_until;
  };

  rclcpp::Logger logger_;
  // Steady clock for throttled logging. Owned here rather than taken from
  // container_node_, since throttled logging happens on the event loop thread.
  std::shared_ptr<rclcpp::Clock> clock_;
  uint64_t self_ipc_ns_inode_;
  BridgeIpcEventLoop event_loop_;
  std::shared_ptr<BridgeLoader> loader_;

  std::shared_ptr<rclcpp::Node> container_node_;
  std::shared_ptr<agnocast::CallbackIsolatedAgnocastExecutor> executor_;
  std::thread executor_thread_;
  std::thread worker_thread_;

  std::mutex pending_msgs_mtx_;
  std::condition_variable pending_msgs_cv_;
  std::deque<BridgeMsg> pending_msgs_;

  std::atomic_bool shutdown_requested_ = false;

  std::unordered_map<std::string, PubsubBridgeResult> active_pubsub_r2a_bridges_;
  std::unordered_map<std::string, PubsubBridgeResult> active_pubsub_a2r_bridges_;
  std::unordered_map<std::string, RequestMap> request_cache_;

  // Daemon-forced cross-NS requests, keyed by topic, split by direction.
  std::unordered_map<std::string, DaemonForcedRequest> daemon_forced_r2a_;
  std::unordered_map<std::string, DaemonForcedRequest> daemon_forced_a2r_;

  std::unordered_map<std::string, ServiceBridgeItem> active_service_bridges_;

  // PoC (stage 1): advertise every Agnocast endpoint on the ROS 2 graph up front, reading the
  // endpoints from the kernel module. Enabled by AGNOCAST_BRIDGE_SHOW=1. Nothing is relayed by
  // these parts; the existing on-demand bridges above keep carrying the data.
  enum class ExecModel { PerPart, Grouped };

  struct ShownPart
  {
    std::shared_ptr<rclcpp::Node> owner;
    std::shared_ptr<rclcpp::GenericPublisher> writer;
    rclcpp::SubscriptionBase::SharedPtr reader;
    std::shared_ptr<void> server;
    rclcpp::CallbackGroup::SharedPtr group;
    // Stage 2: the kmod-side parts that carry the data.
    std::string type_name;
    uint32_t qos_depth = 1;
    bool qos_is_transient_local = false;
    std::shared_ptr<agnocast::GenericSubscription> receiver;  // I, routed from the publisher
    std::shared_ptr<agnocast::GenericPublisher> sender;       // T, routed to the subscriber
    rclcpp::CallbackGroup::SharedPtr relay_group;
    bool relay_paused = false;
    // Group mode: the QoS last written to the kmod-side part, which follows the ROS 2 side.
    uint32_t applied_depth = 0;
    bool applied_transient_local = false;
  };

  // (topic name, endpoint id, is_publisher)
  using ShownKey = std::tuple<std::string, topic_local_id_t, bool>;
  // Group mode (AGNOCAST_BRIDGE_SHOW=group): one set of parts per topic and QoS of the Agnocast
  // endpoints. (topic name, is_publisher, depth, transient local, reliable); a publisher has no
  // reliability, so it is always false for publisher groups.
  using GroupKey = std::tuple<std::string, bool, uint32_t, bool, bool>;

  bool show_enabled_ = false;
  bool group_mode_ = false;
  ExecModel exec_model_ = ExecModel::PerPart;
  std::string show_stats_path_;
  std::map<ShownKey, ShownPart> shown_pubsub_;
  std::map<ShownKey, std::string> failed_pubsub_;
  std::map<GroupKey, ShownPart> shown_groups_;
  std::map<GroupKey, std::string> failed_groups_;
  std::map<std::string, ShownPart> shown_services_;
  std::map<std::pair<std::string, std::string>, std::weak_ptr<rclcpp::Node>> shadow_nodes_;
  std::atomic<uint64_t> shown_reader_msg_count_ = 0;
  std::atomic<uint64_t> shown_server_request_count_ = 0;
  // Grouped: a group belongs to its node, so each owner gets one group, shared by all its parts and
  // run, together with every other owner's group, by one CIE thread.
  struct OwnerGroup
  {
    rclcpp::CallbackGroup::SharedPtr group;
    size_t part_num = 0;
  };
  std::map<const rclcpp::Node *, OwnerGroup> owner_groups_;
  // Grouped, kmod side: every receiver is in this one group of the container node, run by one CIE
  // thread. A receiver leaving it is paused first and dropped on the next sync, so that a receive
  // already queued on that thread finds it paused rather than gone (review high-3).
  rclcpp::CallbackGroup::SharedPtr relay_shared_group_;
  std::vector<std::shared_ptr<agnocast::GenericSubscription>> retiring_receivers_;
  size_t last_logged_part_num_ = 0;
  int last_logged_thread_num_ = -1;

  // PoC startup profiling (AGNOCAST_BRIDGE_PROFILE).
  std::atomic<uint64_t> profile_received_msgs_ = 0;
  std::atomic<uint64_t> profile_pending_max_ = 0;
  int64_t profile_show_create_us_ = 0;
  void profile_event(const std::string & ev, const std::string & fields);

  void sync_shown_parts();
  void sync_relays(const std::string & topic_name, const std::vector<ShownKey> & keys);
  void sync_shown_groups();
  void sync_group_relays(const std::string & topic_name, const std::vector<GroupKey> & keys);
  void retire_dead_services();
  rclcpp::CallbackGroup::SharedPtr create_relay_group();
  void attach_relay_group(const rclcpp::CallbackGroup::SharedPtr & group);
  void detach_relay_group(const rclcpp::CallbackGroup::SharedPtr & group);
  std::set<std::string> own_node_names() const;
  void show_service(const BridgeMsgServicePayload & payload);
  std::shared_ptr<rclcpp::Node> find_or_create_shadow_node(const std::string & full_node_name);
  rclcpp::CallbackGroup::SharedPtr create_shown_part_group(
    const std::shared_ptr<rclcpp::Node> & owner);
  void attach_shown_part_group(const ShownPart & part);
  void detach_shown_part_group(const ShownPart & part);
  void report_shown_parts(std::chrono::steady_clock::duration sync_time);

  void start_ros_execution();
  void start_worker_thread();

  void worker_loop();

  void parse_and_enqueue(const void * data, std::size_t size);
  void dispatch_bridge_message(const BridgeMsg & msg);

  // Signals every background thread to exit: raises shutdown_requested_ (which
  // the run() and worker_loop() poll), wakes the worker sleeping on the CV,
  // and cancels the ROS 2 executor so its thread can leave spin().
  void request_shutdown();

  void on_signal();
  std::string on_socket_request() const;

  void register_daemon_pubsub_request(const BridgeMsgDaemonPubSubPayload & req);
  void register_daemon_service_request(const BridgeMsgDaemonServicePayload & req);
  bool is_daemon_forced(const std::string & topic_name, BridgeDirection direction) const;
  void create_daemon_forced_bridges();
  void activate_daemon_forced_bridge(
    const std::string & topic_name, const std::string & message_type, const rclcpp::QoS & qos,
    bool is_r2a);

  void check_and_create_pubsub_bridges();
  void check_and_remove_pubsub_bridges();
  void check_and_update_service_bridges();
  void check_and_remove_request_cache();
  void check_and_request_shutdown();

  bool should_create_pubsub_bridge(const std::string & topic_name, BridgeDirection direction) const;
  void create_pubsub_bridge_if_needed(
    const std::string & topic_name, RequestMap & requests, const std::string & message_type,
    BridgeDirection direction);
  static void remove_invalid_requests(const std::string & topic_name, RequestMap & request_map);
};

}  // namespace agnocast
