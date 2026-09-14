#pragma once

#include "agnocast_cie_thread_configurator/announcement_sources.hpp"
#include "agnocast_cie_thread_configurator/sched_policy.hpp"
#include "agnocast_cie_thread_configurator/thread_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"
#include "agnocast_cie_config_msgs/srv/reapply_config.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class ThreadConfiguratorNode : public rclcpp::Node
{
  using CallbackGroupEntry = agnocast_cie_thread_configurator::CallbackGroupEntry;
  using NonRosThreadEntry = agnocast_cie_thread_configurator::NonRosThreadEntry;
  using KernelThreadEntry = agnocast_cie_thread_configurator::KernelThreadEntry;
  using IrqEntry = agnocast_cie_thread_configurator::IrqEntry;
  using SchedAttrs = agnocast_cie_thread_configurator::SchedAttrs;
  using SchedPolicy = agnocast_cie_thread_configurator::SchedPolicy;

  // A YAML entry plus what has been observed at runtime for it. Owned in the
  // two vectors below, which the id_to_*/node_to_* maps index into.
  struct TrackedCallbackGroup
  {
    CallbackGroupEntry entry;
    int64_t thread_id = -1;  // -1 until announced; stays -1 for wildcard entries
    // Full incoming callback_group_id -> last announced tid; wildcard
    // ("<node name>/*") entries only. For such entries `applied` means "at
    // least one matched instance has been configured". std::map for
    // deterministic iteration order in the reapply response arrays.
    std::map<std::string, int64_t> matched_tids;
    bool applied = false;  // true once issue_syscalls() has succeeded
  };
  struct TrackedNonRosThread
  {
    NonRosThreadEntry entry;
    int64_t thread_id = -1;  // -1 until announced
    bool applied = false;    // true once issue_syscalls() has succeeded
  };

  // Concurrency:
  // - callback_group_configs_ / id_to_callback_group_config_ /
  //   node_to_wildcard_config_ (incl. each entry's matched_tids): written by
  //   the subscription callbacks AND the reapply service handler, all on the
  //   same SingleThreadedExecutor — no mutex needed.
  // - non_ros_thread_configs_ / id_to_non_ros_thread_config_: written by both
  //   the NonRosThreadInfoListener reader thread and the reapply handler;
  //   all access must hold non_ros_state_mutex_.
  // - kernel_thread_configs_ / irq_configs_: written only by the constructor
  //   (pre-spin) and the reapply handler, so no mutex is needed.
  // - print_all_unapplied(): called only after stop() + executor return, so
  //   reads need no lock.

public:
  explicit ThreadConfiguratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ThreadConfiguratorNode();
  void stop() noexcept;
  void print_all_unapplied();

  const std::vector<rclcpp::Node::SharedPtr> & get_domain_nodes() const;

private:
  // One kernel-thread/IRQ apply pass. Keys: "<comm>:<tid>" (applied/failed)
  // and "<comm>" (skipped) for kernel threads; the decimal IRQ number for
  // IRQs. "applied" = ensured: an already-matching entry counts without
  // syscalls (compare-before-set).
  struct SectionApplyOutcome
  {
    std::vector<std::string> applied;
    std::vector<std::string> failed;
    std::vector<std::string> skipped;
  };

  void validate_hardware_info(const YAML::Node & yaml);
  void validate_rt_throttling(const YAML::Node & yaml);
  bool set_affinity_by_cgroup(int64_t thread_id, const std::vector<int> & cpus);
  // thread_id is passed explicitly because a wildcard entry applies to many
  // threads (one per matched_tids element). attrs.policy must be set.
  bool issue_syscalls(const std::string & thread_str, const SchedAttrs & attrs, int64_t thread_id);
  // Only Deadline takes the cgroup-based affinity path; any other value,
  // including nullopt (an observed policy with no YAML name), uses
  // sched_setaffinity.
  bool issue_affinity_syscalls(
    const std::string & thread_str, std::optional<SchedPolicy> policy,
    const std::vector<int> & affinity, int64_t thread_id);
  SectionApplyOutcome apply_kernel_thread_configs();
  SectionApplyOutcome apply_irq_configs() const;
  // Sole logging point for the write path: emits errno-specific guidance on
  // any open/write/short-write failure (returning false) and the success line.
  bool write_irq_affinity_file(const IrqEntry & config) const;
  void callback_group_callback(
    size_t domain_id, const agnocast_cie_config_msgs::msg::CallbackGroupInfo::SharedPtr msg);
  void non_ros_thread_callback(agnocast_cie_thread_configurator::NonRosThreadInfo info);

  void on_reapply_config_request(
    const std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Request> request,
    std::shared_ptr<agnocast_cie_config_msgs::srv::ReapplyConfig::Response> response);

  rclcpp::Service<agnocast_cie_config_msgs::srv::ReapplyConfig>::SharedPtr reapply_service_;

  std::vector<TrackedCallbackGroup> callback_group_configs_;
  // (domain_id, callback_group_id) -> exact entries only
  std::map<std::pair<size_t, std::string>, TrackedCallbackGroup *> id_to_callback_group_config_;
  // (domain_id, wildcard_prefix) -> wildcard ("<node>/*") entries only
  std::map<std::pair<size_t, std::string>, TrackedCallbackGroup *> node_to_wildcard_config_;

  std::vector<TrackedNonRosThread> non_ros_thread_configs_;
  // thread_name -> entry
  std::map<std::string, TrackedNonRosThread *> id_to_non_ros_thread_config_;

  std::vector<KernelThreadEntry> kernel_thread_configs_;
  std::vector<IrqEntry> irq_configs_;

  std::atomic<int> unapplied_num_{0};
  std::atomic<int> cgroup_num_{0};
  std::atomic<bool> configured_at_least_once_{false};

  const std::string config_file_;
  const size_t default_domain_id_;
  std::mutex non_ros_state_mutex_;
  // Declared last so it is destroyed first. The listener thread must be
  // joined before any state its callback touches goes away.
  std::unique_ptr<agnocast_cie_thread_configurator::AnnouncementSources> sources_;
};
