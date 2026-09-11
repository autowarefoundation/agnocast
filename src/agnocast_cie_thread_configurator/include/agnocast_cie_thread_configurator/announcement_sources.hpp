#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"
#include "rclcpp/rclcpp.hpp"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace agnocast_cie_thread_configurator
{

// The two channels through which threads announce themselves to a daemon,
// namely the CallbackGroupInfo topic (one subscription per ROS domain) and
// the non-ROS thread socket. Each extra domain gets its own node, which the
// caller must add to its executor (domain_nodes()).
//
// on_callback_group runs on the executor. on_non_ros_thread runs on the
// listener's private thread until stop() joins it.
class AnnouncementSources
{
public:
  using CallbackGroupInfo = agnocast_cie_config_msgs::msg::CallbackGroupInfo;
  using CallbackGroupCallback =
    std::function<void(size_t /*domain_id*/, CallbackGroupInfo::SharedPtr)>;

  // The default-domain subscription lives on node; a domain_ids entry equal
  // to default_domain_id is skipped.
  AnnouncementSources(
    rclcpp::Node & node, size_t default_domain_id, const std::set<size_t> & domain_ids,
    CallbackGroupCallback on_callback_group, NonRosThreadInfoListener::Callback on_non_ros_thread);

  const std::vector<rclcpp::Node::SharedPtr> & domain_nodes() const { return domain_nodes_; }
  void stop() noexcept { non_ros_thread_listener_.stop(); }

private:
  NonRosThreadInfoListener non_ros_thread_listener_;
  std::vector<rclcpp::Node::SharedPtr> domain_nodes_;
  std::vector<rclcpp::Subscription<CallbackGroupInfo>::SharedPtr> subscriptions_;
};

}  // namespace agnocast_cie_thread_configurator
