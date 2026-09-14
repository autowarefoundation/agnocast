#include "agnocast_cie_thread_configurator/announcement_sources.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"

#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace agnocast_cie_thread_configurator
{

AnnouncementSources::AnnouncementSources(
  rclcpp::Node & node, size_t default_domain_id, const std::set<size_t> & domain_ids,
  CallbackGroupCallback on_callback_group, NonRosThreadInfoListener::Callback on_non_ros_thread)
: non_ros_thread_listener_(std::move(on_non_ros_thread), node.get_logger())
{
  // Must stay compatible with the client publishers in agnocastlib
  // (cie_client_utils.cpp). reliable + transient_local so announcements made
  // before the daemon starts are still delivered.
  const auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
  const auto subscribe = [&](rclcpp::Node & target, size_t domain_id) {
    return target.create_subscription<CallbackGroupInfo>(
      k_callback_group_info_topic, qos,
      [on_callback_group, domain_id](CallbackGroupInfo::SharedPtr msg) {
        on_callback_group(domain_id, std::move(msg));
      });
  };

  subscriptions_.push_back(subscribe(node, default_domain_id));

  for (const size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }
    domain_nodes_.push_back(create_node_for_domain(domain_id));
    subscriptions_.push_back(subscribe(*domain_nodes_.back(), domain_id));
    RCLCPP_INFO(node.get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }
}

}  // namespace agnocast_cie_thread_configurator
