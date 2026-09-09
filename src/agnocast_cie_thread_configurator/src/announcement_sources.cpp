#include "agnocast_cie_thread_configurator/announcement_sources.hpp"

#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace agnocast_cie_thread_configurator
{

namespace
{

constexpr const char * k_callback_group_info_topic =
  "/agnocast_cie_thread_configurator/callback_group_info";

// Must stay compatible with the client publishers in agnocastlib
// (cie_client_utils.cpp). reliable + transient_local so announcements made
// before the daemon starts are still delivered.
rclcpp::QoS callback_group_info_qos()
{
  return rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
}

}  // namespace

AnnouncementSources::AnnouncementSources(
  rclcpp::Node & node, size_t default_domain_id, const std::set<size_t> & domain_ids,
  CallbackGroupCallback on_callback_group, NonRosThreadInfoListener::Callback on_non_ros_thread)
: non_ros_thread_listener_(std::move(on_non_ros_thread), node.get_logger())
{
  const auto subscribe = [&on_callback_group](rclcpp::Node & target, size_t domain_id) {
    return target.create_subscription<CallbackGroupInfo>(
      k_callback_group_info_topic, callback_group_info_qos(),
      [on_callback_group, domain_id](CallbackGroupInfo::SharedPtr msg) {
        on_callback_group(domain_id, std::move(msg));
      });
  };

  subscriptions_.push_back(subscribe(node, default_domain_id));

  for (const size_t domain_id : domain_ids) {
    if (domain_id == default_domain_id) {
      continue;
    }
    auto domain_node = create_node_for_domain(domain_id);
    subscriptions_.push_back(subscribe(*domain_node, domain_id));
    domain_nodes_.push_back(std::move(domain_node));
    RCLCPP_INFO(node.get_logger(), "Created subscription for domain ID: %zu", domain_id);
  }
}

const std::vector<rclcpp::Node::SharedPtr> & AnnouncementSources::domain_nodes() const
{
  return domain_nodes_;
}

void AnnouncementSources::stop() noexcept
{
  non_ros_thread_listener_.stop();
}

}  // namespace agnocast_cie_thread_configurator
