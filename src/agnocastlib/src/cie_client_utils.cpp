#include "agnocast/cie_client_utils.hpp"

#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <unistd.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>

namespace agnocast
{

constexpr size_t CIE_QOS_DEPTH = 5000;

std::string create_callback_group_id(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
  const std::vector<std::string> & agnocast_topics)
{
  std::string ns = std::string(node->get_namespace());
  if (ns != "/") {
    ns = ns + "/";
  }

  std::vector<std::string> entries;

  auto sub_func = [&entries](const rclcpp::SubscriptionBase::SharedPtr & sub) {
    entries.push_back("Subscription(" + std::string(sub->get_topic_name()) + ")");
  };

  auto service_func = [&entries](const rclcpp::ServiceBase::SharedPtr & service) {
    entries.push_back("Service(" + std::string(service->get_service_name()) + ")");
  };

  auto client_func = [&entries](const rclcpp::ClientBase::SharedPtr & client) {
    entries.push_back("Client(" + std::string(client->get_service_name()) + ")");
  };

  auto timer_func = [&entries](const rclcpp::TimerBase::SharedPtr & timer) {
    std::shared_ptr<const rcl_timer_t> timer_handle = timer->get_timer_handle();
    int64_t period = 0;
    rcl_ret_t ret = rcl_timer_get_period(timer_handle.get(), &period);
    (void)ret;

    entries.push_back("Timer(" + std::to_string(period) + ")");
  };

  auto waitable_func = [](auto &&) {};

  group->collect_all_ptrs(sub_func, service_func, client_func, timer_func, waitable_func);

  // Agnocast Callbacks
  static constexpr size_t SRV_REQUEST_PREFIX_LEN = sizeof("/AGNOCAST_SRV_REQUEST") - 1;    // 21
  static constexpr size_t SRV_RESPONSE_PREFIX_LEN = sizeof("/AGNOCAST_SRV_RESPONSE") - 1;  // 22
  for (const auto & topic : agnocast_topics) {
    if (topic.rfind("/AGNOCAST_SRV_REQUEST", 0) == 0) {
      entries.push_back("Service(" + topic.substr(SRV_REQUEST_PREFIX_LEN) + ")");
    } else if (topic.rfind("/AGNOCAST_SRV_RESPONSE", 0) == 0) {
      auto service_part = topic.substr(SRV_RESPONSE_PREFIX_LEN);
      auto sep_pos = service_part.find("_SEP_");
      entries.push_back("Client(" + service_part.substr(0, sep_pos) + ")");
    } else {
      entries.push_back("Subscription(" + topic + ")");
    }
  }

  std::sort(entries.begin(), entries.end());

  std::stringstream ss;
  ss << ns << node->get_name();
  for (const auto & entry : entries) {
    ss << "@" << entry;
  }

  return ss.str();
}

rclcpp::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_rclcpp_client_publisher()
{
  rclcpp::NodeOptions options;
  // Disable global arguments so that global "__node" remapping cannot override this name
  // and cause duplicate node names for these per-client nodes.
  options.use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>(
    "client_node_" + std::to_string(getpid()), "/agnocast_cie_thread_configurator", options);
  auto publisher = node->create_publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
    "/agnocast_cie_thread_configurator/callback_group_info",
    rclcpp::QoS(CIE_QOS_DEPTH).keep_all().reliable().transient_local());
  return publisher;
}

agnocast::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_agnocast_client_publisher()
{
  rclcpp::NodeOptions options;
  // Disable global arguments so that global "__node" remapping cannot override this name
  // and cause duplicate node names for these per-client nodes.
  options.use_global_arguments(false);
  auto node = std::make_shared<agnocast::Node>(
    "agnocast_client_node_" + std::to_string(getpid()), "/agnocast_cie_thread_configurator",
    options);
  auto publisher = node->create_publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>(
    // Note: agnocast Publisher does not support keep_all(), so KeepLast is used here
    // (unlike the rclcpp variant which uses keep_all()).
    "/agnocast_cie_thread_configurator/callback_group_info",
    rclcpp::QoS(rclcpp::KeepLast(CIE_QOS_DEPTH)).transient_local());
  return publisher;
}

void publish_callback_group_info(
  const rclcpp::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id)
{
  auto message = std::make_shared<agnocast_cie_config_msgs::msg::CallbackGroupInfo>();
  message->thread_id = tid;
  message->callback_group_id = callback_group_id;
  publisher->publish(*message);
}

void publish_callback_group_info(
  const agnocast::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr &
    publisher,
  int64_t tid, const std::string & callback_group_id)
{
  auto message = publisher->borrow_loaned_message();
  message->thread_id = tid;
  message->callback_group_id = callback_group_id;
  publisher->publish(std::move(message));
}

}  // namespace agnocast
