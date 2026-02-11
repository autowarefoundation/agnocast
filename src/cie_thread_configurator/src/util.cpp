#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace cie_thread_configurator
{

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
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
    int64_t period;
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

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group, rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & agnocast_topics)
{
  return create_callback_group_id(group, node->get_node_base_interface(), agnocast_topics);
}

rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr create_client_publisher()
{
  static int idx = 1;

  auto node = std::make_shared<rclcpp::Node>(
    "client_node" + std::to_string(idx++), "/cie_thread_configurator");
  auto publisher = node->create_publisher<cie_config_msgs::msg::CallbackGroupInfo>(
    "/cie_thread_configurator/callback_group_info", rclcpp::QoS(1000).keep_all());
  return publisher;
}

void publish_callback_group_info(
  const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id)
{
  if (publisher->get_subscription_count() == 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("cie_thread_configurator"),
      "No subscriber for CallbackGroupInfo. "
      "Please run thread_configurator_node if you want to configure thread scheduling.");
    return;
  }

  auto message = std::make_shared<cie_config_msgs::msg::CallbackGroupInfo>();
  message->thread_id = tid;
  message->callback_group_id = callback_group_id;
  publisher->publish(*message);
}

std::map<std::string, std::string> get_hardware_info()
{
  std::map<std::string, std::string> hw_info;

  // Execute lscpu command and capture output
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("/usr/bin/lscpu", "r"), pclose);

  if (!pipe) {
    return hw_info;
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  // Parse lscpu output
  std::istringstream iss(result);
  std::string line;

  while (std::getline(iss, line)) {
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string key = line.substr(0, colon_pos);
    std::string value = line.substr(colon_pos + 1);

    // Trim leading/trailing whitespace from value
    size_t start = value.find_first_not_of(" \t");
    size_t end = value.find_last_not_of(" \t\r\n");
    if (start != std::string::npos && end != std::string::npos) {
      value = value.substr(start, end - start + 1);
    }

    // Store relevant hardware information
    if (key == "Model name") {
      hw_info["model_name"] = value;
    } else if (key == "CPU family") {
      hw_info["cpu_family"] = value;
    } else if (key == "Model") {
      hw_info["model"] = value;
    } else if (key == "Thread(s) per core") {
      hw_info["threads_per_core"] = value;
    } else if (key == "Frequency boost") {
      hw_info["frequency_boost"] = value;
    } else if (key == "CPU max MHz") {
      hw_info["cpu_max_mhz"] = value;
    } else if (key == "CPU min MHz") {
      hw_info["cpu_min_mhz"] = value;
    }
  }

  return hw_info;
}

size_t get_default_domain_id()
{
  const char * env_value = std::getenv("ROS_DOMAIN_ID");
  if (env_value != nullptr) {
    return static_cast<size_t>(std::stoul(env_value));
  }
  return 0;  // default domain ID
}

rclcpp::Node::SharedPtr create_node_for_domain(size_t domain_id)
{
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(domain_id);
  init_options.auto_initialize_logging(false);  // logging is already initialized
  context->init(0, nullptr, init_options);

  rclcpp::NodeOptions node_options;
  node_options.context(context);

  return std::make_shared<rclcpp::Node>(
    "cie_thread_configurator_domain_" + std::to_string(domain_id), node_options);
}

}  // namespace cie_thread_configurator
