#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <memory>
#include <string>

namespace agnocast_cie_thread_configurator
{

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
    "agnocast_cie_thread_configurator_domain_" + std::to_string(domain_id), node_options);
}

}  // namespace agnocast_cie_thread_configurator
