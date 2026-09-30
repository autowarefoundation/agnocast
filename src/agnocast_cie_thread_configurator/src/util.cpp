#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "rcl/domain_id.h"
#include "rcl/error_handling.h"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <memory>
#include <string>

namespace agnocast_cie_thread_configurator
{

size_t get_default_domain_id()
{
  // Seed with 0 so unset/empty ROS_DOMAIN_ID (rcl leaves the out-param
  // unchanged) yields the same default domain rcl ultimately uses.
  size_t domain_id = 0;
  const rcl_ret_t ret = rcl_get_default_domain_id(&domain_id);
  if (ret != RCL_RET_OK) {
    rcl_reset_error();
    const char * env_value = std::getenv("ROS_DOMAIN_ID");
    RCLCPP_WARN(
      rclcpp::get_logger("cie_thread_configurator"),
      "Invalid ROS_DOMAIN_ID '%s'; falling back to default domain ID 0",
      env_value != nullptr ? env_value : "");
    return 0;
  }
  // rcl parses ROS_DOMAIN_ID with strtoul. A leading '-' becomes ULONG_MAX /
  // RCL_DEFAULT_DOMAIN_ID with RCL_RET_OK; rclcpp then treats that sentinel as
  // the default domain (0). Normalize so callers see the same domain id.
  if (domain_id == RCL_DEFAULT_DOMAIN_ID) {
    const char * env_value = std::getenv("ROS_DOMAIN_ID");
    RCLCPP_WARN(
      rclcpp::get_logger("cie_thread_configurator"),
      "Invalid ROS_DOMAIN_ID '%s'; falling back to default domain ID 0",
      env_value != nullptr ? env_value : "");
    return 0;
  }
  return domain_id;
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
