#include "agnocast/node/node_interfaces/node_services.hpp"

#include <stdexcept>

namespace agnocast::node_interfaces
{

NodeServices::NodeServices(NodeBase::SharedPtr node_base) : node_base_(std::move(node_base))
{
}

void NodeServices::add_client(
  rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group)
{
  (void)client_base_ptr;
  (void)group;
  throw std::runtime_error(
    "NodeServices::add_client is not supported in agnocast. Use agnocast::create_client instead.");
}

void NodeServices::add_service(
  rclcpp::ServiceBase::SharedPtr service_base_ptr, rclcpp::CallbackGroup::SharedPtr group)
{
  (void)service_base_ptr;
  (void)group;
  throw std::runtime_error(
    "NodeServices::add_service is not supported in agnocast. "
    "Use agnocast::create_service instead.");
}

std::string NodeServices::resolve_service_name(const std::string & name, bool only_expand) const
{
  return node_base_->resolve_topic_or_service_name(name, true, only_expand);
}

}  // namespace agnocast::node_interfaces
