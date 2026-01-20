#pragma once

#include "agnocast/node/node_interfaces/node_base.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/service.hpp"

#include <memory>
#include <string>

namespace agnocast::node_interfaces
{

class NodeServices : public rclcpp::node_interfaces::NodeServicesInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeServices>;
  using WeakPtr = std::weak_ptr<NodeServices>;

  explicit NodeServices(NodeBase::SharedPtr node_base);

  virtual ~NodeServices() = default;

  NodeServices(const NodeServices &) = delete;
  NodeServices & operator=(const NodeServices &) = delete;

  void add_client(
    rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group) override;

  void add_service(
    rclcpp::ServiceBase::SharedPtr service_base_ptr,
    rclcpp::CallbackGroup::SharedPtr group) override;

  std::string resolve_service_name(
    const std::string & name, bool only_expand = false) const override;

private:
  NodeBase::SharedPtr node_base_;
};

}  // namespace agnocast::node_interfaces
