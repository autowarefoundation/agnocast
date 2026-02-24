#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

struct R2AServiceBridgeResult
{
  std::shared_ptr<void> entity_handle;
  rclcpp::CallbackGroup::SharedPtr ros_srv_cb_group;
  rclcpp::CallbackGroup::SharedPtr agno_client_cb_group;
};

R2AServiceBridgeResult create_r2a_service_bridge(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const rclcpp::QoS & srv_qos)
{
  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;

  auto srv_cb_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto client_cb_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  auto agno_client = agnocast::create_client<ServiceT>(
    node.get(), service_name, rclcpp::ServicesQoS(), client_cb_group);

  auto ros_srv = node->create_service<ServiceT>(
    service_name,
    [node, agno_client](
      const ServiceT::Request::SharedPtr ros_req, ServiceT::Response::SharedPtr ros_res) {
      auto agno_req = agno_client->borrow_loaned_request();
      *agno_req = *ros_req;

      RCLCPP_INFO(node->get_logger(), "Forwarding a request");
      auto future = agno_client->async_send_request(std::move(agno_req));

      // NOTE: One executor thread will be occupied until the future is resolved, which may take a
      // substantial amount of time depending on the service.
      auto agno_res = future.get();
      *ros_res = *agno_res;

      RCLCPP_INFO(node->get_logger(), "Returning a response");
    },
    srv_qos.get_rmw_qos_profile(), srv_cb_group);

  return {ros_srv, srv_cb_group, client_cb_group};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("r2a_service_brigde");
  // TODO(bdm-k): Write a note explaining why a single-threaded executor should not be used.
  agnocast::MultiThreadedAgnocastExecutor executor;

  auto result = create_r2a_service_bridge(node, "sum_int_array", rclcpp::ServicesQoS());
  executor.add_callback_group(result.ros_srv_cb_group, node->get_node_base_interface(), true);
  executor.add_callback_group(result.agno_client_cb_group, node->get_node_base_interface(), true);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
