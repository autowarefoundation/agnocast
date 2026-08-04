#pragma once

#include "agnocast/node/node_interfaces/node_base.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace agnocast::node_interfaces
{

class NodeGraph : public rclcpp::node_interfaces::NodeGraphInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeGraph>;
  using WeakPtr = std::weak_ptr<NodeGraph>;

  explicit NodeGraph(NodeBase::SharedPtr node_base);

  virtual ~NodeGraph() = default;

  size_t count_publishers(const std::string & topic_name) const override;
  size_t count_subscribers(const std::string & topic_name) const override;

  // ===== No-op methods =====
  void notify_graph_change() override;
  void notify_shutdown() override;

  // ===== Not supported methods (throw runtime_error) =====
  // Reporting the agnocast nodes requires the kmod to expose them, which it does not do yet.
  std::vector<std::string> get_node_names() const override;
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types(
    bool no_demangle = false) const override;
  std::map<std::string, std::vector<std::string>> get_service_names_and_types() const override;
  std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_) const override;
  std::map<std::string, std::vector<std::string>> get_client_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_) const override;
  std::map<std::string, std::vector<std::string>> get_publisher_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_,
    bool no_demangle = false) const override;
  std::map<std::string, std::vector<std::string>> get_subscriber_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_,
    bool no_demangle = false) const override;
  std::vector<std::tuple<std::string, std::string, std::string>> get_node_names_with_enclaves()
    const override;
  std::vector<std::pair<std::string, std::string>> get_node_names_and_namespaces() const override;
  const rcl_guard_condition_t * get_graph_guard_condition() const override;
  rclcpp::Event::SharedPtr get_graph_event() override;
  void wait_for_graph_change(
    rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) override;
  size_t count_graph_users() const override;
  std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const override;
  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
    const std::string & topic_name, bool no_mangle = false) const override;

private:
  NodeBase::SharedPtr node_base_;
};

}  // namespace agnocast::node_interfaces
