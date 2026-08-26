#include "agnocast/node/node_interfaces/node_graph.hpp"

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace agnocast::node_interfaces
{

NodeGraph::NodeGraph(NodeBase::SharedPtr node_base) : node_base_(std::move(node_base))
{
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_topic_names_and_types(
  bool no_demangle) const
{
  (void)no_demangle;
  throw std::runtime_error("NodeGraph::get_topic_names_and_types is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types() const
{
  throw std::runtime_error("NodeGraph::get_service_names_and_types is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_) const
{
  (void)node_name;
  (void)namespace_;
  throw std::runtime_error(
    "NodeGraph::get_service_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_client_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_) const
{
  (void)node_name;
  (void)namespace_;
  throw std::runtime_error(
    "NodeGraph::get_client_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_publisher_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_, bool no_demangle) const
{
  (void)node_name;
  (void)namespace_;
  (void)no_demangle;
  throw std::runtime_error(
    "NodeGraph::get_publisher_names_and_types_by_node is not supported in agnocast.");
}

std::map<std::string, std::vector<std::string>> NodeGraph::get_subscriber_names_and_types_by_node(
  const std::string & node_name, const std::string & namespace_, bool no_demangle) const
{
  (void)node_name;
  (void)namespace_;
  (void)no_demangle;
  throw std::runtime_error(
    "NodeGraph::get_subscriber_names_and_types_by_node is not supported in agnocast.");
}

namespace
{
std::vector<std::string> query_agnocast_node_names()
{
  std::vector<char> buffer(static_cast<size_t>(MAX_NODE_NUM) * NODE_NAME_BUFFER_SIZE);

  union ioctl_get_node_names_args get_node_names_args = {};
  get_node_names_args.node_name_buffer_addr = reinterpret_cast<uint64_t>(buffer.data());
  get_node_names_args.node_name_buffer_size = MAX_NODE_NUM;
  if (ioctl(agnocast_fd, AGNOCAST_GET_NODE_NAMES_CMD, &get_node_names_args) < 0) {
    // The kmod clamps the buffer to MAX_NODE_NUM either way, so -ENOBUFS means the graph is too
    // large to report, not a broken kmod state -- and so, unlike the other ioctl wrappers, this
    // one must not take the caller down.
    if (errno == ENOBUFS) {
      RCLCPP_ERROR(
        logger,
        "AGNOCAST_GET_NODE_NAMES_CMD failed: more than MAX_NODE_NUM (%d) agnocast nodes in this "
        "IPC namespace and ROS_DOMAIN_ID. get_node_names() reports the calling node only.",
        MAX_NODE_NUM);
      return {};
    }
    RCLCPP_ERROR(logger, "AGNOCAST_GET_NODE_NAMES_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::vector<std::string> node_names;
  node_names.reserve(get_node_names_args.ret_node_num);

  for (uint32_t i = 0; i < get_node_names_args.ret_node_num; ++i) {
    node_names.emplace_back(&buffer[static_cast<size_t>(i) * NODE_NAME_BUFFER_SIZE]);
  }

  return node_names;
}
}  // namespace

// Duplicate names survive: `rclcpp` reports a name once per node that carries it, and the kmod
// likewise counts two same-named nodes in different processes as two.
std::vector<std::string> NodeGraph::get_node_names() const
{
  std::vector<std::string> node_names = query_agnocast_node_names();

  // The kmod only knows nodes that own an endpoint, but rclcpp lists the calling node even
  // when it owns none. Skipping a name already listed hides a same-named node in another process.
  std::string self_name = node_base_->get_fully_qualified_name();
  if (std::find(node_names.begin(), node_names.end(), self_name) == node_names.end()) {
    node_names.push_back(std::move(self_name));
  }

  return node_names;
}

std::vector<std::tuple<std::string, std::string, std::string>>
NodeGraph::get_node_names_with_enclaves() const
{
  throw std::runtime_error("NodeGraph::get_node_names_with_enclaves is not supported in agnocast.");
}

std::vector<std::pair<std::string, std::string>> NodeGraph::get_node_names_and_namespaces() const
{
  throw std::runtime_error(
    "NodeGraph::get_node_names_and_namespaces is not supported in agnocast.");
}

// Counts agnocast and ROS 2 endpoints, excluding the ones created by bridges.
// agnocast::Node::count_publishers()/count_subscribers() delegate here, so the resolution and the
// bridge bookkeeping live in one place.
//
// Note that count_subscribers() does not see agnocast subscribers in the caller's own process:
// get_subscription_count_core() reports ret_other_process_subscriber_num, because its original
// caller (agnocast::Publisher::get_subscription_count()) asks how many peers receive a published
// message. count_publishers() has no such split.
size_t NodeGraph::count_publishers(const std::string & topic_name) const
{
  return get_publisher_count_core(node_base_->resolve_topic_or_service_name(topic_name, false));
}

size_t NodeGraph::count_subscribers(const std::string & topic_name) const
{
  return get_subscription_count_core(node_base_->resolve_topic_or_service_name(topic_name, false));
}

const rcl_guard_condition_t * NodeGraph::get_graph_guard_condition() const
{
  throw std::runtime_error("NodeGraph::get_graph_guard_condition is not supported in agnocast.");
}

// No-op rather than throwing: agnocast has no graph events, so there is nothing to notify.
// rclcpp utilities call these unconditionally when entities are created or the context shuts
// down, and throwing there would break otherwise working code.
void NodeGraph::notify_graph_change()
{
}

void NodeGraph::notify_shutdown()
{
}

rclcpp::Event::SharedPtr NodeGraph::get_graph_event()
{
  throw std::runtime_error("NodeGraph::get_graph_event is not supported in agnocast.");
}

void NodeGraph::wait_for_graph_change(
  rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout)
{
  (void)event;
  (void)timeout;
  throw std::runtime_error("NodeGraph::wait_for_graph_change is not supported in agnocast.");
}

size_t NodeGraph::count_graph_users() const
{
  throw std::runtime_error("NodeGraph::count_graph_users is not supported in agnocast.");
}

std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_publishers_info_by_topic(
  const std::string & topic_name, bool no_mangle) const
{
  (void)topic_name;
  (void)no_mangle;
  throw std::runtime_error("NodeGraph::get_publishers_info_by_topic is not supported in agnocast.");
}

std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_subscriptions_info_by_topic(
  const std::string & topic_name, bool no_mangle) const
{
  (void)topic_name;
  (void)no_mangle;
  throw std::runtime_error(
    "NodeGraph::get_subscriptions_info_by_topic is not supported in agnocast.");
}

// rclcpp 28+ (Jazzy) added these methods to NodeGraphInterface.
#if RCLCPP_VERSION_MAJOR >= 28
size_t NodeGraph::count_clients(const std::string & service_name) const
{
  (void)service_name;
  throw std::runtime_error("NodeGraph::count_clients is not supported in agnocast.");
}

size_t NodeGraph::count_services(const std::string & service_name) const
{
  (void)service_name;
  throw std::runtime_error("NodeGraph::count_services is not supported in agnocast.");
}
#endif

}  // namespace agnocast::node_interfaces
