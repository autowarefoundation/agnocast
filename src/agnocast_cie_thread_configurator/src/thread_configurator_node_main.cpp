#include "agnocast_cie_thread_configurator/thread_configurator_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ThreadConfiguratorNode>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);
  for (const auto & domain_node : node->get_domain_nodes()) {
    executor->add_node(domain_node);
  }

  executor->spin();

  if (!node->has_configured_once()) {
    node->print_all_unapplied();
  }

  rclcpp::shutdown();
  return 0;
}
