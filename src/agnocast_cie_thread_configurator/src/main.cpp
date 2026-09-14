#include "agnocast_cie_thread_configurator/thread_configurator_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ThreadConfiguratorNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor->add_node(node);
    for (const auto & domain_node : node->get_domain_nodes()) {
      executor->add_node(domain_node);
    }

    executor->spin();

    node->stop();
    node->print_all_unapplied();
  } catch (const std::exception & e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
