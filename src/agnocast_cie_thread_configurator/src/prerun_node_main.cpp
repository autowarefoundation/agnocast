#include "agnocast_cie_thread_configurator/prerun_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PrerunNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor->add_node(node);
    for (const auto & sub_node : node->get_domain_nodes()) {
      executor->add_node(sub_node);
    }

    executor->spin();

    node->dump_yaml_config(std::filesystem::current_path());
  } catch (const std::exception & e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
