#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"
#include "agnocast_cie_thread_configurator/thread_configurator_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

static bool validate_hardware_info(const YAML::Node & yaml)
{
  YAML::Node yaml_hw_info = yaml["hardware_info"];
  auto current_hw_info = agnocast_cie_thread_configurator::get_hardware_info();

  bool all_match = true;
  std::vector<std::string> mismatches;

  for (const auto & [key, current_value] : current_hw_info) {
    if (!yaml_hw_info[key]) {
      continue;
    }

    std::string yaml_value = yaml_hw_info[key].as<std::string>();
    if (yaml_value != current_value) {
      all_match = false;
      mismatches.push_back(key + ": expected '" + yaml_value + "', got '" + current_value + "'");
    }
  }

  if (!all_match) {
    std::cerr << "[ERROR] Hardware validation failed with the following mismatches:" << std::endl;
    for (const auto & mismatch : mismatches) {
      std::cerr << "  - " << mismatch << std::endl;
    }
  } else {
    std::cout << "[INFO] Hardware validation successful. Configuration matches this system."
              << std::endl;
  }

  return all_match;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto bootstrap_node = std::make_shared<rclcpp::Node>("thread_configurator_bootstrap");
  const std::string config_filename =
    bootstrap_node->declare_parameter<std::string>("config_file", "");

  if (config_filename.empty()) {
    std::cerr << "[ERROR] Parameter 'config_file' must be provided." << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_filename);
  } catch (const std::exception & e) {
    std::cerr << "Error reading the YAML file: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  if (config["hardware_info"]) {
    if (!validate_hardware_info(config)) {
      std::cerr << "[ERROR] Hardware information validation failed. The configuration may not "
                   "match this system."
                << std::endl;
      rclcpp::shutdown();
      return 1;
    }
  } else {
    std::cout << "[WARN] No hardware_info section found in configuration file. Skipping hardware "
                 "validation."
              << std::endl;
  }

  std::cout << "rt_throttling:" << std::endl;
  std::cout << config["rt_throttling"] << std::endl;
  std::cout << "callback_groups:" << std::endl;
  std::cout << config["callback_groups"] << std::endl;
  std::cout << "non_ros_threads:" << std::endl;
  std::cout << config["non_ros_threads"] << std::endl;

  auto node = std::make_shared<ThreadConfiguratorNode>(config);
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
