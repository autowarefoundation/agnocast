#include "cie_thread_configurator/cie_thread_configurator.hpp"
#include "cie_thread_configurator/prerun_node.hpp"
#include "cie_thread_configurator/thread_configurator_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <filesystem>
#include <memory>
#include <string>

static bool validate_hardware_info(const YAML::Node & yaml)
{
  YAML::Node yaml_hw_info = yaml["hardware_info"];
  auto current_hw_info = cie_thread_configurator::get_hardware_info();

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

  // Report results
  if (!all_match) {
    std::cerr << "[ERROR] Hardware validation failed with the following "
                 "mismatches:"
              << std::endl;
    for (const auto & mismatch : mismatches) {
      std::cerr << "  - " << mismatch << std::endl;
    }
  } else {
    std::cout << "[INFO] Hardware validation successful. Configuration matches "
                 "this system."
              << std::endl;
  }

  return all_match;
}

static void spin_thread_configurator_node(const std::string & config_filename)
{
  YAML::Node config;

  try {
    config = YAML::LoadFile(config_filename);
  } catch (const std::exception & e) {
    std::cerr << "Error reading the YAML file: " << e.what() << std::endl;
    return;
  }

  // Validate hardware information if present in YAML
  if (config["hardware_info"]) {
    if (!validate_hardware_info(config)) {
      std::cerr << "[ERROR] Hardware information validation failed. The "
                   "configuration may not match this system."
                << std::endl;
      return;
    }
  } else {
    std::cout << "[WARN] No hardware_info section found in configuration file. "
                 "Skipping hardware validation."
              << std::endl;
  }

  std::cout << config["callback_groups"] << std::endl;

  auto node = std::make_shared<ThreadConfiguratorNode>(config);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);

  while (rclcpp::ok() && !node->all_applied()) {
    executor->spin_once();
  }

  if (node->all_applied()) {
    if (node->exist_deadline_config()) {
      RCLCPP_INFO(node->get_logger(), "Apply sched deadline?");
      std::cin.get();

      node->apply_deadline_configs();

      RCLCPP_INFO(
        node->get_logger(),
        "Success: All of the configurations are applied."
        "\nPress enter to exit and remove cgroups, if there are "
        "SCHED_DEADLINE tasks:");
      std::cin.get();
    }
    RCLCPP_INFO(node->get_logger(), "Success: All of the configurations are applied.");
  } else {
    node->print_all_unapplied();
  }
}

static void spin_prerun_node()
{
  std::cout << "prerun mode" << std::endl;

  auto node = std::make_shared<PrerunNode>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  node->dump_yaml_config(std::filesystem::current_path());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  bool prerun_mode = false;
  for (const auto & arg : args) {
    if (arg == "--prerun") {
      prerun_mode = true;
    }
  }

  if (prerun_mode) {
    spin_prerun_node();
  } else {
    bool next = false;
    std::string filename;

    for (const auto & arg : args) {
      if (next) {
        filename = arg;
        break;
      }

      if (arg == std::string("--config-file")) {
        next = true;
      }
    }

    spin_thread_configurator_node(filename);
  }

  rclcpp::shutdown();
  return 0;
}
