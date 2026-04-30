#include "agnocast_cie_thread_configurator/prerun_node.hpp"
#include "agnocast_cie_thread_configurator/thread_configurator_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

static std::vector<int64_t> parse_domain_ids(const std::string & domains_str)
{
  std::vector<int64_t> domain_ids;
  std::stringstream ss(domains_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      try {
        domain_ids.push_back(static_cast<int64_t>(std::stol(token)));
      } catch (const std::exception & e) {
        std::cerr << "[WARN] Invalid domain ID value: " << token << ". Skipping." << std::endl;
      }
    }
  }
  return domain_ids;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  bool prerun_mode = false;
  std::vector<int64_t> domain_ids;
  std::string config_filename;

  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--prerun") {
      prerun_mode = true;
    } else if (args[i] == "--domains" && i + 1 < args.size()) {
      domain_ids = parse_domain_ids(args[i + 1]);
      ++i;
    } else if (args[i] == "--config-file" && i + 1 < args.size()) {
      config_filename = args[i + 1];
      ++i;
    }
  }

  if (prerun_mode || !domain_ids.empty() || !config_filename.empty()) {
    std::cerr << "[DEPRECATED] CLI arguments (--prerun, --domains, --config-file) are deprecated. "
              << "Run prerun_node directly with ROS parameters or use the launch file "
              << "(thread_configurator.launch.xml) instead." << std::endl;
  }

  // Backward-compatible CLI path: translates deprecated CLI args into ROS parameter overrides.
  try {
    if (prerun_mode) {
      std::cout << "prerun mode" << std::endl;

      rclcpp::NodeOptions options;
      if (!domain_ids.empty()) {
        options.append_parameter_override("domains", domain_ids);
      }

      auto node = std::make_shared<PrerunNode>(options);
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

      executor->add_node(node);
      for (const auto & sub_node : node->get_domain_nodes()) {
        executor->add_node(sub_node);
      }

      executor->spin();

      node->dump_yaml_config(std::filesystem::current_path());
    } else {
      rclcpp::NodeOptions options;
      if (!config_filename.empty()) {
        options.append_parameter_override("config_file", config_filename);
      }

      auto node = std::make_shared<ThreadConfiguratorNode>(options);
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

      executor->add_node(node);
      for (const auto & domain_node : node->get_domain_nodes()) {
        executor->add_node(domain_node);
      }

      executor->spin();

      node->print_all_unapplied();
    }
  } catch (const std::exception & e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
