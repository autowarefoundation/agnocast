#include "agnocast_cie_thread_configurator/prerun_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>

static std::set<size_t> parse_domain_ids(const std::string & domains_str)
{
  // https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html#choosing-a-domain-id-short-version
  constexpr size_t max_domain_id = 101;

  std::set<size_t> domain_ids;
  std::stringstream ss(domains_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      try {
        size_t domain_id = std::stoul(token);
        if (domain_id > max_domain_id) {
          std::cerr << "[WARN] Domain ID " << domain_id << " exceeds maximum valid value ("
                    << max_domain_id << "). Skipping." << std::endl;
          continue;
        }
        domain_ids.insert(domain_id);
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

  auto bootstrap_node = std::make_shared<rclcpp::Node>("prerun_node_bootstrap");
  const std::string domains = bootstrap_node->declare_parameter<std::string>("domains", "");
  const std::set<size_t> domain_ids = parse_domain_ids(domains);

  std::cout << "prerun mode" << std::endl;

  auto node = std::make_shared<PrerunNode>(domain_ids);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);
  for (const auto & sub_node : node->get_domain_nodes()) {
    executor->add_node(sub_node);
  }

  executor->spin();

  node->dump_yaml_config(std::filesystem::current_path());

  rclcpp::shutdown();
  return 0;
}
