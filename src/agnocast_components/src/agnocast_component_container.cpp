#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include <glog/logging.h>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  google::InstallFailureSignalHandler();

  try {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp_components::ComponentManager>(
      std::weak_ptr<rclcpp::Executor>(), "ComponentManager", options);

    const int get_next_timeout_ms = node->get_parameter_or("get_next_timeout_ms", 50);

    auto executor = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>(
      rclcpp::ExecutorOptions{}, get_next_timeout_ms);

    node->set_executor(executor);
    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();
  } catch (rclcpp_components::ComponentManagerException & ex) {
    std::cerr << "Exception caught in main: " << ex.what() << std::endl;
    close(agnocast::agnocast_fd);
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unknown exception caught in main: " << std::endl;
    close(agnocast::agnocast_fd);
    return EXIT_FAILURE;
  }

  return 0;
}
