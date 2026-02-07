#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace agnocast
{

PerformanceBridgeLoader::PerformanceBridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
}

PerformanceBridgeLoader::~PerformanceBridgeLoader()
{
  for (auto & pair : loaded_libraries_) {
    if (pair.second != nullptr) {
      dlclose(pair.second);
    }
  }
  loaded_libraries_.clear();
}

PerformanceBridgeResult PerformanceBridgeLoader::create_r2a_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "r2a", "create_r2a_bridge");
  if (symbol == nullptr) {
    return {nullptr, nullptr};
  }

  auto factory = reinterpret_cast<BridgeEntryR2A>(symbol);
  return factory(std::move(node), topic_name, qos);
}

PerformanceBridgeResult PerformanceBridgeLoader::create_a2r_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "a2r", "create_a2r_bridge");
  if (symbol == nullptr) {
    return {nullptr, nullptr};
  }

  auto factory = reinterpret_cast<BridgeEntryA2R>(symbol);
  return factory(std::move(node), topic_name, qos);
}

std::string PerformanceBridgeLoader::convert_type_to_snake_case(const std::string & message_type)
{
  std::string result = message_type;
  std::replace(result.begin(), result.end(), '/', '_');
  return result;
}

std::vector<std::string> PerformanceBridgeLoader::generate_library_paths(
  const std::string & snake_type, const std::string & plugin_suffix)
{
  std::vector<std::string> paths;
  const std::string lib_name = "lib" + plugin_suffix + "_bridge_plugin_" + snake_type + ".so";

  // 1. Check environment variable AGNOCAST_BRIDGE_PLUGINS_PATH (colon-separated)
  const char * env_path = std::getenv("AGNOCAST_BRIDGE_PLUGINS_PATH");
  if (env_path != nullptr) {
    std::string env_str(env_path);
    std::istringstream iss(env_str);
    std::string path;
    while (std::getline(iss, path, ':')) {
      if (!path.empty()) {
        std::string full_path;
        full_path.reserve(path.size() + 1 + lib_name.size());
        full_path += path;
        full_path += '/';
        full_path += lib_name;
        paths.push_back(std::move(full_path));
      }
    }
  }

  // 2. Check user-generated agnocast_bridge_plugins package
  try {
    const std::string user_prefix = ament_index_cpp::get_package_prefix("agnocast_bridge_plugins");
    paths.push_back(user_prefix + "/lib/agnocast_bridge_plugins/" + lib_name);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    // Package not found, continue
  }

  return paths;
}

void * PerformanceBridgeLoader::load_library_from_paths(const std::vector<std::string> & paths)
{
  if (paths.empty()) {
    RCLCPP_ERROR(logger_, "No plugin paths available. Have you generated bridge plugins?");
    return nullptr;
  }

  for (const auto & path : paths) {
    // Check cache first
    if (loaded_libraries_.find(path) != loaded_libraries_.end()) {
      return loaded_libraries_[path];
    }

    // Try to load
    void * handle = dlopen(path.c_str(), RTLD_LAZY);
    if (handle != nullptr) {
      loaded_libraries_[path] = handle;
      return handle;
    }
  }

  // All paths failed - log the error
  std::string tried_paths;
  for (const auto & path : paths) {
    tried_paths += "\n  - " + path;
  }
  RCLCPP_ERROR(
    logger_, "Failed to load plugin. Tried paths:%s\nLast error: %s", tried_paths.c_str(),
    dlerror());
  return nullptr;
}

void * PerformanceBridgeLoader::get_bridge_factory_symbol(
  const std::string & message_type, const std::string & direction, const std::string & symbol_name)
{
  std::string snake_type = convert_type_to_snake_case(message_type);
  std::vector<std::string> lib_paths = generate_library_paths(snake_type, direction);

  void * handle = load_library_from_paths(lib_paths);
  if (handle == nullptr) {
    return nullptr;
  }

  dlerror();
  void * symbol = dlsym(handle, symbol_name.c_str());

  const char * dlsym_error = dlerror();
  if (dlsym_error != nullptr) {
    RCLCPP_ERROR(
      logger_, "Failed to find symbol '%s' for message type '%s': %s", symbol_name.c_str(),
      message_type.c_str(), dlsym_error);
    return nullptr;
  }

  if (symbol == nullptr) {
    RCLCPP_ERROR(
      logger_,
      "Symbol '%s' was found for message type '%s' but returned NULL, which is invalid for a "
      "factory function.",
      symbol_name.c_str(), message_type.c_str());
    return nullptr;
  }

  return symbol;
}

}  // namespace agnocast
