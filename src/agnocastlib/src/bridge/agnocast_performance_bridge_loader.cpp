#include "agnocast/bridge/agnocast_performance_bridge_loader.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>

#include <algorithm>

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

rclcpp::SubscriptionBase::SharedPtr PerformanceBridgeLoader::create_r2a_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "r2a", "create_r2a_bridge");
  if (symbol == nullptr) {
    return nullptr;
  }

  auto factory = reinterpret_cast<BridgeEntryR2A>(symbol);

  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "Creating R2A bridge for topic: %s", topic_name.c_str());
  return factory(std::move(node), topic_name, qos);
}

std::shared_ptr<agnocast::SubscriptionBase> PerformanceBridgeLoader::create_a2r_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "a2r", "create_a2r_bridge");
  if (symbol == nullptr) {
    return nullptr;
  }

  auto factory = reinterpret_cast<BridgeEntryA2R>(symbol);

  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "Creating A2R bridge for topic: %s", topic_name.c_str());
  return factory(std::move(node), topic_name, qos);
}

std::string PerformanceBridgeLoader::convert_type_to_snake_case(const std::string & message_type)
{
  std::string result = message_type;
  std::replace(result.begin(), result.end(), '/', '_');
  return result;
}

std::string PerformanceBridgeLoader::generate_library_path(
  const std::string & snake_type, const std::string & plugin_suffix) const
{
  try {
    const std::string package_prefix = ament_index_cpp::get_package_prefix("agnocastlib");
    return package_prefix + "/lib/agnocastlib/bridge_plugins/lib" + plugin_suffix +
           "_bridge_plugin_" + snake_type + ".so";

  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    RCLCPP_ERROR(
      logger_, "Could not find package 'agnocastlib' to locate plugins. Error: %s", e.what());
    return "";
  }
}

void * PerformanceBridgeLoader::load_library(const std::string & library_path)
{
  if (library_path.empty()) {
    return nullptr;
  }

  if (loaded_libraries_.find(library_path) != loaded_libraries_.end()) {
    return loaded_libraries_[library_path];
  }

  void * handle = dlopen(library_path.c_str(), RTLD_LAZY);

  if (handle == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to load plugin '%s'. Error: %s", library_path.c_str(), dlerror());
    return nullptr;
  }

  // TODO(yutarokobayashi): For debugging. Remove later.
  RCLCPP_INFO(logger_, "Loaded plugin: %s", library_path.c_str());
  loaded_libraries_[library_path] = handle;
  return handle;
}

void * PerformanceBridgeLoader::get_bridge_factory_symbol(
  const std::string & message_type, const std::string & direction, const std::string & symbol_name)
{
  std::string snake_type = convert_type_to_snake_case(message_type);
  std::string lib_path = generate_library_path(snake_type, direction);

  void * handle = load_library(lib_path);
  if (handle == nullptr) {
    return nullptr;
  }

  dlerror();
  void * symbol = dlsym(handle, symbol_name.c_str());

  const char * dlsym_error = dlerror();
  if (dlsym_error != nullptr) {
    RCLCPP_ERROR(
      logger_, "Failed to find symbol '%s' in %s: %s", symbol_name.c_str(), lib_path.c_str(),
      dlsym_error);
    return nullptr;
  }

  if (symbol == nullptr) {
    RCLCPP_ERROR(
      logger_,
      "Symbol '%s' was found in %s but returned NULL, which is invalid for a factory function.",
      symbol_name.c_str(), lib_path.c_str());
    return nullptr;
  }

  return symbol;
}

}  // namespace agnocast
