#pragma once

#include "agnocast/agnocast_mq.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace agnocast
{

class BridgeBase;

using BridgeFn = std::shared_ptr<BridgeBase> (*)(
  rclcpp::Node::SharedPtr, const std::string &, const rclcpp::QoS &);

class StandardBridgeLoader
{
public:
  explicit StandardBridgeLoader(const rclcpp::Logger & logger);
  ~StandardBridgeLoader();

  StandardBridgeLoader(const StandardBridgeLoader &) = delete;
  StandardBridgeLoader & operator=(const StandardBridgeLoader &) = delete;

  std::shared_ptr<BridgeBase> create(
    const std::string & topic_name, BridgeDirection direction,
    const std::optional<std::string> & shared_lib_path, uintptr_t fn_offset_r2a,
    uintptr_t fn_offset_a2r, const rclcpp::Node::SharedPtr & node, const rclcpp::QoS & qos);

private:
  rclcpp::Logger logger_;

  std::map<std::string, std::pair<BridgeFn, std::shared_ptr<void>>> cached_factories_;

  std::shared_ptr<BridgeBase> create_bridge_instance(
    BridgeFn entry_func, const std::shared_ptr<void> & lib_handle,
    const rclcpp::Node::SharedPtr & node, const std::string & topic_name, const rclcpp::QoS & qos);
  static std::pair<void *, uintptr_t> load_library(
    const std::optional<std::string> & shared_lib_path);
  std::pair<BridgeFn, std::shared_ptr<void>> resolve_factory_function(
    const std::string & topic_name, BridgeDirection direction,
    const std::optional<std::string> & shared_lib_path, uintptr_t fn_offset_r2a,
    uintptr_t fn_offset_a2r);
  static bool is_address_in_library_code_segment(void * handle, uintptr_t addr);
};

}  // namespace agnocast
