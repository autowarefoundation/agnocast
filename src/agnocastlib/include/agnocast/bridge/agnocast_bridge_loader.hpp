#pragma once

#include "agnocast/agnocast_mq.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace agnocast
{

using BridgeFn = std::shared_ptr<void> (*)(rclcpp::Node::SharedPtr, const BridgeTargetInfo &);

class BridgeLoader
{
public:
  explicit BridgeLoader(const rclcpp::Logger & logger);
  ~BridgeLoader();

  BridgeLoader(const BridgeLoader &) = delete;
  BridgeLoader & operator=(const BridgeLoader &) = delete;

  std::shared_ptr<void> create(
    const MqMsgBridge & req, const std::string & topic_name_with_direction,
    const rclcpp::Node::SharedPtr & node);

private:
  rclcpp::Logger logger_;

  std::map<std::string, std::pair<BridgeFn, std::shared_ptr<void>>> cached_factories_;

  std::shared_ptr<void> create_bridge_instance(
    BridgeFn entry_func, const std::shared_ptr<void> & lib_handle,
    const rclcpp::Node::SharedPtr & node, const BridgeTargetInfo & target);
  static std::pair<void *, uintptr_t> load_library(const char * lib_path, const char * symbol_name);
  std::pair<BridgeFn, std::shared_ptr<void>> resolve_factory_function(
    const MqMsgBridge & req, const std::string & topic_name_with_direction);
  static bool is_address_in_library_code_segment(void * handle, uintptr_t addr);
};

}  // namespace agnocast
