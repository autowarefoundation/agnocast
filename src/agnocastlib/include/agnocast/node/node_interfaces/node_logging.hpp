#pragma once

#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/version.h"

#include <memory>

namespace agnocast::node_interfaces
{

class NodeLogging : public rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeLogging>;
  using WeakPtr = std::weak_ptr<NodeLogging>;

  explicit NodeLogging(const rclcpp::Logger & logger);

  virtual ~NodeLogging() = default;

  NodeLogging(const NodeLogging &) = delete;
  NodeLogging & operator=(const NodeLogging &) = delete;

  rclcpp::Logger get_logger() const override;

  const char * get_logger_name() const override;

  // rclcpp 28+ (Jazzy) added this method to NodeLoggingInterface.
#if RCLCPP_VERSION_MAJOR >= 28
  void create_logger_services(
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services) override;
#endif

private:
  rclcpp::Logger logger_;
};

}  // namespace agnocast::node_interfaces
