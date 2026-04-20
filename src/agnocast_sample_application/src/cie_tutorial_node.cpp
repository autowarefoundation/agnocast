// Tutorial: https://autowarefoundation.github.io/agnocast_doc/callback-isolated-executor/tutorial/

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// A simple node with three CallbackGroups to demonstrate
/// per-callback scheduling with CallbackIsolatedExecutor.
class CieTutorialNode : public rclcpp::Node
{
  // CallbackGroups — each gets a dedicated OS thread
  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr processing_group_;
  rclcpp::CallbackGroup::SharedPtr logging_group_;

  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr logging_timer_;

  void sensor_callback() { RCLCPP_INFO(get_logger(), "[sensor] reading data"); }

  void processing_callback() { RCLCPP_INFO(get_logger(), "[processing] computing"); }

  void logging_callback() { RCLCPP_INFO(get_logger(), "[logging] writing log"); }

public:
  CieTutorialNode() : Node("cie_tutorial_node")
  {
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    processing_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    logging_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    sensor_timer_ =
      create_wall_timer(100ms, std::bind(&CieTutorialNode::sensor_callback, this), sensor_group_);
    processing_timer_ = create_wall_timer(
      200ms, std::bind(&CieTutorialNode::processing_callback, this), processing_group_);
    logging_timer_ =
      create_wall_timer(1s, std::bind(&CieTutorialNode::logging_callback, this), logging_group_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  agnocast::CallbackIsolatedAgnocastExecutor executor;
  auto node = std::make_shared<CieTutorialNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
