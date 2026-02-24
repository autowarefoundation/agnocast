/**
 * @brief Sample application to demonstrate create_timer with use_sim_time support
 *
 * This sample creates a timer using agnocast::Node::create_timer() which supports
 * ROS_TIME (simulation time). When use_sim_time:=true, the timer will use the
 * /clock topic time instead of wall clock time.
 *
 * Usage:
 *   # Terminal 1: Run with use_sim_time enabled
 *   ros2 run agnocast_sample_application sim_time_timer --ros-args -p use_sim_time:=true
 *
 *   # Terminal 2: Publish clock messages
 *   ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 0, nanosec: 0}}" --once
 *   ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 1, nanosec: 0}}" --once
 *   ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 2, nanosec: 0}}" --once
 *
 *   # Or run without use_sim_time (uses wall clock)
 *   ros2 run agnocast_sample_application sim_time_timer
 */

#include "agnocast/agnocast.hpp"

#include <chrono>

using namespace std::chrono_literals;

class SimTimeTimerNode : public agnocast::Node
{
public:
  SimTimeTimerNode() : agnocast::Node("sim_time_timer_node")
  {
    // Log whether we're using sim time
    const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(
      this->get_logger(), "Starting timer node (use_sim_time: %s)",
      use_sim_time ? "true" : "false");

    // Create timer using create_timer() which supports ROS_TIME
    // This timer will respect use_sim_time parameter
    timer_ = this->create_timer(500ms, std::bind(&SimTimeTimerNode::timer_callback, this));

    const auto clock_type = timer_->get_clock()->get_clock_type();
    const char * clock_type_str = (clock_type == RCL_STEADY_TIME)   ? "STEADY_TIME"
                                  : (clock_type == RCL_ROS_TIME)    ? "ROS_TIME"
                                  : (clock_type == RCL_SYSTEM_TIME) ? "SYSTEM_TIME"
                                                                    : "UNKNOWN";
    RCLCPP_INFO(
      this->get_logger(), "Timer created with period 500ms, clock type: %s", clock_type_str);
  }

private:
  void timer_callback()
  {
    const auto now = this->now();
    const int64_t sec = now.seconds();
    const int64_t nsec = now.nanoseconds() % 1000000000;

    RCLCPP_INFO(
      this->get_logger(), "Timer callback! Current time: %ld.%09ld (count: %d)", sec, nsec,
      callback_count_++);
  }

  agnocast::TimerBase::SharedPtr timer_;
  int callback_count_ = 0;
};

int main(int argc, char * argv[])
{
  agnocast::init(argc, argv);

  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<SimTimeTimerNode>();
  executor.add_node(node);
  executor.spin();

  return 0;
}
