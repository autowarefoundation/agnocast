#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

class TestROS2Server : public rclcpp::Node
{
  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  int target_count_;
  int received_count_ = 0;
  rclcpp::Service<ServiceT>::SharedPtr srv_;

  void callback(
    const std::shared_ptr<Request> & request, const std::shared_ptr<Response> & response)
  {
    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", request->data[0]);

    response->sum = request->data[0];

    received_count_ += 1;
    if (received_count_ >= target_count_) {
      RCLCPP_INFO(this->get_logger(), "All requests have been handled. Shutting down.");
      rclcpp::shutdown();
    }
  }

public:
  explicit TestROS2Server(const rclcpp::NodeOptions & options) : Node("test_ros2_server", options)
  {
    this->declare_parameter<std::string>("service_name", "/test_service");
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<int>("target_count", 1);

    std::string service_name = this->get_parameter("service_name").as_string();
    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    target_count_ = this->get_parameter("target_count").as_int();

    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};

    srv_ = this->create_service<ServiceT>(
      service_name, std::bind(&TestROS2Server::callback, this, _1, _2), qos.get_rmw_qos_profile(),
      cbg);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestROS2Server)
