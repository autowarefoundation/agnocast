#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestClient : public rclcpp::Node
{
  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  bool use_response_callback_;
  int target_count_;
  int iteration_ = 0;
  agnocast::Client<ServiceT>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_future<agnocast::ipc_shared_ptr<Response>> fut_;

  void future_based()
  {
    // Check that the response for the previous request has been received.
    if (iteration_ > 0) {
      if (fut_.wait_for(0s) == std::future_status::timeout) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response.");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Receiving %ld.", fut_.get()->sum);

      if (iteration_ >= target_count_) {
        RCLCPP_INFO(this->get_logger(), "All responses have been received. Shutting down.");
        rclcpp::shutdown();
        return;
      }
    }

    // Send the next request.
    if (iteration_ < target_count_) {
      auto request = client_->borrow_loaned_request();
      request->data.push_back(iteration_);
      fut_ = client_->async_send_request(std::move(request)).share();
    }

    iteration_++;
  }

  void callback_based()
  {
    // Check that the response for the previous request has been received.
    if (iteration_ > 0) {
      if (fut_.wait_for(0s) == std::future_status::timeout) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response.");
        rclcpp::shutdown();
        return;
      }

      if (iteration_ >= target_count_) {
        RCLCPP_INFO(this->get_logger(), "All responses have been received. Shutting down.");
        rclcpp::shutdown();
        return;
      }
    }

    // Send the next request.
    if (iteration_ < target_count_) {
      auto request = client_->borrow_loaned_request();
      request->data.push_back(iteration_);
      fut_ = client_
               ->async_send_request(
                 std::move(request),
                 [this](agnocast::Client<ServiceT>::SharedFuture sfut) {
                   RCLCPP_INFO(this->get_logger(), "Receiving %ld.", sfut.get()->sum);
                 })
               .future;
    }

    iteration_++;
  }

  void timer_callback()
  {
    if (use_response_callback_) {
      callback_based();
    } else {
      future_based();
    }
  }

public:
  explicit TestClient(const rclcpp::NodeOptions & options) : Node("test_client", options)
  {
    this->declare_parameter<std::string>("service_name", "/test_service");
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("use_response_callback", false);
    this->declare_parameter<int>("target_count", 1);

    std::string service_name = this->get_parameter("service_name").as_string();
    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    use_response_callback_ = this->get_parameter("use_response_callback").as_bool();
    target_count_ = this->get_parameter("target_count").as_int();

    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
    client_ = agnocast::create_client<ServiceT>(this, service_name, qos, cbg);

    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting for 5 seconds.");
      rclcpp::shutdown();
    }

    timer_ = this->create_wall_timer(200ms, std::bind(&TestClient::timer_callback, this));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestClient)
