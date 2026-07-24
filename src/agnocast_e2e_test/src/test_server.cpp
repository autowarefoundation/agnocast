#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

class TestServer : public rclcpp::Node
{
  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
  using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
  using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

  int target_count_;
  int received_count_ = 0;
  agnocast::Service<ServiceT>::SharedPtr srv_;

  void basic_callback(
    const agnocast::ipc_shared_ptr<Request> & request,
    const agnocast::ipc_shared_ptr<Response> & response)
  {
    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", request->data[0]);

    response->sum = request->data[0];

    received_count_ += 1;
    if (received_count_ >= target_count_) {
      RCLCPP_INFO(this->get_logger(), "All requests have been handled. Shutting down.");
      rclcpp::shutdown();
    }
  }

  void deferred_callback(
    agnocast::Service<ServiceT>::SharedPtr srv_handle, agnocast::ipc_shared_ptr<Request> && request)
  {
    // TODO: Implement this.
    (void)srv_handle;
    (void)request;
  }

public:
  explicit TestServer(const rclcpp::NodeOptions & options) : Node("test_server", options)
  {
    this->declare_parameter<std::string>("service_name", "/test_service");
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("use_deferred_callback", false);
    this->declare_parameter<int>("target_count", 1);

    std::string service_name = this->get_parameter("service_name").as_string();
    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    bool use_deferred_callback = this->get_parameter("use_deferred_callback").as_bool();
    target_count_ = this->get_parameter("target_count").as_int();

    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};

    if (use_deferred_callback) {
      srv_ = agnocast::create_service<ServiceT>(
        this, service_name, std::bind(&TestServer::deferred_callback, this, _1, _2), qos, cbg);
    } else {
      srv_ = agnocast::create_service<ServiceT>(
        this, service_name, std::bind(&TestServer::basic_callback, this, _1, _2), qos, cbg);
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestServer)
