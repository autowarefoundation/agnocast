#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

struct NodeParams
{
  std::string service_name;
  int64_t qos_depth;
  bool use_deferred_callback;
  // The number of requests to handle before exiting
  int64_t target_count;
};

NodeParams get_node_params(rclcpp::Node * node)
{
  node->declare_parameter<std::string>("service_name", "/test_service");
  node->declare_parameter<int64_t>("qos_depth", 10);
  node->declare_parameter<bool>("use_deferred_callback", false);
  node->declare_parameter<int64_t>("target_count", 1);

  NodeParams params;
  params.service_name = node->get_parameter("service_name").as_string();
  params.qos_depth = node->get_parameter("qos_depth").as_int();
  params.use_deferred_callback = node->get_parameter("use_deferred_callback").as_bool();
  params.target_count = node->get_parameter("target_count").as_int();
  return params;
}

class TestServer : public rclcpp::Node
{
  int target_count_;
  int received_count_ = 0;
  rclcpp::CallbackGroup::SharedPtr cbg_;
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
    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", request->data[0]);

    std::thread([this, srv_handle = std::move(srv_handle), request = std::move(request)]() {
      // Wait for a while to simulate an asynchronous operation.
      std::this_thread::sleep_for(100ms);

      auto response = srv_handle->borrow_loaned_response(request);
      response->sum = request->data[0];
      auto request_movable = request;
      srv_handle->send_response(std::move(request_movable), std::move(response));

      received_count_ += 1;
      if (received_count_ >= target_count_) {
        RCLCPP_INFO(this->get_logger(), "All requests have been handled. Shutting down.");
        rclcpp::shutdown();
      }
    }).detach();
  }

public:
  explicit TestServer(const rclcpp::NodeOptions & options) : Node("test_server", options)
  {
    auto params = get_node_params(this);
    target_count_ = params.target_count;

    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(params.qos_depth)};

    if (params.use_deferred_callback) {
      srv_ = agnocast::create_service<ServiceT>(
        this, params.service_name, std::bind(&TestServer::deferred_callback, this, _1, _2), qos,
        cbg_);
    } else {
      srv_ = agnocast::create_service<ServiceT>(
        this, params.service_name, std::bind(&TestServer::basic_callback, this, _1, _2), qos, cbg_);
    }
  }
};

class TestROS2Server : public rclcpp::Node
{
  int target_count_;
  int received_count_ = 0;
  rclcpp::CallbackGroup::SharedPtr cbg_;
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
    auto params = get_node_params(this);
    assert(!params.use_deferred_callback && "deferred callback not implemented for TestROS2Server");
    target_count_ = params.target_count;

    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(params.qos_depth)};

    srv_ = this->create_service<ServiceT>(
      params.service_name, std::bind(&TestROS2Server::callback, this, _1, _2),
      qos.get_rmw_qos_profile(), cbg_);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestServer)
RCLCPP_COMPONENTS_REGISTER_NODE(TestROS2Server)
