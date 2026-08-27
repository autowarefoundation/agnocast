#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
using Request = agnocast_sample_interfaces::srv::SumIntArray::Request;
using Response = agnocast_sample_interfaces::srv::SumIntArray::Response;

struct NodeParams
{
  std::string service_name;
  int64_t qos_depth;
  bool use_response_callback;
  // Whether to wait for a response before sending the next request. If `use_response_callback` is
  // false, this field is coerced to true.
  bool wait_response;
  // The number of responses to receive before shutting down.
  int64_t target_count;
};

NodeParams get_node_params(rclcpp::Node * node)
{
  node->declare_parameter<std::string>("service_name", "/test_service");
  node->declare_parameter<int64_t>("qos_depth", 10);
  node->declare_parameter<bool>("use_response_callback", false);
  node->declare_parameter<bool>("wait_response", true);
  node->declare_parameter<int64_t>("target_count", 1);

  NodeParams params;
  params.service_name = node->get_parameter("service_name").as_string();
  params.qos_depth = node->get_parameter("qos_depth").as_int();
  params.use_response_callback = node->get_parameter("use_response_callback").as_bool();
  params.wait_response =
    !params.use_response_callback || node->get_parameter("wait_response").as_bool();
  params.target_count = node->get_parameter("target_count").as_int();
  return params;
}

class TestClient : public rclcpp::Node
{
  bool use_response_callback_;
  bool wait_response_;
  int64_t target_count_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  agnocast::Client<ServiceT>::SharedPtr client_;

  std::atomic<int64_t> response_count_{0};
  std::thread request_thread_;
  std::vector<agnocast::Client<ServiceT>::SharedFuture> pending_futures_;

  // The asynchronous pattern has no deadline in step_once(), so it is enforced here.
  void wait_for_pending_responses()
  {
    for (auto & sfut : pending_futures_) {
      if (sfut.wait_for(2s) == std::future_status::timeout) {
        RCLCPP_ERROR(
          this->get_logger(), "Timeout waiting for responses. Received %ld of %ld.",
          response_count_.load(), target_count_);
        rclcpp::shutdown();
        return;
      }
    }
  }

  bool step_once(int64_t iteration)
  {
    auto request = client_->borrow_loaned_request();
    request->data.push_back(iteration);

    std::shared_future<agnocast::ipc_shared_ptr<Response>> sfut;
    if (use_response_callback_) {
      sfut =
        client_
          ->async_send_request(
            std::move(request),
            [this](agnocast::Client<ServiceT>::SharedFuture sfut) {
              RCLCPP_INFO(this->get_logger(), "Receiving %ld.", sfut.get()->sum);

              response_count_++;
              if (response_count_ >= target_count_) {
                RCLCPP_INFO(this->get_logger(), "All responses have been received. Shutting down.");
                rclcpp::shutdown();
              }
            })
          .future;
    } else {
      sfut = client_->async_send_request(std::move(request)).share();
    }

    if (!wait_response_) {
      pending_futures_.push_back(sfut);
      return true;
    }

    if (sfut.wait_for(2s) == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response.");
      rclcpp::shutdown();
      return false;
    }

    if (!use_response_callback_) {
      RCLCPP_INFO(this->get_logger(), "Receiving %ld.", sfut.get()->sum);

      response_count_++;
      if (response_count_ >= target_count_) {
        RCLCPP_INFO(this->get_logger(), "All responses have been received. Shutting down.");
        rclcpp::shutdown();
        return false;
      }
    }

    return true;
  }

public:
  explicit TestClient(const rclcpp::NodeOptions & options) : Node("test_client", options)
  {
    auto params = get_node_params(this);
    use_response_callback_ = params.use_response_callback;
    wait_response_ = params.wait_response;
    target_count_ = params.target_count;

    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(params.qos_depth)};
    client_ = agnocast::create_client<ServiceT>(this, params.service_name, qos, cbg_);

    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting for 5 seconds.");
      rclcpp::shutdown();
      return;
    }

    request_thread_ = std::thread([this]() {
      for (int64_t i = 0; i < target_count_; i++) {
        if (!step_once(i)) {
          return;
        }
      }
      wait_for_pending_responses();
    });
  }

  ~TestClient() override
  {
    if (request_thread_.joinable()) {
      request_thread_.join();
    }
  }
};

class TestROS2Client : public rclcpp::Node
{
  bool wait_response_;
  int64_t target_count_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Client<ServiceT>::SharedPtr client_;

  std::atomic<int64_t> response_count_{0};
  std::thread request_thread_;
  std::vector<rclcpp::Client<ServiceT>::SharedFuture> pending_futures_;

  // The asynchronous pattern has no deadline in step_once(), so it is enforced here.
  void wait_for_pending_responses()
  {
    for (auto & sfut : pending_futures_) {
      if (sfut.wait_for(2s) == std::future_status::timeout) {
        RCLCPP_ERROR(
          this->get_logger(), "Timeout waiting for responses. Received %ld of %ld.",
          response_count_.load(), target_count_);
        rclcpp::shutdown();
        return;
      }
    }
  }

  bool step_once(int64_t iteration)
  {
    auto request = std::make_shared<Request>();
    request->data.push_back(iteration);

    auto sfut =
      client_
        ->async_send_request(
          request,
          [this](rclcpp::Client<ServiceT>::SharedFuture sfut) {
            RCLCPP_INFO(this->get_logger(), "Receiving %ld.", sfut.get()->sum);

            response_count_++;
            if (response_count_ >= target_count_) {
              RCLCPP_INFO(this->get_logger(), "All responses have been received. Shutting down.");
              rclcpp::shutdown();
            }
          })
        .future;

    if (!wait_response_) {
      pending_futures_.push_back(sfut);
      return true;
    }

    if (sfut.wait_for(2s) == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response.");
      rclcpp::shutdown();
      return false;
    }

    return true;
  }

public:
  explicit TestROS2Client(const rclcpp::NodeOptions & options) : Node("test_ros2_client", options)
  {
    auto params = get_node_params(this);
    wait_response_ = params.wait_response;
    target_count_ = params.target_count;

    if (!params.use_response_callback) {
      RCLCPP_ERROR(this->get_logger(), "TestROS2Client requires use_response_callback to be true.");
      rclcpp::shutdown();
      return;
    }

    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::QoS qos{rclcpp::KeepLast(params.qos_depth)};
#if RCLCPP_VERSION_MAJOR >= 28
    client_ = this->create_client<ServiceT>(params.service_name, qos, cbg_);
#else
    client_ = this->create_client<ServiceT>(params.service_name, qos.get_rmw_qos_profile(), cbg_);
#endif

    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting for 5 seconds.");
      rclcpp::shutdown();
      return;
    }

    request_thread_ = std::thread([this]() {
      for (int64_t i = 0; i < target_count_; i++) {
        if (!step_once(i)) {
          return;
        }
      }
      wait_for_pending_responses();
    });
  }

  ~TestROS2Client() override
  {
    if (request_thread_.joinable()) {
      request_thread_.join();
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestClient)
RCLCPP_COMPONENTS_REGISTER_NODE(TestROS2Client)
