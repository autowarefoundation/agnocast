// Black-box tests for agnocast::AsyncParametersClient.
//
// A "server" agnocast::Node declares parameters and exposes them through its
// agnocast::ParameterService (enabled by NodeOptions::start_parameter_services).
// A second "client" node reads and writes them through an
// agnocast::AsyncParametersClient. Both nodes are spun by one
// AgnocastOnlySingleThreadedExecutor, so every request/response round trip goes
// through the real Agnocast service path.

#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "agnocast/node/agnocast_parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;

// Generous enough for a loaded CI machine, short enough that a genuine failure
// does not stall the suite.
constexpr auto kServiceTimeout = 5s;
constexpr auto kResponseTimeout = 5s;
}  // namespace

class TestAsyncParametersClient : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);

    rclcpp::NodeOptions server_options;
    server_options.start_parameter_services(true);
    server_ =
      std::make_shared<agnocast::Node>("param_server", "/test_param_client", server_options);
    server_->declare_parameter<bool>("enable_partial_load", true);
    server_->declare_parameter<double>("radius", 50.0);
    server_->declare_parameter<std::string>("nested.name", "map_loader");

    // The client node needs no parameter services of its own; disabling them keeps the
    // parameter endpoints in this process unambiguous.
    rclcpp::NodeOptions client_options;
    client_options.start_parameter_services(false);
    client_ =
      std::make_shared<agnocast::Node>("param_client", "/test_param_client", client_options);

    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(server_);
    executor_->add_node(client_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    params_ = std::make_shared<agnocast::AsyncParametersClient>(
      client_.get(), server_->get_fully_qualified_name());
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    params_.reset();
    executor_.reset();
    client_.reset();
    server_.reset();
    agnocast::shutdown();
  }

  // Blocks the test thread until the response arrives; the executor thread resolves it.
  template <typename FutureT>
  static bool resolved(FutureT & future)
  {
    return future.wait_for(kResponseTimeout) == std::future_status::ready;
  }

  std::shared_ptr<agnocast::Node> server_;
  std::shared_ptr<agnocast::Node> client_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<agnocast::AsyncParametersClient> params_;
};

TEST_F(TestAsyncParametersClient, wait_for_service_finds_the_remote_parameter_services)
{
  EXPECT_TRUE(params_->wait_for_service(kServiceTimeout));
  EXPECT_TRUE(params_->service_is_ready());
}

TEST_F(TestAsyncParametersClient, get_parameters_returns_declared_values_in_request_order)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->get_parameters({"radius", "enable_partial_load"});
  ASSERT_TRUE(resolved(future));

  const auto parameters = future.get();
  ASSERT_EQ(2u, parameters.size());
  EXPECT_EQ("radius", parameters[0].get_name());
  EXPECT_DOUBLE_EQ(50.0, parameters[0].as_double());
  EXPECT_EQ("enable_partial_load", parameters[1].get_name());
  EXPECT_TRUE(parameters[1].as_bool());
}

TEST_F(TestAsyncParametersClient, get_parameters_returns_nothing_when_any_name_is_undeclared)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->get_parameters({"enable_partial_load", "not_declared"});
  ASSERT_TRUE(resolved(future));

  // A single undeclared name makes the server abandon the whole response, not just that name:
  // its handler calls get_parameters() for all names at once and swallows the resulting
  // ParameterNotDeclaredException. rclcpp::ParameterService behaves identically, so callers must
  // treat a short response as "ask again one name at a time", not "these ones exist".
  EXPECT_TRUE(future.get().empty());
}

TEST_F(TestAsyncParametersClient, get_parameters_invokes_the_completion_callback)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  std::promise<std::vector<rclcpp::Parameter>> from_callback;
  auto from_callback_future = from_callback.get_future();
  auto future = params_->get_parameters(
    {"radius"}, [&from_callback](std::shared_future<std::vector<rclcpp::Parameter>> completed) {
      from_callback.set_value(completed.get());
    });

  ASSERT_TRUE(resolved(future));
  ASSERT_EQ(std::future_status::ready, from_callback_future.wait_for(kResponseTimeout));
  const auto parameters = from_callback_future.get();
  ASSERT_EQ(1u, parameters.size());
  EXPECT_DOUBLE_EQ(50.0, parameters[0].as_double());
}

TEST_F(TestAsyncParametersClient, get_parameter_types_reports_the_declared_types)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->get_parameter_types({"enable_partial_load", "radius", "nested.name"});
  ASSERT_TRUE(resolved(future));

  const auto types = future.get();
  ASSERT_EQ(3u, types.size());
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_BOOL, types[0]);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_DOUBLE, types[1]);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_STRING, types[2]);
}

TEST_F(TestAsyncParametersClient, set_parameters_updates_the_remote_node)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->set_parameters({rclcpp::Parameter("radius", 120.0)});
  ASSERT_TRUE(resolved(future));

  const auto results = future.get();
  ASSERT_EQ(1u, results.size());
  EXPECT_TRUE(results[0].successful);
  EXPECT_DOUBLE_EQ(120.0, server_->get_parameter("radius").as_double());
}

TEST_F(TestAsyncParametersClient, set_parameters_atomically_updates_the_remote_node)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->set_parameters_atomically(
    {rclcpp::Parameter("radius", 30.0), rclcpp::Parameter("enable_partial_load", false)});
  ASSERT_TRUE(resolved(future));

  EXPECT_TRUE(future.get().successful);
  EXPECT_DOUBLE_EQ(30.0, server_->get_parameter("radius").as_double());
  EXPECT_FALSE(server_->get_parameter("enable_partial_load").as_bool());
}

TEST_F(TestAsyncParametersClient, describe_parameters_returns_one_descriptor_per_name)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->describe_parameters({"radius"});
  ASSERT_TRUE(resolved(future));

  const auto descriptors = future.get();
  ASSERT_EQ(1u, descriptors.size());
  EXPECT_EQ("radius", descriptors[0].name);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_DOUBLE, descriptors[0].type);
}

TEST_F(TestAsyncParametersClient, list_parameters_reports_the_declared_names)
{
  ASSERT_TRUE(params_->wait_for_service(kServiceTimeout));

  auto future = params_->list_parameters({}, 0);
  ASSERT_TRUE(resolved(future));

  const auto & names = future.get().names;
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "enable_partial_load"));
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "radius"));
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "nested.name"));
}

TEST_F(TestAsyncParametersClient, empty_remote_node_name_targets_the_owning_node)
{
  // The client node runs no parameter services, so a self-targeted client must never
  // become ready. This pins down which node an empty remote name resolves to.
  auto self_client = std::make_shared<agnocast::AsyncParametersClient>(client_.get());
  EXPECT_FALSE(self_client->wait_for_service(500ms));

  auto server_side = std::make_shared<agnocast::AsyncParametersClient>(server_.get());
  EXPECT_TRUE(server_side->wait_for_service(kServiceTimeout));
}

TEST_F(TestAsyncParametersClient, wait_for_service_times_out_for_an_unknown_node)
{
  auto absent = std::make_shared<agnocast::AsyncParametersClient>(
    client_.get(), "/test_param_client/no_such_node");
  EXPECT_FALSE(absent->wait_for_service(500ms));
  EXPECT_FALSE(absent->service_is_ready());
}
