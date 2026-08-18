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

using namespace std::chrono_literals;

class ParameterServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);

    rclcpp::NodeOptions client_options;
    client_options.start_parameter_services(false);
    client_node_ = std::make_shared<agnocast::Node>("param_client", client_options);

    params_client_ =
      std::make_shared<agnocast::AsyncParametersClient>(client_node_.get(), "param_server");

    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (agnocast::ok()) {
      agnocast::shutdown();
    }
  }

  std::shared_ptr<agnocast::Node> create_server_node(const rclcpp::NodeOptions & options)
  {
    auto node = std::make_shared<agnocast::Node>("param_server", options);

    node->declare_parameter<bool>("enable_partial_load", true);
    node->declare_parameter<double>("radius", 50.0);
    node->declare_parameter<std::string>("nested.name", "map_loader");

    executor_->add_node(node);

    return node;
  }

  // Blocks the test thread until the response arrives; the executor thread resolves it.
  template <typename FutureT>
  static bool resolved(FutureT & future)
  {
    return future.wait_for(500ms) == std::future_status::ready;
  }

  std::shared_ptr<agnocast::Node> client_node_;
  std::shared_ptr<agnocast::AsyncParametersClient> params_client_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(ParameterServiceTest, AllParameterEndpointsAreAvailable)
{
  // Arrange & Act
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Assert: the client confirms the parameter services are ready.
  EXPECT_TRUE(params_client_->wait_for_service(1s));
  EXPECT_TRUE(params_client_->service_is_ready());
}

TEST_F(ParameterServiceTest, GetParametersHappyPath)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->get_parameters({"radius", "enable_partial_load"});

  // Assert: the response contains all the parameters in request order.
  ASSERT_TRUE(resolved(future));

  const auto & parameters = future.get();
  ASSERT_EQ(2, parameters.size());
  EXPECT_EQ("radius", parameters[0].get_name());
  EXPECT_DOUBLE_EQ(50.0, parameters[0].as_double());
  EXPECT_EQ("enable_partial_load", parameters[1].get_name());
  EXPECT_TRUE(parameters[1].as_bool());
}

TEST_F(ParameterServiceTest, GetParametersWithUndeclaredParameter1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->get_parameters({"enable_partial_load", "not_declared"});

  // Assert: the response is empty because of an undeclared parameter.
  ASSERT_TRUE(resolved(future));
  EXPECT_TRUE(future.get().empty());
}

TEST_F(ParameterServiceTest, GetParametersWithUndeclaredParameter2)
{
  // Arrange: create a server node that allows undeclared parameters.
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto server_node = create_server_node(options);

  // Act
  auto future = params_client_->get_parameters({"enable_partial_load", "not_declared"});

  // Assert: the response contains both declared and undeclared parameters, with undeclared
  // parameters having a type of PARAMETER_NOT_SET.
  ASSERT_TRUE(resolved(future));

  const auto & parameters = future.get();
  ASSERT_EQ(2, parameters.size());
  EXPECT_EQ("enable_partial_load", parameters[0].get_name());
  EXPECT_TRUE(parameters[0].as_bool());
  EXPECT_EQ("not_declared", parameters[1].get_name());
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, parameters[1].get_type());
}

TEST_F(ParameterServiceTest, GetParameterTypesHappyPath)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->get_parameter_types({"radius", "enable_partial_load"});

  // Assert: the response contains all the parameter types in request order.
  ASSERT_TRUE(resolved(future));

  const auto & types = future.get();
  ASSERT_EQ(2, types.size());
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_DOUBLE, types[0]);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_BOOL, types[1]);
}

TEST_F(ParameterServiceTest, GetParameterTypesWithUndeclaredParameter1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->get_parameter_types({"enable_partial_load", "not_declared"});

  // Assert: the response is empty because of an undeclared parameter.
  ASSERT_TRUE(resolved(future));
  EXPECT_TRUE(future.get().empty());
}

TEST_F(ParameterServiceTest, GetParameterTypesWithUndeclaredParameter2)
{
  // Arrange: create a server node that allows undeclared parameters.
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto server_node = create_server_node(options);

  // Act
  auto future = params_client_->get_parameter_types({"enable_partial_load", "not_declared"});

  // Assert: the response contains the declared and undeclared parameters, with undeclared
  // parameters having a type of PARAMETER_NOT_SET.
  ASSERT_TRUE(resolved(future));

  const auto & types = future.get();
  ASSERT_EQ(2, types.size());
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_BOOL, types[0]);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, types[1]);
}

TEST_F(ParameterServiceTest, SetParametersWithUndeclaredParameter1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->set_parameters(
    {rclcpp::Parameter("radius", 120.0), rclcpp::Parameter("topic_name", "my_topic")});

  // Assert: declared parameters are updated, while undeclared parameters are rejected with
  // `successful` being false.
  ASSERT_TRUE(resolved(future));

  const auto & results = future.get();
  ASSERT_EQ(2, results.size());
  EXPECT_TRUE(results[0].successful);
  EXPECT_FALSE(results[1].successful);
  EXPECT_DOUBLE_EQ(120.0, server_node->get_parameter("radius").as_double());
}

TEST_F(ParameterServiceTest, SetParametersWithUndeclaredParameter2)
{
  // Arrange: create a server node that allows undeclared parameters
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto server_node = create_server_node(options);

  // Act
  auto future = params_client_->set_parameters(
    {rclcpp::Parameter("radius", 120.0), rclcpp::Parameter("topic_name", "my_topic")});

  // Assert: values of declared parameters are updated, while undeclared parameters are implicitly
  // declared with the given value.
  ASSERT_TRUE(resolved(future));

  const auto & results = future.get();
  ASSERT_EQ(2, results.size());
  EXPECT_TRUE(results[0].successful);
  EXPECT_TRUE(results[1].successful);
  EXPECT_DOUBLE_EQ(120.0, server_node->get_parameter("radius").as_double());
  EXPECT_STREQ("my_topic", server_node->get_parameter("topic_name").as_string().c_str());
}

TEST_F(ParameterServiceTest, SetParametersAtomicallyHappyPath1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->set_parameters_atomically(
    {rclcpp::Parameter("radius", 120.0), rclcpp::Parameter("enable_partial_load", false)});

  // Assert: the result is successful and all declared parameters are updated.
  ASSERT_TRUE(resolved(future));
  EXPECT_TRUE(future.get().successful);
  EXPECT_DOUBLE_EQ(120.0, server_node->get_parameter("radius").as_double());
  EXPECT_FALSE(server_node->get_parameter("enable_partial_load").as_bool());
}

TEST_F(ParameterServiceTest, SetParametersAtomicallyDoesNothingIfAnyFails1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->set_parameters_atomically(
    {rclcpp::Parameter("radius", 120.0), rclcpp::Parameter("topic_name", "my_topic")});

  // Assert: the result is unsuccessful and no parameters are updated because of an undeclared
  // parameter.
  ASSERT_TRUE(resolved(future));
  EXPECT_FALSE(future.get().successful);
  EXPECT_DOUBLE_EQ(50.0, server_node->get_parameter("radius").as_double());
  EXPECT_TRUE(server_node->get_parameter("enable_partial_load").as_bool());
}

TEST_F(ParameterServiceTest, SetParametersAtomicallyDoesNothingIfAnyFails2)
{
  // Arrange: create a server node that allows undeclared parameters
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto server_node = create_server_node(options);

  // Act
  auto future = params_client_->set_parameters_atomically(
    {rclcpp::Parameter("radius", true), rclcpp::Parameter("topic_name", "my_topic")});

  // Assert: the result is unsuccessful and no parameters are updated or declared because of type
  // inconsistency.
  ASSERT_TRUE(resolved(future));
  EXPECT_FALSE(future.get().successful);
  EXPECT_DOUBLE_EQ(50.0, server_node->get_parameter("radius").as_double());
  EXPECT_EQ(
    rclcpp::ParameterType::PARAMETER_NOT_SET, server_node->get_parameter("topic_name").get_type());
}

TEST_F(ParameterServiceTest, DescribeParametersHappyPath)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->describe_parameters({"radius", "enable_partial_load"});

  // Assert: the response contains all the parameters in requested order.
  ASSERT_TRUE(resolved(future));

  const auto & descriptors = future.get();
  ASSERT_EQ(2, descriptors.size());
  EXPECT_EQ("radius", descriptors[0].name);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_DOUBLE, descriptors[0].type);
  EXPECT_EQ("enable_partial_load", descriptors[1].name);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_BOOL, descriptors[1].type);
}

TEST_F(ParameterServiceTest, DescribeParametersWithUndeclaredParameter1)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->describe_parameters({"enable_partial_load", "not_declared"});

  // Assert: the response is empty because of an undeclared parameter.
  ASSERT_TRUE(resolved(future));
  EXPECT_TRUE(future.get().empty());
}

TEST_F(ParameterServiceTest, DescribeParametersWithUndeclaredParameter2)
{
  // Arrange: create a server node that allows undeclared parameters
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto server_node = create_server_node(options);

  // Act
  auto future = params_client_->describe_parameters({"enable_partial_load", "not_declared"});

  // Assert: the response contains both declared and undeclared parameters, with undeclared
  // parameters having a type of PARAMETER_NOT_SET.
  ASSERT_TRUE(resolved(future));

  const auto & descriptors = future.get();
  ASSERT_EQ(2, descriptors.size());
  EXPECT_EQ("enable_partial_load", descriptors[0].name);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_BOOL, descriptors[0].type);
  EXPECT_EQ("not_declared", descriptors[1].name);
  EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, descriptors[1].type);
}

TEST_F(ParameterServiceTest, ListParametersHappyPath)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->list_parameters({}, 0);

  // Assert: the response contains all declared parameter names, including `use_sim_time`.
  ASSERT_TRUE(resolved(future));

  const auto & names = future.get().names;
  ASSERT_EQ(4, names.size());
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "enable_partial_load"));
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "radius"));
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "nested.name"));
  // use_sim_time is declared automatically.
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "use_sim_time"));
}

TEST_F(ParameterServiceTest, ListParametersWithPrefixSpecified)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});

  // Act
  auto future = params_client_->list_parameters({"nested", "unknown"}, 0);

  // Assert: the response contains all parameter names under the given prefixes; a prefix that
  // matches nothing contributes no names (no error).
  ASSERT_TRUE(resolved(future));

  const auto & names = future.get().names;
  ASSERT_EQ(1, names.size());
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(), "nested.name"));
}

/* --- ParameterServiceTest: end --- */

using AsyncParametersClientTest = ParameterServiceTest;

TEST_F(AsyncParametersClientTest, EmptyRemoteNodeNameTargetsSelf)
{
  // Arrange
  auto server_node = create_server_node(rclcpp::NodeOptions{});
  auto server_params_client = std::make_shared<agnocast::AsyncParametersClient>(server_node.get());
  ASSERT_TRUE(server_params_client->wait_for_service(1s));

  {
    // Act
    auto future = server_params_client->get_parameters({"enable_partial_load"});

    // Assert: the future completes and the server node's own parameter is retrieved successfully.
    ASSERT_TRUE(resolved(future));

    const auto & parameters = future.get();
    ASSERT_EQ(1, parameters.size());
    EXPECT_EQ("enable_partial_load", parameters[0].get_name());
    EXPECT_TRUE(parameters[0].as_bool());
  }
  // Ensure that `future` is destroyed before `server_params_client` is destroyed; otherwise,
  // AGNOCAST_RELEASE_SUB_REF_CMD will be invoked with an already removed subscriber.
}

TEST_F(AsyncParametersClientTest, WaitForServiceTimesOutWhenParameterServicesNotAvailable)
{
  // Arrange: create an async parameters client that targets a node that does not serve parameter
  // services.
  auto client_params_client = std::make_shared<agnocast::AsyncParametersClient>(client_node_.get());

  // Act & Assert: the timeout expires and wait_for_service() returns false.
  EXPECT_FALSE(client_params_client->wait_for_service(1s));
}

TEST_F(AsyncParametersClientTest, InvokesCompletionCallback)
{
  // Arrange: prepare a promise that will be set by the completion callback as well as a server
  // node.
  auto server_node = create_server_node(rclcpp::NodeOptions{});
  std::promise<bool> from_callback;
  auto from_callback_future = from_callback.get_future();

  // Act: invoke get_parameters() with a completion callback that sets the promise.
  auto future = params_client_->get_parameters(
    {"radius"}, [&from_callback](std::shared_future<std::vector<rclcpp::Parameter>>) {
      from_callback.set_value(true);
    });

  // Assert: both the response future and `from_callback_future` complete.
  ASSERT_TRUE(resolved(future));
  ASSERT_EQ(std::future_status::ready, from_callback_future.wait_for(500ms));
  EXPECT_EQ(true, from_callback_future.get());
}
