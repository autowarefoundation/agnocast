#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ClientTest : public ::testing::Test
{
  using Request = std_srvs::srv::SetBool::Request;
  using Response = std_srvs::srv::SetBool::Response;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<agnocast::SingleThreadedAgnocastExecutor> executor_;
  std::thread spin_thread_;

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_ = std::make_shared<agnocast::SingleThreadedAgnocastExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  auto create_service()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_service<std_srvs::srv::SetBool>(
      node_.get(), "test_service",
      [](
        agnocast::ipc_shared_ptr<Request> && request,
        agnocast::ipc_shared_ptr<Response> && response) { response->success = request->data; },
      rclcpp::ServicesQoS(), cb_group);
  }

  auto create_client()
  {
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return agnocast::create_client<std_srvs::srv::SetBool>(
      node_.get(), "test_service", rclcpp::ServicesQoS(), cb_group);
  }
};

TEST_F(ClientTest, ServiceIsReadyReturnsCorrectValue)
{
  auto client = create_client();

  EXPECT_FALSE(client->service_is_ready())
    << "service_is_ready() should return false before the service is created";

  auto service = create_service();

  EXPECT_TRUE(client->service_is_ready())
    << "service_is_ready() should return true after the service is created";
}

TEST_F(ClientTest, WaitForServiceReturnsWhenServiceIsReady)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  EXPECT_NE(future.wait_for(0ms), std::future_status::ready)
    << "wait_for_service() (indefinite) should block while the service is not ready";

  auto service = create_service();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly after the service is created";
  EXPECT_TRUE(future.get()) << "wait_for_service() should return true after the service is created";
}

TEST_F(ClientTest, WaitForServiceTimesOutWhenNoService)
{
  auto client = create_client();
  auto future =
    std::async(std::launch::async, [&client]() { return client->wait_for_service(100ms); });

  EXPECT_FALSE(future.get()) << "wait_for_service() should return false after the timeout";
}

TEST_F(ClientTest, WaitForServiceReturnsOnShutdown)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  rclcpp::shutdown();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly on shutdown";
  EXPECT_FALSE(future.get()) << "wait_for_service() should return false on shutdown";
}

TEST_F(ClientTest, RequestIdIsUnique)
{
  auto client = create_client();
  auto request1 = client->borrow_loaned_request();
  auto request2 = client->borrow_loaned_request();
  auto future_and_request_id2 = client->async_send_request(std::move(request2));
  auto future_and_request_id1 = client->async_send_request(std::move(request1));
  EXPECT_NE(future_and_request_id1.request_id, future_and_request_id2.request_id)
    << "Request IDs should be unique";
}

TEST_F(ClientTest, AllowsMultipleClientsOnTheSameNode)
{
  // Arrange: create a service and two clients.
  auto service = create_service();

  auto client1 = create_client();
  auto request1 = client1->borrow_loaned_request();
  request1->data = true;

  auto client2 = create_client();
  auto request2 = client2->borrow_loaned_request();
  request2->data = false;

  // Act: send true for client1, false for client2.
  auto future1 = client1->async_send_request(std::move(request1));
  auto future2 = client2->async_send_request(std::move(request2));

  // Assert: each client receives the correct response.
  ASSERT_EQ(future1.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(future2.wait_for(1s), std::future_status::ready);

  const auto & response1 = future1.get();
  EXPECT_EQ(response1->success, true);

  const auto & response2 = future2.get();
  EXPECT_EQ(response2->success, false);
}

/* --- ClientTest: end --- */

class AgnocastNodeClientTest : public ::testing::Test
{
  using Request = std_srvs::srv::Empty::Request;
  using Response = std_srvs::srv::Empty::Response;

  std::shared_ptr<agnocast::Node> node_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_;
  std::thread spin_thread_;

protected:
  void SetUp() override
  {
    agnocast::init(0, nullptr);
    node_ = std::make_shared<agnocast::Node>("test_node");
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
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

  auto create_client()
  {
    return agnocast::create_client<std_srvs::srv::Empty>(node_.get(), "test_service");
  }
};

TEST_F(AgnocastNodeClientTest, WaitForServiceReturnsOnShutdown)
{
  auto client = create_client();
  auto future = std::async(std::launch::async, [&client]() { return client->wait_for_service(); });
  // Give wait_for_service() a moment to start blocking.
  std::this_thread::sleep_for(100ms);

  agnocast::shutdown();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready)
    << "wait_for_service() should return promptly on shutdown";
  EXPECT_FALSE(future.get()) << "wait_for_service() should return false on shutdown";
}
