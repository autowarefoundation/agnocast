#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"

#include <gtest/gtest.h>

using namespace agnocast;

int decrement_rc_mock_called_count = 0;
int increment_rc_mock_called_count = 0;
int publish_core_mock_called_count = 0;

namespace agnocast
{
void decrement_rc(const std::string &, const topic_local_id_t, const int64_t)
{
  decrement_rc_mock_called_count++;
}
void increment_rc(const std::string &, const topic_local_id_t, const int64_t)
{
  increment_rc_mock_called_count++;
}
topic_local_id_t initialize_publisher(
  const std::string &, const std::string &, const rclcpp::QoS &, const bool)
{
  return 0;  // Dummy value
}
union ioctl_publish_msg_args publish_core(
  const void *, const std::string &, const topic_local_id_t, const uint64_t,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> &)
{
  publish_core_mock_called_count++;
  return ioctl_publish_msg_args{};  // Dummy value
}
}  // namespace agnocast

// =========================================
// Publisher tests
// =========================================

class AgnocastPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    dummy_tn = "/dummy";
    pid = getpid();
    node = std::make_shared<rclcpp::Node>("dummy_node");
    dummy_publisher =
      agnocast::create_publisher<std_msgs::msg::Int32>(node.get(), dummy_tn, dummy_qos);

    publish_core_mock_called_count = 0;
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node;
  agnocast::Publisher<std_msgs::msg::Int32>::SharedPtr dummy_publisher;
  std::string dummy_tn;
  pid_t pid;
  rclcpp::QoS dummy_qos{10};
};

TEST_F(AgnocastPublisherTest, test_publish_normal)
{
  // Arrange
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();

  // Act
  dummy_publisher->publish(std::move(message));

  // Assert
  EXPECT_EQ(publish_core_mock_called_count, 1);
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0);
}

TEST_F(AgnocastPublisherTest, test_publish_null_message)
{
  // Arrange
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message;

  // Act & Assert
  EXPECT_EXIT(
    dummy_publisher->publish(std::move(message)), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Invalid message to publish.");
}

TEST_F(AgnocastPublisherTest, test_publish_already_published_message)
{
  // Arrange
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  dummy_publisher->publish(std::move(message));

  // Act & Assert
  EXPECT_EXIT(
    dummy_publisher->publish(std::move(message)), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Invalid message to publish.");
}

TEST_F(AgnocastPublisherTest, test_publish_different_message)
{
  // Arrange
  std::string diff_dummy_tn = "/dummy2";
  agnocast::Publisher<std_msgs::msg::Int32>::SharedPtr diff_publisher =
    agnocast::create_publisher<std_msgs::msg::Int32>(node.get(), diff_dummy_tn, dummy_qos);
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> diff_message =
    diff_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();

  // Act & Assert
  EXPECT_EXIT(
    dummy_publisher->publish(std::move(diff_message)), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Invalid message to publish.");
}

TEST_F(AgnocastPublisherTest, test_publish_loan_num_and_pub_num_mismatch)
{
  // Arrange
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  message.reset();  // This simulates the early return.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> next_message =
    dummy_publisher->borrow_loaned_message();

  // Act
  dummy_publisher->publish(std::move(next_message));

  // Assert
  EXPECT_EQ(agnocast_get_borrowed_publisher_num(), 0);
}

// =========================================
// ipc_shared_ptr tests
// =========================================

class AgnocastSmartPointerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dummy_tn = "dummy";
    dummy_pubsub_id = 1;
    dummy_entry_id = 2;

    decrement_rc_mock_called_count = 0;
    increment_rc_mock_called_count = 0;
  }

  std::string dummy_tn;
  topic_local_id_t dummy_pubsub_id;
  int64_t dummy_entry_id;
};

TEST_F(AgnocastSmartPointerTest, reset_normal)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut{new int(0), dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  sut.reset();

  // Assert
  EXPECT_EQ(decrement_rc_mock_called_count, 1);
  EXPECT_EQ(nullptr, sut.get());
}

TEST_F(AgnocastSmartPointerTest, reset_nullptr)
{
  // Arrange
  std::shared_ptr<agnocast::ipc_shared_ptr<int>> sut;

  // Act
  sut.reset();

  // Assert
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
}

TEST_F(AgnocastSmartPointerTest, copy_constructor_normal)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut{new int(0), dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<int> sut2 = sut;

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 1);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(sut.get(), sut2.get());
  EXPECT_EQ(sut.get_topic_name(), sut2.get_topic_name());
  EXPECT_EQ(sut.get_entry_id(), sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, copy_constructor_empty)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut;

  // Act & Assert
  EXPECT_NO_THROW(agnocast::ipc_shared_ptr<int> sut2{sut});
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(increment_rc_mock_called_count, 0);
}

TEST_F(AgnocastSmartPointerTest, copy_assignment_normal)
{
  // Arrange
  int * ptr = new int(0);
  int * ptr2 = new int(1);
  std::string dummy_tn2 = "dummy2";
  topic_local_id_t dummy_pubsub_id2 = 2;
  int64_t dummy_entry_id2 = 3;

  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};
  agnocast::ipc_shared_ptr<int> sut2{ptr2, dummy_tn2, dummy_pubsub_id2, dummy_entry_id2};

  // Act
  sut2 = sut;

  // Assert
  EXPECT_EQ(decrement_rc_mock_called_count, 1);
  EXPECT_EQ(increment_rc_mock_called_count, 1);
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_pubsub_id, sut2.get_pubsub_id());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, copy_assignment_self)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  sut = sut;

  // Assert
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(ptr, sut.get());
  EXPECT_EQ(dummy_tn, sut.get_topic_name());
  EXPECT_EQ(dummy_pubsub_id, sut.get_pubsub_id());
  EXPECT_EQ(dummy_entry_id, sut.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, copy_assignment_empty)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut;

  // Act
  sut = agnocast::ipc_shared_ptr<int>();

  // Assert
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(increment_rc_mock_called_count, 0);
}

TEST_F(AgnocastSmartPointerTest, move_constructor_normal)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<int> sut2 = std::move(sut);

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(nullptr, sut.get());
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, move_assignment_normal)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};
  agnocast::ipc_shared_ptr<int> sut2;

  // Act
  sut2 = std::move(sut);

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(nullptr, sut.get());
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, move_assignment_self)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-move"
  sut = std::move(sut);
#pragma GCC diagnostic pop

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(ptr, sut.get());
  EXPECT_EQ(dummy_tn, sut.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, dereference_operator)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  int & result = *sut;

  // Assert
  EXPECT_EQ(ptr, &result);
}

TEST_F(AgnocastSmartPointerTest, arrow_operator)
{
  // Arrange
  agnocast::ipc_shared_ptr<std::vector<int>> sut{
    new std::vector<int>{0}, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  size_t result = sut->size();

  // Assert
  EXPECT_EQ(1, result);
}

TEST_F(AgnocastSmartPointerTest, bool_operator_true)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut{new int(0), dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  bool result = static_cast<bool>(sut);

  // Assert
  EXPECT_TRUE(result);
}

TEST_F(AgnocastSmartPointerTest, bool_operator_false)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut;

  // Act
  bool result = static_cast<bool>(sut);

  // Assert
  EXPECT_FALSE(result);
}

// =========================================
// CallbackInfo tests
// =========================================

class AgnocastCallbackInfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dummy_tn = "dummy";
    dummy_pubsub_id = 1;
  }

  std::string dummy_tn;
  topic_local_id_t dummy_pubsub_id;
};

TEST_F(AgnocastCallbackInfoTest, get_erased_callback_normal)
{
  // Arrange
  bool callback_called = false;
  int data = 0;
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id)};
  auto int_callback = [&](const agnocast::ipc_shared_ptr<int> & /*unused_arg*/) {
    callback_called = true;
  };

  // Act
  agnocast::TypeErasedCallback erased_callback = agnocast::get_erased_callback<int>(int_callback);
  erased_callback(std::move(int_arg));

  // Assert
  EXPECT_TRUE(callback_called);
}

TEST_F(AgnocastCallbackInfoTest, get_erased_callback_invalid_type)
{
  // Arrange
  int data = 0;
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id)};
  auto float_callback = [&](agnocast::ipc_shared_ptr<float> /*unused_arg*/) {};

  // Act & Assert
  agnocast::TypeErasedCallback erased_callback =
    agnocast::get_erased_callback<float>(float_callback);
  EXPECT_EXIT(
    erased_callback(std::move(int_arg)), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Agnocast internal implementation error: bad allocation when callback is called");
}

TEST_F(AgnocastCallbackInfoTest, get_erased_callback_const_ptr)
{
  // Arrange
  bool callback_called = false;
  int data = 0;
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id)};
  auto const_callback = [&](const agnocast::ipc_shared_ptr<const int> & /*unused_arg*/) {
    callback_called = true;
  };

  // Act
  agnocast::TypeErasedCallback erased_callback = agnocast::get_erased_callback<int>(const_callback);
  erased_callback(std::move(int_arg));

  // Assert
  EXPECT_TRUE(callback_called);
}

TEST_F(AgnocastSmartPointerTest, converting_copy_constructor)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<const int> sut2 = sut;

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 1);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, converting_move_constructor)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<const int> sut2 = std::move(sut);

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(nullptr, sut.get());
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, converting_copy_assignment)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};
  agnocast::ipc_shared_ptr<const int> sut2;

  // Act
  sut2 = sut;

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 1);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}

TEST_F(AgnocastSmartPointerTest, converting_move_assignment)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};
  agnocast::ipc_shared_ptr<const int> sut2;

  // Act
  sut2 = std::move(sut);

  // Assert
  EXPECT_EQ(increment_rc_mock_called_count, 0);
  EXPECT_EQ(decrement_rc_mock_called_count, 0);
  EXPECT_EQ(nullptr, sut.get());
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}
