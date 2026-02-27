#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_timer_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"

#include <gtest/gtest.h>

using namespace agnocast;

int release_subscriber_reference_mock_called_count = 0;
int publish_core_mock_called_count = 0;
uint32_t mock_borrowed_publisher_num = 0;

extern "C" uint32_t agnocast_get_borrowed_publisher_num()
{
  return mock_borrowed_publisher_num;
}

namespace agnocast
{
void release_subscriber_reference(const std::string &, const topic_local_id_t, const int64_t)
{
  release_subscriber_reference_mock_called_count++;
}

void decrement_borrowed_publisher_num()
{
  if (mock_borrowed_publisher_num > 0) {
    mock_borrowed_publisher_num--;
  }
}

void increment_borrowed_publisher_num()
{
  mock_borrowed_publisher_num++;
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

BridgeMode get_bridge_mode()
{
  return BridgeMode::Off;  // Skip MQ sending in tests
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
    mock_borrowed_publisher_num = 0;
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

    release_subscriber_reference_mock_called_count = 0;
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
  // Last reference destroyed, so release_subscriber_reference is called once.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 1);
  EXPECT_EQ(nullptr, sut.get());
}

TEST_F(AgnocastSmartPointerTest, reset_nullptr)
{
  // Arrange
  std::shared_ptr<agnocast::ipc_shared_ptr<int>> sut;

  // Act
  sut.reset();

  // Assert
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
}

TEST_F(AgnocastSmartPointerTest, copy_constructor_normal)
{
  // Arrange
  agnocast::ipc_shared_ptr<int> sut{new int(0), dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<int> sut2 = sut;

  // Assert
  // Copy does not trigger any ioctl - reference counting is done in userspace.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
  // sut2's old reference is released (was the only reference), then copy happens (no ioctl).
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 1);
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
  // Self-assignment is a no-op.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
  // Both are empty, no ioctl calls.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
}

TEST_F(AgnocastSmartPointerTest, move_constructor_normal)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<int> sut2 = std::move(sut);

  // Assert
  // Move transfers ownership, no ioctl calls.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
  // sut2 was empty, so reset does nothing. Move transfers ownership, no ioctl calls.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wself-move"
#endif
  sut = std::move(sut);
#pragma GCC diagnostic pop

  // Assert
  // Self-assignment is a no-op.
  EXPECT_EQ(release_subscriber_reference_mock_called_count, 0);
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
// Publisher-side invalidation tests
// =========================================

class AgnocastInvalidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    dummy_tn = "/dummy";
    node = std::make_shared<rclcpp::Node>("dummy_node");
    dummy_publisher =
      agnocast::create_publisher<std_msgs::msg::Int32>(node.get(), dummy_tn, dummy_qos);

    publish_core_mock_called_count = 0;
    mock_borrowed_publisher_num = 0;
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node;
  agnocast::Publisher<std_msgs::msg::Int32>::SharedPtr dummy_publisher;
  std::string dummy_tn;
  rclcpp::QoS dummy_qos{10};
};

TEST_F(AgnocastInvalidationTest, dereference_after_publish_terminates)
{
  // Arrange: Borrow a message, make a copy, then publish.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy = message;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: Dereferencing the copy after publish should call std::terminate().
  EXPECT_DEATH(*copy, "invalidated ipc_shared_ptr");
}

TEST_F(AgnocastInvalidationTest, arrow_operator_after_publish_terminates)
{
  // Arrange: Borrow a message, make a copy, then publish.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy = message;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: Using arrow operator on the copy after publish should call std::terminate().
  EXPECT_DEATH(copy->data, "invalidated ipc_shared_ptr");
}

TEST_F(AgnocastInvalidationTest, get_returns_nullptr_after_publish)
{
  // Arrange: Borrow a message, make a copy, then publish.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy = message;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: get() should return nullptr after invalidation.
  EXPECT_EQ(copy.get(), nullptr);
}

TEST_F(AgnocastInvalidationTest, bool_operator_returns_false_after_publish)
{
  // Arrange: Borrow a message, make a copy, then publish.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy = message;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: bool operator should return false after invalidation.
  EXPECT_FALSE(static_cast<bool>(copy));
}

TEST_F(AgnocastInvalidationTest, multiple_copies_all_invalidated)
{
  // Arrange: Borrow a message, make multiple copies, then publish.
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy1 = message;
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy2 = message;
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy3 = copy1;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: All copies should be invalidated.
  EXPECT_EQ(copy1.get(), nullptr);
  EXPECT_EQ(copy2.get(), nullptr);
  EXPECT_EQ(copy3.get(), nullptr);
  EXPECT_FALSE(static_cast<bool>(copy1));
  EXPECT_FALSE(static_cast<bool>(copy2));
  EXPECT_FALSE(static_cast<bool>(copy3));
}

TEST_F(AgnocastInvalidationTest, copy_of_copy_invalidated)
{
  // Arrange: Create a chain of copies: original -> copy1 -> copy2
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> message = dummy_publisher->borrow_loaned_message();
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy1 = message;
  agnocast::ipc_shared_ptr<std_msgs::msg::Int32> copy2 = copy1;
  dummy_publisher->publish(std::move(message));

  // Act & Assert: Dereferencing copy2 (copy of copy) should also terminate.
  EXPECT_DEATH(*copy2, "invalidated ipc_shared_ptr");
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
  // Use subscriber-side constructor (4 args with entry_id) to avoid publisher-side delete behavior.
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id, /*entry_id=*/1)};
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
  // Use subscriber-side constructor (4 args with entry_id) to avoid publisher-side delete behavior.
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id, /*entry_id=*/1)};
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
  // Use subscriber-side constructor (4 args with entry_id) since publisher-side constructor is
  // private (users must use borrow_loaned_message()).
  agnocast::TypedMessagePtr<int> int_arg{
    agnocast::ipc_shared_ptr<int>(&data, dummy_tn, dummy_pubsub_id, /*entry_id=*/1)};
  auto const_callback = [&](const agnocast::ipc_shared_ptr<const int> & /*unused_arg*/) {
    callback_called = true;
  };

  // Act
  agnocast::TypeErasedCallback erased_callback = agnocast::get_erased_callback<int>(const_callback);
  erased_callback(std::move(int_arg));

  // Assert
  EXPECT_TRUE(callback_called);
}

// =========================================
// create_timer free function tests
// =========================================

class CreateTimerFreeFunctionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<agnocast::Node>("test_timer_node");
  }

  void TearDown() override
  {
    node.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<agnocast::Node> node;
};

TEST_F(CreateTimerFreeFunctionTest, creates_timer_and_registers_info)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {});

  // Assert
  ASSERT_NE(timer, nullptr);
  EXPECT_EQ(timer->get_clock()->get_clock_type(), RCL_STEADY_TIME);
}

TEST_F(CreateTimerFreeFunctionTest, uses_default_callback_group_when_nullptr)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(100));
  auto expected_group = node->get_node_base_interface()->get_default_callback_group();

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {}, nullptr);

  // Assert: verify the timer was registered with the default callback group
  ASSERT_NE(timer, nullptr);
  // Check via id2_timer_info that the callback group matches the default
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  bool found = false;
  for (const auto & [id, info] : agnocast::id2_timer_info) {
    if (info->callback_group == expected_group && info->timer.lock() == timer) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Timer should be registered with the default callback group";
}

TEST_F(CreateTimerFreeFunctionTest, uses_explicit_callback_group)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(50));
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Act
  auto timer = agnocast::create_timer(node.get(), clock, period, []() {}, group);

  // Assert
  ASSERT_NE(timer, nullptr);
  std::lock_guard<std::mutex> lock(agnocast::id2_timer_info_mtx);
  bool found = false;
  for (const auto & [id, info] : agnocast::id2_timer_info) {
    if (info->callback_group == group && info->timer.lock() == timer) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Timer should be registered with the explicit callback group";
}

TEST_F(CreateTimerFreeFunctionTest, callback_is_invoked)
{
  // Arrange
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = rclcpp::Duration(std::chrono::milliseconds(10));
  bool called = false;
  auto timer = agnocast::create_timer(node.get(), clock, period, [&called]() { called = true; });

  // Act
  timer->execute_callback();

  // Assert
  EXPECT_TRUE(called);
}

// =========================================
// ID overflow tests
// =========================================

TEST(AllocateCallbackInfoIdTest, throws_when_id_has_reserved_epoll_flag_bits)
{
  // Arrange: Set next_callback_info_id so the next allocation hits SHUTDOWN_EVENT_FLAG (bit 29).
  const uint32_t original = next_callback_info_id.load();
  next_callback_info_id.store(SHUTDOWN_EVENT_FLAG);

  // Act & Assert
  EXPECT_THROW(allocate_callback_info_id(), std::runtime_error);

  // Cleanup
  next_callback_info_id.store(original);
}

TEST(AllocateCallbackInfoIdTest, succeeds_just_below_reserved_range)
{
  // Arrange: Set next_callback_info_id to the maximum valid value (one below SHUTDOWN_EVENT_FLAG).
  const uint32_t original = next_callback_info_id.load();
  next_callback_info_id.store(SHUTDOWN_EVENT_FLAG - 1);

  // Act & Assert
  EXPECT_EQ(allocate_callback_info_id(), SHUTDOWN_EVENT_FLAG - 1);

  // Cleanup
  next_callback_info_id.store(original);
}

TEST(AllocateTimerIdTest, throws_when_id_has_reserved_epoll_flag_bits)
{
  // Arrange: Set next_timer_id so the returned ID includes a reserved epoll flag bit.
  // This covers the bug where timer IDs OR'd with CLOCK_EVENT_FLAG/TIMER_EVENT_FLAG
  // could collide with reserved flags.
  const uint32_t original = next_timer_id.load();
  next_timer_id.store(SHUTDOWN_EVENT_FLAG);

  // Act & Assert
  EXPECT_THROW(allocate_timer_id(), std::runtime_error);

  // Cleanup
  next_timer_id.store(original);
}

TEST(AllocateTimerIdTest, succeeds_just_below_reserved_range)
{
  // Arrange: Set next_timer_id to the maximum valid value.
  const uint32_t original = next_timer_id.load();
  next_timer_id.store(SHUTDOWN_EVENT_FLAG - 1);

  // Act & Assert
  EXPECT_EQ(allocate_timer_id(), SHUTDOWN_EVENT_FLAG - 1);

  // Cleanup
  next_timer_id.store(original);
}

TEST_F(AgnocastSmartPointerTest, converting_copy_constructor)
{
  // Arrange
  int * ptr = new int(0);
  agnocast::ipc_shared_ptr<int> sut{ptr, dummy_tn, dummy_pubsub_id, dummy_entry_id};

  // Act
  agnocast::ipc_shared_ptr<const int> sut2 = sut;

  // Assert
  EXPECT_EQ(ptr, sut.get());
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
  EXPECT_EQ(ptr, sut.get());
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
  EXPECT_EQ(nullptr, sut.get());
  EXPECT_EQ(ptr, sut2.get());
  EXPECT_EQ(dummy_tn, sut2.get_topic_name());
  EXPECT_EQ(dummy_entry_id, sut2.get_entry_id());
}
