#include "node_for_executor_test.hpp"

NodeForExecutorTest::NodeForExecutorTest(
  const int64_t num_agnocast_sub_cbs, const int64_t num_ros2_sub_cbs,
  const int64_t num_agnocast_cbs_to_be_added, const std::chrono::milliseconds pub_period,
  const std::chrono::milliseconds cb_exec_time, const std::string cbg_type)
: Node("node_for_executor_test")
{
  if (cbg_type == "mutually_exclusive") {
    agnocast_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ros2_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  } else if (cbg_type == "reentrant") {
    agnocast_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ros2_common_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }
  cb_exec_time_ = cb_exec_time;

  // For Agnocast
  agnocast_timer_ =
    create_wall_timer(pub_period, std::bind(&NodeForExecutorTest::agnocast_timer_cb, this));
  for (int64_t i = 0; i < num_agnocast_sub_cbs; i++) {
    add_agnocast_sub_cb();
  }
  agnocast_sub_cbs_called_.assign(num_agnocast_sub_cbs + num_agnocast_cbs_to_be_added, false);

  // For ROS 2
  ros2_pub_ = create_publisher<std_msgs::msg::Bool>(ros2_topic_name_, 1);
  ros2_timer_ = create_wall_timer(pub_period, std::bind(&NodeForExecutorTest::ros2_timer_cb, this));
  for (int64_t i = 0; i < num_ros2_sub_cbs; i++) {
    rclcpp::SubscriptionOptions options;
    if (ros2_common_cbg_) {
      options.callback_group = ros2_common_cbg_;
    } else {  // individual
      options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    ros2_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      ros2_topic_name_, 1,
      [this, i](const std::shared_ptr<const std_msgs::msg::Bool> msg) { ros2_sub_cb(msg, i); },
      options));
  }
  ros2_sub_cbs_called_.assign(num_ros2_sub_cbs, false);
}

NodeForExecutorTest::~NodeForExecutorTest()
{
  for (auto & mq_sender : mq_senders_) {
    if (mq_close(mq_sender.second) == -1) {
      std::cerr << "mq_close failed for mq_name='" << mq_sender.first << "': " << strerror(errno)
                << std::endl;
    }
  }

  for (auto & mq_receiver : mq_receivers_) {
    if (mq_close(mq_receiver.first) == -1) {
      std::cerr << "mq_close failed for mq_name='" << mq_receiver.second << "': " << strerror(errno)
                << std::endl;
    }
    if (mq_unlink(mq_receiver.second.c_str()) == -1) {
      std::cerr << "mq_unlink failed for mq_name='" << mq_receiver.second
                << "': " << strerror(errno) << std::endl;
    }
  }
}

void NodeForExecutorTest::add_agnocast_sub_cb()
{
  int64_t cb_i = mq_receivers_.size();
  rclcpp::SubscriptionOptions options;
  auto cbg = (agnocast_common_cbg_)
               ? agnocast_common_cbg_
               : create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cbg;
  auto mq = open_mq_for_receiver(cb_i);
  std::function<void(const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> &)> callback =
    [this, cb_i](const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> & msg) {
      agnocast_sub_cb(msg, cb_i);
    };
  const bool is_transient_local = false;
  agnocast::register_callback<std_msgs::msg::Bool>(
    callback, agnocast_topic_name_, cb_i, is_transient_local, mq, cbg);
}

// NOTE: If the implementation of agnocast is changed, this function does not
// necessarily have to be changed as well, because this test is for the Executor.
mqd_t NodeForExecutorTest::open_mq_for_receiver(const int64_t cb_i)
{
  std::string mq_name = agnocast_topic_name_ + "@" + std::to_string(cb_i);
  struct mq_attr attr = {};
  attr.mq_flags = 0;                                  // Blocking queue
  attr.mq_msgsize = sizeof(agnocast::MqMsgAgnocast);  // Maximum message size
  attr.mq_curmsgs = 0;  // Number of messages currently in the queue (not set by mq_open)
  attr.mq_maxmsg = 1;

  const int mq_mode = 0666;
  mqd_t mq = mq_open(mq_name.c_str(), O_CREAT | O_RDONLY | O_NONBLOCK, mq_mode, &attr);
  if (mq == -1) {
    std::cerr << "mq_open failed for mq_name='" << mq_name << "': " << strerror(errno) << std::endl;
    exit(EXIT_FAILURE);
  }
  mq_receivers_.push_back(std::make_pair(mq, mq_name));

  return mq;
}

void NodeForExecutorTest::dummy_work(std::chrono::milliseconds exec_time)
{
  auto start = std::chrono::high_resolution_clock::now();
  while (std::chrono::high_resolution_clock::now() - start < exec_time) {
  }
}

// NOTE: If the implementation of agnocast is changed, this function does not
// necessarily have to be changed as well, because this test is for the Executor.
void NodeForExecutorTest::agnocast_timer_cb()
{
  // mq_send()
  for (auto & mq_receiver : mq_receivers_) {
    mqd_t mq = 0;
    if (mq_senders_.find(mq_receiver.second) != mq_senders_.end()) {
      mq = mq_senders_[mq_receiver.second];
    } else {
      mq = mq_open(mq_receiver.second.c_str(), O_WRONLY | O_NONBLOCK);
      if (mq == -1) {
        std::cerr << "mq_open failed for mq_name='" << mq_receiver.second
                  << "': " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
      }
      mq_senders_.insert({mq_receiver.second, mq});
    }

    agnocast::MqMsgAgnocast mq_msg = {};
    if (mq_send(mq, reinterpret_cast<char *>(&mq_msg), 0 /*msg_len*/, 0) == -1) {
      if (errno != EAGAIN) {
        std::cerr << "mq_send failed for mq_name='" << mq_receiver.second
                  << "': " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
      }
    }
  }

  // Add new agnocast sub callbacks
  if (mq_receivers_.size() < agnocast_sub_cbs_called_.size()) {
    add_agnocast_sub_cb();
  }
}

void NodeForExecutorTest::agnocast_sub_cb(
  [[maybe_unused]] const agnocast::ipc_shared_ptr<std_msgs::msg::Bool> & msg, int64_t cb_i)
{
  std::unique_lock<std::mutex> lock(mutex_for_agnocast_cbg_, std::try_to_lock);
  if (!lock.owns_lock()) {
    is_mutually_exclusive_agnocast_ = false;
  }

  // Each callback only accesses its own index, so it's safe to access the vector without a mutex.
  agnocast_sub_cbs_called_[cb_i] = true;
  dummy_work(cb_exec_time_);
}

void NodeForExecutorTest::ros2_timer_cb()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  ros2_pub_->publish(msg);
}

void NodeForExecutorTest::ros2_sub_cb(
  [[maybe_unused]] const std::shared_ptr<const std_msgs::msg::Bool> & msg, int64_t cb_i)
{
  std::unique_lock<std::mutex> lock(mutex_for_ros2_cbg_, std::try_to_lock);
  if (!lock.owns_lock()) {
    is_mutually_exclusive_ros2_ = false;
  }

  // Each callback only accesses its own index, so it's safe to access the vector without a mutex.
  ros2_sub_cbs_called_[cb_i] = true;
  dummy_work(cb_exec_time_);
}

bool NodeForExecutorTest::is_all_ros2_sub_cbs_called() const
{
  return std::all_of(ros2_sub_cbs_called_.begin(), ros2_sub_cbs_called_.end(), [](bool is_called) {
    return is_called;
  });
}

bool NodeForExecutorTest::is_all_agnocast_sub_cbs_called() const
{
  return std::all_of(
    agnocast_sub_cbs_called_.begin(), agnocast_sub_cbs_called_.end(),
    [](bool is_called) { return is_called; });
}

bool NodeForExecutorTest::is_mutually_exclusive_agnocast() const
{
  return is_mutually_exclusive_agnocast_;
}

bool NodeForExecutorTest::is_mutually_exclusive_ros2() const
{
  return is_mutually_exclusive_ros2_;
}
