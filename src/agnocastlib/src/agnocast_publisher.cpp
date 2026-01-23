#include "agnocast/agnocast_publisher.hpp"

#include <sys/types.h>

namespace agnocast
{

thread_local uint32_t borrowed_publisher_num = 0;

extern "C" uint32_t agnocast_get_borrowed_publisher_num()
{
  return borrowed_publisher_num;
}

void increment_borrowed_publisher_num()
{
  if (borrowed_publisher_num == 1) {
    return;

    // NOTE:
    //   This is a workaround for the case where borrow_loaned_message() is called but publish() is
    //   not. This implementation assumes only one loan/publish within a single callback and will
    //   need to be modified in the future. For this future modification, the type of
    //   borrowed_publisher_num is left as uint32_t.
  }

  borrowed_publisher_num++;
}

void decrement_borrowed_publisher_num()
{
  if (borrowed_publisher_num == 0) {
    RCLCPP_ERROR(
      logger,
      "The number of publish() called exceeds the number of borrow_loaned_message() called.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  borrowed_publisher_num--;
}

topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos,
  const bool is_bridge)
{
  validate_ld_preload();

  union ioctl_add_publisher_args pub_args = {};
  pub_args.topic_name = {topic_name.c_str(), topic_name.size()};
  pub_args.node_name = {node_name.c_str(), node_name.size()};
  pub_args.qos_depth = qos.depth();
  pub_args.qos_is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  pub_args.is_bridge = is_bridge;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_PUBLISHER_CMD, &pub_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_PUBLISHER_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return pub_args.ret_id;
}

union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle /* for CARET */, const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs)
{
  union ioctl_publish_msg_args publish_msg_args = {};
  publish_msg_args.topic_name = {topic_name.c_str(), topic_name.size()};
  publish_msg_args.publisher_id = publisher_id;
  publish_msg_args.msg_virtual_address = msg_virtual_address;
  if (ioctl(agnocast_fd, AGNOCAST_PUBLISH_MSG_CMD, &publish_msg_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_PUBLISH_MSG_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  TRACEPOINT(agnocast_publish, publisher_handle, publish_msg_args.ret_entry_id);

  for (uint32_t i = 0; i < publish_msg_args.ret_subscriber_num; i++) {
    const topic_local_id_t subscriber_id = publish_msg_args.ret_subscriber_ids[i];
    mqd_t mq = 0;
    if (opened_mqs.find(subscriber_id) != opened_mqs.end()) {
      std::tuple<mqd_t, bool> & t = opened_mqs[subscriber_id];
      mq = std::get<0>(t);
      // The boolean in the tuple indicates whether the mq is used in this publication round.
      // An unused mq means that its corresponding subscribers have exited, so we close such mqs
      // later.
      std::get<1>(t) = true;
    } else {
      const std::string mq_name = create_mq_name_for_agnocast_publish(topic_name, subscriber_id);
      mq = mq_open(mq_name.c_str(), O_WRONLY | O_NONBLOCK);
      if (mq == -1) {
        // Right after a subscriber is added, its message queue has not been created yet. Therefore,
        // the `mq_open` call above might fail. In that case, we log a warning and continue, but if
        // the warning keeps appearing, something must be wrong.
        RCLCPP_WARN(logger, "mq_open failed: %s", strerror(errno));
        continue;
      }
      opened_mqs.insert({subscriber_id, {mq, true}});
    }

    struct MqMsgAgnocast mq_msg = {};
    // Although the size of the struct is 1, we deliberately send a zero-length message
    if (mq_send(mq, reinterpret_cast<char *>(&mq_msg), 0 /*msg_len*/, 0) == -1) {
      // If it returns EAGAIN, it means mq_send has already been executed, but the subscriber
      // hasn't received it yet. Thus, there's no need to send it again since the notification has
      // already been sent.
      if (errno != EAGAIN) {
        RCLCPP_ERROR(logger, "mq_send failed: %s", strerror(errno));
      }
    }
  }

  // Close mqs that are no longer needed and update `opened_mqs`
  for (auto it = opened_mqs.begin(); it != opened_mqs.end();) {
    bool & keep = std::get<1>(it->second);
    if (!keep) {
      mqd_t mq = std::get<0>(it->second);
      if (mq_close(mq) == -1) {
        RCLCPP_ERROR(logger, "mq_close failed: %s", strerror(errno));
      }
      it = opened_mqs.erase(it);
    } else {
      // Update the value for the next publication round
      keep = false;
      ++it;
    }
  }

  return publish_msg_args;
}

uint32_t get_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  uint32_t inter_count = args.ret_other_process_subscriber_num;
  // Assumes at most one bridge subscriber per topic
  if (args.ret_a2r_bridge_exist && inter_count > 0) {
    inter_count--;
  }

  uint32_t ros2_count = args.ret_ros2_subscriber_num;
  // Assumes at most one bridge subscriber per topic
  if (args.ret_r2a_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  return inter_count + ros2_count;
}

uint32_t get_intra_subscription_count_core(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args get_subscriber_count_args = {};
  get_subscriber_count_args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &get_subscriber_count_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return get_subscriber_count_args.ret_same_process_subscriber_num;
}

}  // namespace agnocast
