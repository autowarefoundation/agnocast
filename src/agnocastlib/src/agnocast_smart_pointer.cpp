#include "agnocast/agnocast_smart_pointer.hpp"

namespace agnocast
{

void release_subscriber_reference(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id)
{
  struct ioctl_update_entry_args entry_args = {};
  entry_args.topic_name = {topic_name.c_str(), topic_name.size()};
  entry_args.pubsub_id = pubsub_id;
  entry_args.entry_id = entry_id;
  // Releasing a reference the kernel no longer tracks is not an error: the ioctl is idempotent, so
  // a message reference that outlives the Subscription which delivered it (~SubscriptionBase
  // already cleared this subscriber's bit on every entry of the topic) succeeds as a no-op. Any
  // failure that still reaches here therefore indicates real corruption, and is fatal as before.
  if (ioctl(agnocast_fd, AGNOCAST_RELEASE_SUB_REF_CMD, &entry_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_RELEASE_SUB_REF_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
}

}  // namespace agnocast
