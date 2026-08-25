#include "agnocast/agnocast_smart_pointer.hpp"

#include "agnocast/agnocast_ipc.hpp"

namespace agnocast
{

void release_subscriber_reference(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id)
{
  struct ioctl_update_entry_args entry_args = {};
  entry_args.topic_name = {topic_name.c_str(), topic_name.size()};
  entry_args.pubsub_id = pubsub_id;
  entry_args.entry_id = entry_id;
  if (agnocast_ipc_release_sub_ref(&entry_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_RELEASE_SUB_REF_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
}

}  // namespace agnocast
