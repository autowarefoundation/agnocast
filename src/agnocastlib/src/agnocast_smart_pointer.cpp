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
  if (ioctl(agnocast_fd, AGNOCAST_RELEASE_SUB_REF_CMD, &entry_args) < 0) {
    throw std::runtime_error(
      std::string("AGNOCAST_RELEASE_SUB_REF_CMD failed: ") + strerror(errno));
  }
}

}  // namespace agnocast
