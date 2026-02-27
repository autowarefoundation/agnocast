#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {

struct topic_info_ret * get_agnocast_sub_nodes(const char * topic_name, int * topic_info_ret_count)
{
  *topic_info_ret_count = 0;

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return nullptr;
  }

  struct topic_info_ret * agnocast_topic_info_ret_buffer = static_cast<struct topic_info_ret *>(
    malloc(MAX_TOPIC_INFO_RET_NUM * sizeof(struct topic_info_ret)));

  if (agnocast_topic_info_ret_buffer == nullptr) {
    fprintf(stderr, "Memory allocation failed\n");
    close(fd);
    return nullptr;
  }

  union ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {topic_name, strlen(topic_name)};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(agnocast_topic_info_ret_buffer);
  topic_info_args.topic_info_ret_buffer_size = MAX_TOPIC_INFO_RET_NUM;
  if (ioctl(fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    perror("AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD failed");
    free(agnocast_topic_info_ret_buffer);
    close(fd);
    return nullptr;
  }

  if (topic_info_args.ret_topic_info_ret_num == 0) {
    free(agnocast_topic_info_ret_buffer);
    close(fd);
    return nullptr;
  }

  *topic_info_ret_count = static_cast<int>(topic_info_args.ret_topic_info_ret_num);
  return agnocast_topic_info_ret_buffer;
}

struct topic_info_ret * get_agnocast_pub_nodes(const char * topic_name, int * topic_info_ret_count)
{
  *topic_info_ret_count = 0;

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return nullptr;
  }

  struct topic_info_ret * agnocast_topic_info_ret_buffer = static_cast<struct topic_info_ret *>(
    malloc(MAX_TOPIC_INFO_RET_NUM * sizeof(struct topic_info_ret)));

  if (agnocast_topic_info_ret_buffer == nullptr) {
    fprintf(stderr, "Memory allocation failed\n");
    close(fd);
    return nullptr;
  }

  union ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {topic_name, strlen(topic_name)};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(agnocast_topic_info_ret_buffer);
  topic_info_args.topic_info_ret_buffer_size = MAX_TOPIC_INFO_RET_NUM;
  if (ioctl(fd, AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD, &topic_info_args) < 0) {
    perror("AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD failed");
    free(agnocast_topic_info_ret_buffer);
    close(fd);
    return nullptr;
  }

  if (topic_info_args.ret_topic_info_ret_num == 0) {
    free(agnocast_topic_info_ret_buffer);
    close(fd);
    return nullptr;
  }

  *topic_info_ret_count = static_cast<int>(topic_info_args.ret_topic_info_ret_num);
  return agnocast_topic_info_ret_buffer;
}

void free_agnocast_topic_info_ret(struct topic_info_ret * array)
{
  free(array);
}
}
