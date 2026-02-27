#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {

char ** get_agnocast_sub_topics(const char * node_name, int * topic_count)
{
  *topic_count = 0;

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return nullptr;
  }

  char * agnocast_topic_buffer =
    static_cast<char *>(malloc(MAX_TOPIC_NUM * TOPIC_NAME_BUFFER_SIZE));

  union ioctl_node_info_args node_info_args = {};
  node_info_args.topic_name_buffer_addr = reinterpret_cast<uint64_t>(agnocast_topic_buffer);
  node_info_args.topic_name_buffer_size = MAX_TOPIC_NUM;
  node_info_args.node_name = {node_name, strlen(node_name)};
  if (ioctl(fd, AGNOCAST_GET_NODE_SUBSCRIBER_TOPICS_CMD, &node_info_args) < 0) {
    perror("AGNOCAST_TAKE_NODE_SUBSCRIBER_TOPICS_CMD failed");
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  if (node_info_args.ret_topic_num == 0) {
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  *topic_count = static_cast<int>(node_info_args.ret_topic_num);

  std::vector<std::string> agnocast_sub_topics(*topic_count);
  for (uint32_t i = 0; i < *topic_count; i++) {
    agnocast_sub_topics[i] =
      agnocast_topic_buffer + static_cast<size_t>(i) * TOPIC_NAME_BUFFER_SIZE;
  }

  free(agnocast_topic_buffer);
  close(fd);

  char ** topic_array = static_cast<char **>(malloc(*topic_count * sizeof(char *)));
  if (topic_array == nullptr) {
    return nullptr;
  }

  for (size_t i = 0; i < *topic_count; i++) {
    topic_array[i] =
      static_cast<char *>(malloc((agnocast_sub_topics[i].size() + 1) * sizeof(char)));
    if (!topic_array[i]) {
      for (size_t j = 0; j < i; j++) {
        free(topic_array[j]);
      }
      free(topic_array);
      return nullptr;
    }
    std::strcpy(topic_array[i], agnocast_sub_topics[i].c_str());
  }
  return topic_array;
}

char ** get_agnocast_pub_topics(const char * node_name, int * topic_count)
{
  *topic_count = 0;

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return nullptr;
  }

  char * agnocast_topic_buffer =
    static_cast<char *>(malloc(MAX_TOPIC_NUM * TOPIC_NAME_BUFFER_SIZE));

  union ioctl_node_info_args node_info_args = {};
  node_info_args.topic_name_buffer_addr = reinterpret_cast<uint64_t>(agnocast_topic_buffer);
  node_info_args.topic_name_buffer_size = MAX_TOPIC_NUM;
  node_info_args.node_name = {node_name, strlen(node_name)};
  if (ioctl(fd, AGNOCAST_GET_NODE_PUBLISHER_TOPICS_CMD, &node_info_args) < 0) {
    perror("AGNOCAST_TAKE_NODE_PUBLISHER_TOPICS_CMD failed");
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  if (node_info_args.ret_topic_num == 0) {
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  *topic_count = static_cast<int>(node_info_args.ret_topic_num);

  std::vector<std::string> agnocast_pub_topics(*topic_count);
  for (uint32_t i = 0; i < *topic_count; i++) {
    agnocast_pub_topics[i] =
      agnocast_topic_buffer + static_cast<size_t>(i) * TOPIC_NAME_BUFFER_SIZE;
  }

  free(agnocast_topic_buffer);
  close(fd);

  char ** topic_array = static_cast<char **>(malloc(*topic_count * sizeof(char *)));
  if (topic_array == nullptr) {
    return nullptr;
  }

  for (size_t i = 0; i < *topic_count; i++) {
    topic_array[i] =
      static_cast<char *>(malloc((agnocast_pub_topics[i].size() + 1) * sizeof(char)));
    if (!topic_array[i]) {
      for (size_t j = 0; j < i; j++) {
        free(topic_array[j]);
      }
      free(topic_array);
      return nullptr;
    }
    std::strcpy(topic_array[i], agnocast_pub_topics[i].c_str());
  }
  return topic_array;
}

void free_agnocast_topics(char ** topic_array, int topic_count)
{
  if (topic_array == nullptr) {
    return;
  }

  for (int i = 0; i < topic_count; i++) {
    free(topic_array[i]);
  }
  free(topic_array);
}

}  // extern "C"
