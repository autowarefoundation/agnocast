#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {

bool is_service_topic(const std::string & topic_name)
{
  const std::string prefix = "/AGNOCAST_SRV_";
  if (topic_name.size() < prefix.size()) {
    return false;
  }
  return topic_name.compare(0, prefix.size(), prefix) == 0;
}

char ** get_agnocast_topics(int * topic_count)
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

  if (agnocast_topic_buffer == nullptr) {
    close(fd);
    return nullptr;
  }

  ioctl_topic_list_args topic_list_args = {};
  topic_list_args.topic_name_buffer_addr = reinterpret_cast<uint64_t>(agnocast_topic_buffer);
  if (ioctl(fd, AGNOCAST_GET_TOPIC_LIST_CMD, &topic_list_args) < 0) {
    perror("AGNOCAST_GET_TOPIC_LIST_CMD failed");
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  if (topic_list_args.ret_topic_num == 0) {
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  *topic_count = topic_list_args.ret_topic_num;

  char ** topic_array = static_cast<char **>(malloc(*topic_count * sizeof(char *)));
  if (topic_array == nullptr) {
    *topic_count = 0;
    return nullptr;
  }

  const size_t topic_count_size = static_cast<size_t>(*topic_count);
  for (size_t i = 0; i < topic_count_size; i++) {
    topic_array[i] = static_cast<char *>(malloc((TOPIC_NAME_BUFFER_SIZE + 1) * sizeof(char)));
    if (!topic_array[i]) {
      for (size_t j = 0; j < i; j++) {
        free(topic_array[j]);
      }
      free(topic_array);
      topic_array = nullptr;
      break;
    }
    std::string topic_name = agnocast_topic_buffer + i * TOPIC_NAME_BUFFER_SIZE;
    if (is_service_topic(topic_name)) {
      topic_array[i][0] = '\0';
      continue;
    }
    std::strcpy(topic_array[i], agnocast_topic_buffer + i * TOPIC_NAME_BUFFER_SIZE);
  }

  free(agnocast_topic_buffer);
  close(fd);
  return topic_array;
}

}  // extern "C"
