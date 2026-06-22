#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {

// When domain_ids_out is non-null, the function allocates a buffer into *domain_ids_out and
// passes its address to the GET_TOPIC_LIST ioctl, which fills it with the topics' domain ids.
// It is the same size as the topic buffer and maps one-to-one to it (the id at index i is
// topic i's domain).
// Allocation and freeing of the returned buffers are unified in this layer (the ioctl
// wrapper) for simplicity, so the caller never sizes them and frees the domain buffer with
// free_agnocast_topic_domains.
// Pass nullptr as domain_ids_out when you don't need the buffer to be allocated.
char ** get_agnocast_topics(int * topic_count, uint32_t ** domain_ids_out)
{
  *topic_count = 0;
  if (domain_ids_out != nullptr) {
    *domain_ids_out = nullptr;
  }

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

  uint32_t * domain_buffer = nullptr;
  if (domain_ids_out != nullptr) {
    domain_buffer = static_cast<uint32_t *>(malloc(MAX_TOPIC_NUM * sizeof(uint32_t)));
    if (domain_buffer == nullptr) {
      free(agnocast_topic_buffer);
      close(fd);
      return nullptr;
    }
  }

  ioctl_topic_list_args topic_list_args = {};
  topic_list_args.topic_name_buffer_addr = reinterpret_cast<uint64_t>(agnocast_topic_buffer);
  topic_list_args.domain_id_buffer_addr = reinterpret_cast<uint64_t>(domain_buffer);
  topic_list_args.topic_name_buffer_size = MAX_TOPIC_NUM;
  if (ioctl(fd, AGNOCAST_GET_TOPIC_LIST_CMD, &topic_list_args) < 0) {
    perror("AGNOCAST_GET_TOPIC_LIST_CMD failed");
    free(domain_buffer);
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  if (topic_list_args.ret_topic_num == 0) {
    free(domain_buffer);
    free(agnocast_topic_buffer);
    close(fd);
    return nullptr;
  }

  *topic_count = topic_list_args.ret_topic_num;

  char ** topic_array = static_cast<char **>(malloc(*topic_count * sizeof(char *)));
  if (topic_array == nullptr) {
    *topic_count = 0;
    free(domain_buffer);
    free(agnocast_topic_buffer);
    close(fd);
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
      *topic_count = 0;
      free(domain_buffer);
      domain_buffer = nullptr;
      break;
    }
    std::strcpy(topic_array[i], agnocast_topic_buffer + i * TOPIC_NAME_BUFFER_SIZE);
  }

  if (domain_ids_out != nullptr) {
    *domain_ids_out = domain_buffer;
  }

  free(agnocast_topic_buffer);
  close(fd);
  return topic_array;
}

void free_agnocast_topic_domains(uint32_t * domain_ids)
{
  free(domain_ids);
}
}  // extern "C"
