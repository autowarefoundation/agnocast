#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

extern "C" {

// Fetch every topic and its pub/sub endpoints in the caller's IPC namespace in one shot.
// On success returns 0 and sets *out_topics / *out_endpoints to malloc'd arrays (free them via
// free_agnocast_discovery_snapshot) plus their counts. Returns -1 on failure (outputs left empty).
int get_agnocast_discovery_snapshot(
  struct snapshot_topic_ret ** out_topics, int * out_topic_count,
  struct snapshot_endpoint_ret ** out_endpoints, int * out_endpoint_count)
{
  *out_topics = nullptr;
  *out_topic_count = 0;
  *out_endpoints = nullptr;
  *out_endpoint_count = 0;

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return -1;
  }

  // Probe: count-only call (null buffers / zero capacity). The kmod reports the required
  // capacity in ret_topic_num / ret_endpoint_num and returns -ENOBUFS when non-empty.
  struct ioctl_discovery_snapshot_args args = {};
  if (ioctl(fd, AGNOCAST_GET_DISCOVERY_SNAPSHOT_CMD, &args) < 0 && errno != ENOBUFS) {
    perror("AGNOCAST_GET_DISCOVERY_SNAPSHOT_CMD (probe) failed");
    close(fd);
    return -1;
  }

  if (args.ret_topic_num == 0) {
    // Empty namespace: nothing to fetch.
    close(fd);
    return 0;
  }

  struct snapshot_topic_ret * topics = nullptr;
  struct snapshot_endpoint_ret * endpoints = nullptr;
  uint32_t topic_cap = args.ret_topic_num;
  uint32_t ep_cap = args.ret_endpoint_num;

  // The topology can grow between probe and fill; retry with the newly reported capacity.
  constexpr int MAX_ATTEMPTS = 5;
  for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
    struct snapshot_topic_ret * t = static_cast<struct snapshot_topic_ret *>(
      realloc(topics, static_cast<size_t>(topic_cap) * sizeof(struct snapshot_topic_ret)));
    if (t == nullptr) {
      fprintf(stderr, "Memory allocation failed\n");
      break;
    }
    topics = t;

    if (ep_cap > 0) {
      struct snapshot_endpoint_ret * e = static_cast<struct snapshot_endpoint_ret *>(
        realloc(endpoints, static_cast<size_t>(ep_cap) * sizeof(struct snapshot_endpoint_ret)));
      if (e == nullptr) {
        fprintf(stderr, "Memory allocation failed\n");
        break;
      }
      endpoints = e;
    }

    struct ioctl_discovery_snapshot_args fill = {};
    fill.topic_buffer_addr = reinterpret_cast<uint64_t>(topics);
    fill.topic_buffer_size = topic_cap;
    fill.endpoint_buffer_addr = reinterpret_cast<uint64_t>(endpoints);
    fill.endpoint_buffer_size = ep_cap;

    if (ioctl(fd, AGNOCAST_GET_DISCOVERY_SNAPSHOT_CMD, &fill) == 0) {
      *out_topics = topics;
      *out_topic_count = static_cast<int>(fill.ret_topic_num);
      *out_endpoints = endpoints;
      *out_endpoint_count = static_cast<int>(fill.ret_endpoint_num);
      close(fd);
      return 0;
    }

    if (errno != ENOBUFS) {
      perror("AGNOCAST_GET_DISCOVERY_SNAPSHOT_CMD failed");
      break;
    }

    // Grew between calls: adopt the required capacity and retry.
    topic_cap = fill.ret_topic_num;
    ep_cap = fill.ret_endpoint_num;
  }

  free(topics);
  free(endpoints);
  close(fd);
  return -1;
}

void free_agnocast_discovery_snapshot(
  struct snapshot_topic_ret * topics, struct snapshot_endpoint_ret * endpoints)
{
  free(topics);
  free(endpoints);
}
}  // extern "C"
