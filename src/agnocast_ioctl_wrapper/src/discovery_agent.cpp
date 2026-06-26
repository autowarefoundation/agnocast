#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>

extern "C" {

// Returns 1 (should exit), 0 (keep running), or -1 on error (errno set).
int agnocast_discovery_agent_should_exit(uint32_t domain_id)
{
  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return -1;
  }

  struct ioctl_discovery_agent_should_exit_args args = {};
  args.domain_id = domain_id;
  if (ioctl(fd, AGNOCAST_DISCOVERY_AGENT_SHOULD_EXIT_CMD, &args) < 0) {
    perror("AGNOCAST_DISCOVERY_AGENT_SHOULD_EXIT_CMD failed");
    close(fd);
    return -1;
  }

  close(fd);
  return args.ret_should_exit ? 1 : 0;
}
}
