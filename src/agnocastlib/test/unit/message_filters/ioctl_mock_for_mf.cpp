#include <sys/ioctl.h>

#include <cstdarg>
#include <cstdio>

// Simple ioctl mock used during message_filters unit tests.
// Returns success for all ioctl calls to bypass kernel-level IPC and driver
// interactions; no actual ioctl processing or validation is performed.
// This mock must only be used in tests, not in production code.
extern "C" int ioctl(int fd, unsigned long request, ...)
{
  (void)fd;
  (void)request;
  // Always report success to isolate tests from kernel behavior.
  return 0;
}
