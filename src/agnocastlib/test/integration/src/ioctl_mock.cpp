#include "agnocast/agnocast_ioctl.hpp"

#include <cstdarg>

// Override for `ioctl(agnocast_fd, AGNOCAST_RECEIVE_MSG_CMD, &receive_args)`
extern "C" int ioctl(int fd, unsigned long request, ...)
{
  (void)fd;

  va_list args;
  va_start(args, request);
  void * arg_ptr = va_arg(args, void *);
  va_end(args);

  agnocast::ioctl_receive_msg_args * receive_args =
    static_cast<agnocast::ioctl_receive_msg_args *>(arg_ptr);
  receive_args->ret_entry_num = 1;                   // Do not change this value
  receive_args->ret_call_again = false;              // Required for pagination loop to exit
  receive_args->ret_entry_ids[0] = 0;                // dummy
  receive_args->ret_entry_addrs[0] = 0;              // dummy
  receive_args->ret_pub_shm_info.publisher_num = 0;  // Do not change this value

  return 0;
}
