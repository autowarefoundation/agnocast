# Agnocast Shared Memory Design Document

## Shared Memory Related Operations

### Initialization of a process

Agnocast processes invoke `AGNOCAST_ADD_PROCESS_CMD` ioctl during the initialization phase. The
ioctl returns a virtual memory range allocated for the process to map **writable** shared memory.

The process then creates an anonymous file via the `memfd_create` system call and maps it into the
virtual memory range. Specifically, the following seals are applied to the anonymous file before
mapping:

```cpp
int seals = F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_FUTURE_WRITE;
```

This prevents the file from being resized or modified by other processes. Note that other processes
or subscribers only need read permissions to the shared memory.

Next, the process registers the anonymous file with the kernel module using the
`AGNOCAST_REGISTER_PROCESS_SHM_CMD` ioctl. Inside the kernel module, the ioctl obtains a
`struct file *` reference to the anonymous file and stores it for future subscribers.

### Reception of published messages

When a subscriber receives a message, it uses the `AGNOCAST_RECEIVE_MSG_CMD` ioctl. In addition to
the message metadata, the ioctl also returns information about the shared memory that needs to be
mapped into the subscriber process's virtual memory.

```cpp
struct publisher_shm_info
{
  int32_t memfd;
  uint64_t shm_addr;
  uint64_t shm_size;
};
```

The information includes the file descriptor `memfd` referring to the shared memory, the virtual
memory address `shm_addr` to map the shared memory, and the shared memory size `shm_size`. The
subscriber processes the received messages only after mapping the shared memory.

## Memory allocation for shared memory

In the [original paper](https://www.arxiv.org/pdf/2506.16882) and its corresponding prototype implementation ([sykwer/agnocast](https://github.com/sykwer/agnocast)), all heap allocations are redirected to shared memory.
In contrast, in the [autowarefoundation/agnocast](https://github.com/autowarefoundation/agnocast) implementation, not all heap allocations are redirected to shared memory.
Ideally, only objects referenced by `agnocast::ipc_shared_ptr` should be placed in shared memory, while all other allocations should reside in the process-private heap.
However, since it is difficult to fully achieve this in practice, the implementation is designed to approximate this ideal as closely as possible.
Those interested may refer to the `agnocast_get_borrowed_publisher_num()` function in `agnocastlib` and `agnocast_heaphook`.
The current approach is that heap allocations occurring between the `AgnocastPublisher::borrow_loaned_message()` call and the subsequent `AgnocastPublisher::publish()` call are redirected to shared memory.
This is because it is not possible to determine exactly when, within this interval, a heap allocation for an object referenced by `agnocast::ipc_shared_ptr` will occur.

The virtual address space resources are managed in [agnocast_kmod/agnocast_memory_allocator.h](https://github.com/autowarefoundation/agnocast/blob/main/agnocast_kmod/agnocast_memory_allocator.h), and the ranges defined in this file are arbitrarily chosen.

## Mempool size configuration

The mempool size per process can be configured when loading the kernel module. See [Environment Setup](https://autowarefoundation.github.io/agnocast_doc/environment-setup/) on the documentation site for the module parameters (`mempool_num`, `mempool_size_gb`, `mempool_start_addr`) and their defaults.

## Known issues

- The current implementation suppose that the memory after 0x40000000000 is always allocatable, though it is not investigated in detail.
