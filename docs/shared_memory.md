
## What is shared memory in Linux?

See official man page: <https://man7.org/linux/man-pages/man7/shm_overview.7.html>

## How shared memory is used in Agnocast?

### Basic usage

- A publisher opens a shared memory with a writable privilege.
- A subscriber opens the shared memory with a read-only privilege which the corresponding publisher opened.

### Detailed usage

There are three invocations for shared memory related procedures.

#### Initialization of a process

Each process linked with Agnocast opens a shared memory, which is writable only for the process.
When a process first calls malloc or other memory related functions, Agnocast starts and the shared memory is opened in the following steps:

1. get an allocatable area through `AGNOCAST_ADD_PROCESS_CMD` ioctl.
2. open a writable shared memory on the allocatable area, with `shm_open`, `ftruncate` and `mmap` system calls.
3. create a thread and open a message queue so that the process can recognize a emergence of a new publisher later.

#### Creation of a publisher

When a process calls `create_publisher` for a topic `T`, the shared memory of the process should be mapped by processes which have a subscription for `T`.
Thus the following procedures are executed:

1. The publisher process gets the information about subscribers already registered for the topic `T` through `AGNOCAST_ADD_PUBLISHER_CMD` ioctl.
2. The publisher process opens a message queue and send the shared memory information in order to notify the subscribers already created for the topic `T` that a new publisher is registered.
3. The subscriber process receives the message and maps the publisher's shared memory area with a read-only privilege.

#### Creation of a subscriber

When a process calls `create_subscription` for topic `T`, then the process maps the corresponding publisher's shared memory area with a read-only privilege in the following steps:

1. get the information about the publishers for the topic `T` through `AGNOCAST_ADD_SUBSCRIBER_CMD` ioctl.
2. map the publisher's shared memories with a read-only privilege.

### Naming rule and restrictions

In Agnocast, there is exactly one writable process and there are some read-only processes for a shared memory.
Suppose the writable process's id is `pid`, then the shared memory is named as "/agnocast@pid".
The name should start with '/' and should not include '/' any more.

Message queue is also created for each process.
Suppose the process's id is `pid`, then the message queue is named as "/new_publisher@pid".
The restriction for the name is the same as the shared memory.

## Memory allocation for shared memory

In the [original paper](https://www.arxiv.org/pdf/2506.16882) and its corresponding prototype implementation ([sykwer/agnocast](https://github.com/sykwer/agnocast)), all heap allocations are redirected to shared memory.
In contrast, in the [tier4/agnocast](https://github.com/tier4/agnocast) implementation, not all heap allocations are redirected to shared memory.
Ideally, only objects referenced by `agnocast::ipc_shared_ptr` should be placed in shared memory, while all other allocations should reside in the process-private heap.
However, since it is difficult to fully achieve this in practice, the implementation is designed to approximate this ideal as closely as possible.
Those interested may refer to the `agnocast_get_borrowed_publisher_num()` function in `agnocastlib` and `agnocast_heaphook`.
The current approach is that heap allocations occurring between the `AgnocastPublisher::borrow_loaned_message()` call and the subsequent `AgnocastPublisher::publish()` call are redirected to shared memory.
This is because it is not possible to determine exactly when, within this interval, a heap allocation for an object referenced by `agnocast::ipc_shared_ptr` will occur.

The virtual address space resources are managed in [agnocast_kmod/agnocast_memory_allocator.h](https://github.com/tier4/agnocast/blob/main/agnocast_kmod/agnocast_memory_allocator.h), and the ranges defined in this file are arbitrarily chosen.

## Mempool size configuration (Experimental)

> [!WARNING]
> The `mempool_size_gb` parameter is experimental and may be removed or changed in future versions.

The mempool size per process can be configured when loading the kernel module using the `mempool_size_gb` parameter.

### Usage

```bash
# Default: 8GB per process
sudo modprobe agnocast

# Custom size: 16GB per process
sudo modprobe agnocast mempool_size_gb=16

# Or with insmod
sudo insmod agnocast.ko mempool_size_gb=16
```

### Notes

- The parameter value is in gigabytes (GB)
- Default value is 8GB
- The configured size is logged to kernel messages (`dmesg`) when the module is loaded
- Due to demand paging, physical memory is allocated only upon the first access (first touch) of a page, so the configured size does not consume physical memory immediately

## Known issues

- When a heaphook fails to allocate a memory due to the lack of enough memory pool, heaphook tries to add a new memory. However, the added area will not be mapped by the subscribers in the current implementation and in turn Agnocast will not work well.
- The current implementation suppose that the memory after 0x40000000000 is always allocatable, though it is not investigated in detail.
