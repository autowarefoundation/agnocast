# The purpose
This development extends the Agnocast specification to allow zero-copy sharing of GPU device memory data among processes using the same GPU.
The existing Agnocast specification and implementation are documented in the files under the following directory:
https://github.com/autowarefoundation/agnocast/blob/main/README.md
https://github.com/autowarefoundation/agnocast/tree/main/docs/

## Inherited characteristics from Agnocast
Agnocast has 3 strong points:
1. The user can switch from ROS2's traditional rclcpp communication to Agnocast zero-copy communication with very small changes.
2. Agnocast supports any data types available in ROS2.
3. Agnocast and the traditional rclcpp communication can work together, they are not mutually exclusive.

To meet implementation restrictions, point 2 may be compromised.

## Abstraction of implementation for inter-process communication
Sharing of GPU device memory data is supported by 2 infrastructures depending on the type of NVIDIA GPU.
A. CUDA IPC on dedicated GPUs.
B. NvSci libraries on SoC configurations (Tegra family).

These infrastructures should be hidden in the "backend" layer. Users do not need to worry about their GPU architecture.
That abstraction should employ delegation, not inheritance.
For the first draft implementation, only the "CUDA IPC" version should be implemented.

## Initial status of the development branch
- Development work should be done on the local git branch "feature/gpu-ipc".
- The branch "feature/gpu-ipc" has been branched from the remote "origin/cuda-ipc" branch.
- The remote "origin/cuda-ipc" is just a proto-type for "proof-of-concept" that shows CUDA IPC can be used for shareing of GPU device memory.
- To meet implemantation restrictions, only agnocast::cuda::PointCloud2 and agnocast::cuda::Image are supported.
  - See agnocast/src/agnocast_cuda/include/agnocast/cuda/message_types.hpp 
- To clarify the image of the design, the following imperfect header files are implemented (commit cb02e715d47c4f83166abd5b9c6fa29502c54c4b):
  - agnocast_cuda/src/gpu_shared_memory_pool_proxy.hpp
  - agnocast_cuda/src/gpu_shared_memory_pool_proxy_cuda_ipc.hpp
  - agnocast_cuda/src/gpu_shared_memory_pool_proxy_nvsci.hpp


## Things to be considered in design
Sharing GPU device memory among processes has the following restrictions:
* Allocation of sharable GPU device memory takes considerable amount of time.
* Mapping of sharable GPU device memory allocated by other process (importing) takes considerable amount of time.
* A handle for a sharable GPU device memory block is exported by the allocator, then imported by other processes for sharing.
* The logical address of imported GPU device memory handle is unique to the importer. Other importers receive different logical addresses.
* GPU device memory is used by GPU streams asynchronously with their launcher CPU processes. Minimize synchronization between the CPU and GPU as possible.
  * A message with GPU device memory should be able to be published without waiting for finish of writing by  a GPU stream.
  * On call of publish(), cudaEventRecord() is inserted into the stream.
  * Subscribers call cudaStreamWaitForEvent() on reception of the message so that the callbacks can read valid data.
  * When the last subscriber finishes its callback, cudaEventRecord() is inserted in to the stream so that the publisher can wait for the event before freeing the slot.

# Grand design
The design will have the following modules.

## GpuSharedMemoryPoolDaemon
A daemon process that manages the "GPU shared memory pool". There should be a single instance per GPU.
* Started by systemd.
* Terminated by systemd.
* Restarted by systemd, when the process has been aborted.
* Allocates predefined number of "slots" at the initialization and manages them in pools.
  * A "slot" consists of:
    * An allocated GPU memory block with predefined size.
    * An inter-process event that can be waited for completion of writing into the corresponding memory block.
    * An inter-process event that can be waited for completion of reading by the subscribers.
* Exports the handles of slot-members of all slots so that they can be imported by other processes.
* Upon a "list" request from a client, sends a list of the slots and their exported handles.
* Upon an "alloc" request from a client, selects an free slot in the pools and returns the index of the slot.
  * The allocated slot is marked as "allocated" and removed from the free pool.
  * An "alloc" request has a "size" parameter in byte and a slot larger than the "size" parameter is selected.
* Upon an "free" request from a client, returns the slot specified by the index to the free pool.
* During termination of the daemon process, all of allocated GPU device memory blocks and inter-process events are freed or destroyed.

## GpuSharedMemoryPoolProxy
* A proxy class for the GpuSharedMemoryPoolDaemon.
* This is a Singleton class in a process.
* Communicates with the GpuSharedMemoryPoolDaemon on behalf of the publishers and the subscribers in the same process.
* Be instanciated on the first request (lazy instantiation).
* Be terminated at the end of the process (including abortion).
* During initialization, get a list of all slots from the GpuSharedMemoryPoolDaemon and import the handles for the process so that the following operations can be executed without delay.
* Upon an "alloc" request, forward the request to the GpuSharedMemoryPoolDaemon and get an index of the allocated slot and returns the index.
  * The returned index can be used for getting the logical address of the GPU device memory block in the allocated slot.
  * The returned index can be used for notifying the finish of writing/reading and waiting those finish.
* Upon an "free" request, forward the "free" request to the GpuSharedMemoryPoolDaemon and have the slot be freeed.

## libagnocast_heaphook
* Hooks cudaMalloc(), cudaFree() and their family fuctions using LD_PRELOAD.
* The first version should support cudaMalloc() and cudaFree().
* Other family functions will be implemented later. Do not implement them yet.
* Hooking cudaMalloc()
   * If cudaMalloc() is called by a publisher between borrow_loaned_message() and publish(), call GpuSharedMemoryPoolProxy::allocateMemory() and get the logical address of the allocated slot for the process.
   * If cudaMalloc() is called in other context, call the original cudaMalloc().
* Hooking cudaFree()
  * If cudaFree() is called in publish() and  the given GPU device memory pointer is found in the list of the "allocated by the same process" slots, call GpuSharedMemoryPoolProxy()::freeMemory().
  * Otherwise, call the original cudaFree().

# Implementation decisions
* Use unix domain sokets for communication between GpuSharedMemoryPoolDaemon and GpuSharedMemoryPoolProxy(s).
  * Unix domain socket communication is local in the host computer.
  * Implementation sample can be found in the following modules:
    * https://github.com/autowarefoundation/autoware_universe/blob/main/system/autoware_system_monitor/src/hdd_monitor/hdd_monitor.cpp
    * https://github.com/autowarefoundation/autoware_universe/blob/main/system/autoware_system_monitor/reader/hdd_reader/hdd_reader.cpp
* The source codes for GpuSharedMemoryPoolDaemon should be placed under "./daemon" directory and distinguished from the library code.

# The order about the development
* Devide large development into small steps. After each step, commit the code locally and ask the user for a review.
  * With this project, the order will be:
    * CUDA IPC version
      * GpuSharedMemoryPoolDaemon
      * GpuSharedMemoryPoolProxy
      * libagnocast_heaphook
        * In the first version, only cudaMalloc() and cudaFree() are hooked.
        * Hook all of cudaMalloc(), cudaFree() family, including their counterparts of CUDA driver API.
    * NvSci version
      * GpuSharedMemoryPoolDaemon
      * GpuSharedMemoryPoolProxy
