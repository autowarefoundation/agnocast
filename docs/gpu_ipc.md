# GPU IPC Design

GPU payloads can be shared across processes without copying their contents. Instead, only a
reference to the GPU allocation is passed between processes.

This reference is deliberately not a pointer. A device address is valid only within the process that
mapped the allocation, so each process resolves the reference into its own local address space.

The underlying physical memory location—whether device memory on a discrete GPU or shared DRAM on an
SoC—is irrelevant to the design. What matters is that every process can map the same allocation,
ensuring that the payload is neither moved nor duplicated during sharing.

This document records the architectural decisions shaping the feature as a whole. Localized
implementation decisions belong alongside their respective code.

```text
     Publisher process                                      Subscriber process
     -----------------                                      ------------------

     host VA 0x7f10_0000 ---.                          .--- host VA 0x7f10_0000
                            |    the same address      |         (read-only)
                            v      in every process    v
                    +------------------------------------------+
                    |  message, in host shared memory          |
                    |    header, width, point_step, ...        |
                    |    payload reference:  region 7, slot 2  |
                    +------------------------------------------+

                     the reference names memory without addressing it;
                        each process resolves it for itself, below

     device VA 0x7f00_0000 -.                          .- device VA 0x7d80_0000
                            |   a different address    |
                            v      in each process     v
                    +------------------------------------------+
                    |  one GPU allocation  =  region 7         |
                    |   +-------+-------+-------+-------+      |
                    |   | slot 0| slot 1| slot 2| slot 3|      |
                    |   +-------+-------+---^---+-------+      |
                    |                       |                  |
                    |   base + 2 * slot_size = this payload    |
                    +------------------------------------------+
```

## A message identifies its memory location using a region and a slot

An allocation is divided into equal-sized slots. Borrowing a message reserves a slot and records two
identifiers within the message: the region to which the slot belongs, and the slot's index within
that region. Upon receiving its first message from a given publisher, a subscriber reads these
values and maps the corresponding region. From then on, it resolves every subsequent message into
its own local address space independently.

Region IDs remain unique throughout the kernel module's lifetime and are never reused. Consequently,
a message referencing a defunct region resolves to nothing rather than mapping to an unrelated
allocation. Because these two identifiers originate from another process, they are strictly
bounds-checked before being converted into an address.

## The kernel module holds the region's lifetime

The module operates strictly within the control plane: it never reads or writes device memory. It
handles two critical tasks that userspace cannot perform for itself:

First, it holds a liveness reference to the allocation, ensuring the memory outlives the process that
created it. Even if a publisher crashes, a subscriber reading its message is accessing memory that
remains validly allocated.

Second, it installs a descriptor for each importer, eliminating the need to pass descriptors between
processes over side channels.

## Why CUDA IPC is Excluded

CUDA IPC (`cudaIpcGetMemHandle`) seems like the obvious candidate for sharing device memory between
processes, but it is deliberately omitted. The rationale is as follows:

- **The Requirement:** A subscriber may still be reading a payload when the publisher that created
  it dies—which is precisely why the module maintains a liveness reference. A mechanism qualifies
  only if the memory allocation can outlive its creating process.
- **Why Chosen Mechanisms Qualify:** For both CUDA VMM and NvSciBuf, allocations are backed by
  objects that a third party can hold on the creator's behalf (a file descriptor for CUDA VMM, and
  native reference counting for NvSciBuf). The module holds onto these, allowing memory to survive
  process exit.
- **Why CUDA IPC Fails:** A CUDA IPC handle is merely an opaque token rather than a handle to a
  reference-counted kernel object. There is nothing for the module—or any other process—to hold. The
  allocation remains strictly owned by the exporting process and is freed upon its exit, leaving
  importers with dangling device pointers.
- **What Support Would Require:** Restoring lifetime guarantees would require moving allocation
  ownership out of the publisher. A separate, long-lived central daemon would need to execute every
  `cudaMalloc` and distribute handles, ensuring the owner never exits while a payload is active.
- **Why It Isn't Worth It:** Such a daemon introduces a new component to deploy, supervise, and
  version-match, as well as a single point of failure for all GPU topics. Furthermore, it would
  exist solely for CUDA IPC, whereas other mechanisms require no such daemon because their
  underlying kernel objects handle lifetime natively. Finally, CUDA IPC is a legacy interface being
  actively superseded by the CUDA VMM API for this exact reason.

## A publisher grows for payload size, never for message rate

Choosing an allocation size requires predicting a payload size that the publisher may not know in
advance. Rather than failing a borrow request when no existing region can accommodate the payload,
the publisher allocates an additional region. The resulting message identifies the region containing
its payload. Adding a region requires no cross-process coordination because a subscriber lazily maps
a previously unseen region when it first receives a message referring to that region.

Region growth serves this purpose alone. A borrow request may also fail for a different reason: a
region large enough for the payload exists, but all of its slots are still occupied by messages in
flight. Allocating another region in that situation would be a mistake. The number of slots is
derived from the publisher's QoS depth, so exhausting them means that the publisher already has at
least as many outstanding messages as its configured depth allows. Growing the pool would silently
bypass that limit and respond to a consumer that is not keeping up by consuming more device
memory—a resource shared across the entire machine. Instead, the borrow request fails, just as an
enqueue operation fails or a message is dropped when a bounded queue is full.

Size-driven growth must itself be bounded because payload sizes may vary without a known upper
limit, whereas device memory is finite. The bound is enforced where the necessary knowledge resides:
only the publisher can determine that a region no longer contains any live messages and can
therefore be released in favor of a differently sized region. The kernel module cannot make that
determination because it never observes which region contains the payload of any particular message.

## Reclaiming Regions from Departed Publishers

A subscriber caches every region it maps and releases one only when both of the following hold:

1. No live reference to the region remains within the subscriber process.
2. The kernel module no longer recognizes the publisher that exported it.

Caching is what makes processing every frame after the first free of overhead. Releasing eventually
is necessary because an imported handle holds its own driver reference, so a mapping that is never
released is device memory that can never be freed—accumulated for every region of every publisher
the subscriber has ever encountered, and unbounded if a supervisor keeps restarting one.

The two conditions divide the question by who is able to answer it. The kernel module knows whether
the publisher still exists, and because region IDs are never reused, a publisher the module has
forgotten can never refer to that region again—which is what makes the release final. Only the
subscriber process knows whether it is still holding the region, because the module's accounting of
messages in flight does not track what the local process retains.

## GPU metadata processing stays outside the shared-memory allocation window

Between borrowing a message and publishing it, host allocations intercepted by the allocator are
redirected to the process's shared-memory mempool. This is how dynamically allocated parts of the
message payload are placed in the shared-memory segment. However, the GPU driver may also perform
internal host allocations while the library processes GPU-related metadata during this interval. If
those allocations were redirected as well, long-lived driver bookkeeping would unnecessarily consume
space in a segment mapped by every subscriber.

The library therefore temporarily disables redirection to shared memory while processing GPU-related
metadata. Allocations made during that processing belong to the library or GPU driver, not to the
message. Only message-owned allocations should be redirected to the shared-memory segment.
