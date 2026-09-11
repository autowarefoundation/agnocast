# GPU IPC Design

A GPU payload remains in the allocation that the GPU already reads and writes, with no process ever
copying it. What crosses process boundaries is merely a short reference to that allocation,
transmitted within standard host shared-memory messages alongside ROS fields.

This reference is deliberately not a pointer. A device address is valid only within the process that
mapped the allocation, so each process resolves the reference into its own local address space.

The underlying physical memory location—whether device memory on a discrete GPU or shared DRAM on an
SoC—is irrelevant to the design. What matters is that every process can map the same allocation,
ensuring nothing is moved or duplicated during sharing.

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

## A publisher grows for size, never for rate

Sizing an allocation is a guess about a payload size the publisher may not know in advance. Rather
than fail a borrow that does not fit, a publisher allocates an additional region, and the message
records which one its payload went into. Additional regions require no coordination between
processes, precisely because a subscriber maps an unseen region on first receipt.

Growth answers that question and no other. A borrow can also find no slot for the opposite reason—a
region sized for the payload exists, but every one of its slots is still held by a message in
flight—and allocating there would be a mistake. The slot count comes from the publisher's QoS depth,
so a full region means the node already has as many messages outstanding as it declared it wanted.
Growing would silently overrule that number, and would answer a consumer that is not keeping up by
taking more of a resource the whole machine shares. A full region fails the borrow instead, the way
a full queue drops.

Growth for size is bounded in turn, because payload sizes can vary without limit while device memory
cannot. Where a bound has to be enforced, it is enforced where the knowledge sits: only the publisher
can tell that a region holds no message and may be given up for a differently sized one, since the
module never sees which region a message was written into.

## Reclaiming Regions from Departed Publishers

Caching a mapping is what makes processing every frame after the first zero-overhead. However,
retaining mappings indefinitely causes a subscriber to accumulate mappings—and, because an imported
handle holds its own driver reference, unfreeable device memory—for every region of every publisher
it ever encounters. If a publisher is repeatedly restarted by a supervisor, this resource footprint
would grow without bound.

To prevent this, an imported region is released once two conditions are met:

1. No active references to the region remain within the local subscriber process.
2. The kernel module no longer recognizes the publisher that exported it.

The second condition makes the release completely safe: because region IDs are never reused, a
publisher that the module has forgotten can never reference that region again. The first condition
must be tracked internally by the subscriber process itself, as the module's accounting of in-flight
messages is not a reliable indicator of what the local process is still actively holding.

## GPU work stays out of the shared-memory window

Everything allocated between borrowing a message and publishing it comes from the process's
shared-memory mempool—that is how a message's payload gets there. The GPU driver allocates host
memory of its own on paths the library must call from inside that interval, and left alone, that
bookkeeping would become a permanent resident of the segment every subscriber maps.

The library therefore suspends the window around its own work and the driver's, and restores it
around the caller's. The division is what makes this safe: only the caller's code can allocate
something that belongs to the message, and only the message's allocations belong in the segment.
