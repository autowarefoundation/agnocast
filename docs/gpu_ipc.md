# GPU IPC Design

A GPU payload stays in the allocation the GPU already reads and writes, and no process ever copies
it. What crosses a process boundary is a short reference to that allocation, carried in the ordinary
host shared-memory message alongside the ROS fields.

The reference is deliberately *not* a pointer. A device address is meaningful only in the process
that mapped the allocation, so each process resolves the reference into an address of its own.

Which physical memory the allocation lives in does not matter to the design: device memory on a
discrete GPU, the DRAM the GPU shares with the host on an SoC. What matters is that every process
can map the same allocation, and that nothing moves or is duplicated in order to share it.

This document records the decisions that shape the feature as a whole. Decisions local to one piece
of code belong next to that code.

## A message identifies its memory by region and slot

An allocation is divided into equally sized slots. Borrowing a message reserves one and records two
numbers in the message: which region the slot belongs to, and which slot within it. A subscriber
reads them back, maps that region the first time it sees one from a given publisher, and from then
on turns every message into an address of its own without asking anyone.

Region ids are unique for the module's lifetime and never reused, so a message naming a region that
has gone resolves to nothing rather than to an unrelated one. The two numbers arrive from another
process, so they are bounds-checked before they become an address.

## The kernel module holds the region's lifetime

The module stays control plane only: it never reads or writes device memory. It does two things
userspace cannot do for itself.

It **holds the allocation's liveness reference**, so the memory outlives the process that created
it. A subscriber reading a message whose publisher has just crashed is reading memory that is still
allocated.

It **installs a descriptor per importer**, so descriptors never have to be passed between processes
over a side channel.

## Why CUDA IPC is not one of the mechanisms

CUDA IPC (`cudaIpcGetMemHandle`) is the obvious candidate for sharing device memory between
processes, and it is deliberately absent. The reasoning runs as follows.

**The requirement.** A subscriber may still be reading a payload when the publisher that produced it
dies — that is precisely what the module's liveness reference is for. So a mechanism qualifies only
if the allocation can outlive the process that created it.

**Why the chosen mechanisms qualify.** In both, the allocation is owned by something a third party
can hold on its behalf: a file descriptor for CUDA VMM, the object's own reference count for
NvSciBuf. The module holds that, and the memory survives its creator.

**Why CUDA IPC cannot.** A CUDA IPC handle is an opaque token, not a reference to a kernel object.
There is nothing for the module — or anyone else — to hold. The allocation belongs to the exporting
process and is freed with it, leaving importers holding dangling device pointers.

**What supporting it would take.** The only way to restore the guarantee is to move ownership out of
the publisher: a separate long-lived process that performs every `cudaMalloc` and hands the handles
out, so that the owner never exits while a payload is in use.

**Why that was not worth it.** Such a daemon is a new component to deploy, supervise and
version-match, and a new single point of failure for every GPU topic. It would also exist for this
one mechanism alone — the others need nothing of the kind, because the kernel object they are built
on already provides the lifetime. CUDA IPC is additionally the older interface, superseded by the
VMM API for exactly this purpose, so the cost would buy a mechanism that is on its way out.

## The trust boundary is the host data plane's

A process that may subscribe to a topic may read that topic's payloads; a process that merely knows
a topic name may not. That is where Agnocast's host shared memory already draws the line, and GPU
payloads draw it in the same place.

It is a boundary, not a sandbox: the module cannot establish that a descriptor handed to it is GPU
memory at all, so it does not try, and the read-only access an importer gets is the importing
library's doing rather than something the kernel imposes. Within a topic, peers trust each other
exactly as much as they already do for host payloads.

## The mechanism axis is allocation and export only

How an allocation was made and made importable is the only axis the mechanism type represents.
Cross-process GPU synchronization stays independent of it: the same memory may be paired with CUDA
events, NvSciSync, or nothing at all.

CUDA VMM is implemented. NvSciBuf has a reserved place in the type, but the shape of the current
ABI cannot serve it.

## A publisher grows rather than fails

Sizing an allocation is a guess about a payload size the publisher may not know in advance. Rather
than fail a borrow that does not fit, a publisher allocates another region, and the message records
whichever region its payload went into. Additional regions need no coordination between processes,
precisely because a subscriber maps an unseen region on first receipt.

Growth is bounded on three axes: slot sizes bucket, so a growing payload keeps reusing its region;
slot count comes from the QoS depth that already expresses how many messages may be in flight; and
the number of regions one publisher holds at once is capped. Reaching that cap is not terminal,
because a region holding no message can be released to make room for a differently sized one — and
only the publisher is in a position to know a region is empty, since the module never sees which
region a message was written into.

## A subscriber reclaims a departed publisher's regions

Keeping a mapping is what makes every frame after the first free. Keeping it forever would mean a
subscriber accumulates a mapping — and, since an imported handle holds its own driver reference, a
share of device memory that can never be freed — for every region of every publisher it has ever
seen. A publisher restarting under a supervisor is enough to grow that without bound.

So an imported region is released once nothing in this process still refers to it and the module no
longer knows the publisher that exported it. The second condition is what makes the release final:
region ids are never reused, so a publisher the module has forgotten can never name that region
again. The first has to be tracked in the process itself — the module's accounting of messages in
flight is not a reliable proxy for what this process is still holding.

## The stream belongs to the submission, not to the message

GPU work is submitted through a call that owns the stream and passes it to the caller's callable, so
a message and the work touching it cannot end up on different streams. Designs that take the stream
through a publisher or a message allow writing on one stream and publishing against another, sending
out unfinished data.

That call returns only after the submitted work has completed. This is what preserves the host-side
invariant that a message is finished when its reference count reaches zero: on the GPU a read is
asynchronous, so a callback that merely launched a kernel would return with the device still
reading. Waiting costs the node its own GPU time — a quantity its schedulability analysis already
accounts for — and adds no coupling to other nodes.

Transfers are declared rather than written inside the callable so that the library owns them, and
declaring the messages a submission touches is load-bearing rather than documentation: declaring a
message as read is what establishes the mapping for a publisher this process has not seen before.

## Interaction with the shared-memory mempool

Everything allocated between borrowing a message and publishing it comes from the process's
shared-memory mempool — that is how a message's payload gets there — and CUDA does not distinguish
its own host allocations from a message's. A first submission inside that window therefore parks the
driver's one-time bookkeeping in the mempool, where it stays for the life of the process. None of it
is a leak and none of it is per message, but it is accounted against message memory and is visible
to every subscriber that maps the segment.

A publishing node avoids the bulk of it by submitting work once outside any window, on each thread
that will submit — which is only possible for threads the node owns. Where the executor creates its
own callback threads, there is no hook on which to prime them.

## Limits this design accepts

- **A publisher and its subscribers must be on the same GPU.** An allocation belongs to a device, so
  a region carries the identity of the device it was made on and an import across that boundary is
  refused rather than attempted. A process binds itself to one device the first time it touches the
  GPU.
- **A GPU topic does not bridge to ROS 2.** A GPU message type is not a generated ROS type, so it
  registers with no type name and the Agnocast-ROS 2 bridge cannot carry it. Both ends say so once
  at construction rather than leaving a topic that silently never arrives.
