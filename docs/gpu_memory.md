# GPU IPC Design

Agnocast's host data plane keeps a message's bytes in shared memory so that no process copies them.
For a payload a GPU produces or consumes, the same argument applies one level down: a point cloud
filtered on the device and then read by two more nodes should not travel to host memory and back to
say so. GPU IPC puts such a payload in shared GPU device memory.

This document is the design rationale for that mechanism. Source comments are kept to what is local
to the code they sit next to; the reasoning behind the shape of the thing is here.

| Component | Where |
|---|---|
| Region registry, liveness, authorization | `agnocast_kmod/agnocast_ioctl.c` (`*_gpu_region`) |
| Message handle, region table, slot pool | `src/agnocastlib/include/agnocast/internal/gpu_*.hpp` |
| Allocation mechanisms (CUDA VMM) | `src/agnocast_gpu/src/vmm_backend.cpp` |
| Submission API, GPU message types | `src/agnocast_gpu/include/agnocast/gpu/` |

## A message identifies its memory by region and slot, never by address

A device virtual address is meaningful only in the process that mapped it, so no message ever
carries one. Instead, a region of device memory is divided into equally sized slots, and the
publisher does two things when it borrows a message: it reserves one free slot for the payload, and
it records *in the message itself* the id of the region that slot belongs to together with the
slot's index.

A subscriber receiving the message reads those two numbers back out. The region id tells it which
region the payload was written into, so it knows what to map — on the first message from a given
publisher it asks the kernel module for that region and maps it; on every later message the mapping
is already in place. The slot index then gives the payload's offset within the region, which the
subscriber adds to *its own* mapping base to get an address valid in *its own* process.

```text
message in host shared memory        GPU device memory
+-----------------------------+      +--------------------------------+
| header, height, width, ...  |      | slot 0 | slot 1 | slot 2 | ... |
| data: {region_id, slot_index|----->|        |   ^    |        |     |
|        count, publisher_id} |      |        |   |    |        |     |
+-----------------------------+      +--------------------------------+
                                                 |
        each process maps the region at its own address and
        computes slot_index * slot_size from its own base
```

Two properties follow. A region id is unique for the module's lifetime and never reused, so a stale
id resolves to nothing rather than to some unrelated region. And the pair is self-describing: a
subscriber needs no knowledge of which topic or publisher produced a message to turn it back into an
address, because the region id alone selects the mapping.

Everything the pair is checked against — slot count, slot size, mapped size — arrives from another
process, so every import and every resolution validates it rather than trusting it. An out-of-range
slot index or an oversized payload yields a null pointer, never a wild device address.

## The kernel module holds the region's lifetime

The module remains control plane only: it never reads or writes device memory. It does two things
userspace cannot do for itself.

It **holds the region's liveness reference**. A publisher exports its allocation as a file
descriptor and the module keeps that open file for as long as any message from the publisher may
still be in flight, so device memory outlives the publishing process. A subscriber reading a message
whose publisher has just crashed is reading memory that is still allocated.

It **installs a descriptor per importer**. A subscriber asks for the region whose id it read out of
a message, and the module installs a new descriptor for the same open file into the calling process,
so descriptors never have to be passed between processes over a side channel.

This is why CUDA IPC is not among the supported mechanisms. Both mechanisms that are give the
allocation a lifetime independent of its creator — a VMM allocation is held by a descriptor, an
NvSciBuf object by its own reference count. A CUDA IPC handle is an opaque token with no backing
kernel object, so nothing can hold a reference on the allocation's behalf: the memory is freed when
the exporter dies and subscribers are left with dangling device pointers. Restoring that guarantee
would take a dedicated process owning every allocation — another component to supervise, and a new
single point of failure.

## Topic membership is the trust boundary

The module cannot establish that a registered descriptor is GPU memory at all; no in-kernel
interface exposes that, and the module deliberately does not interpret an export. What it can do, it
does:

- A registration must be internally consistent — the handle must be the kind the declared mechanism
  uses, and the slots must fit the mapping.
- Only the process that owns a publisher may register or remove memory under its name.
- A descriptor is handed out only to a caller naming a subscriber of that topic which belongs to the
  calling process. Without that check the module would be a general descriptor-passing channel keyed
  by topic name, where the host data plane requires a registered subscription before it maps
  anything.

So the boundary is membership of the topic, exactly as it already is for the host shared memory a
subscriber maps. A peer on the topic is trusted; a process that merely knows a topic name is not.

Imports are mapped read-only, and a subscriber's message handle yields a `const` device pointer to
match. The publisher writes; nobody else does. A buggy or hostile subscriber cannot corrupt a
payload other subscribers are still reading.

## The mechanism axis is allocation and export only

How a region's memory was allocated and made importable is one axis, and the only one the mechanism
type represents. Cross-process GPU synchronization stays independent of it: NvSciBuf memory may be
paired with CUDA events, NvSciSync, or nothing at all. Mechanism values cross the userspace-kernel
ABI so that an importer can reject one it cannot handle, and the module takes no part in deciding
what a machine supports.

CUDA VMM is implemented. NvSciBuf is represented in the ABI but **cannot be served by it as it
stands**: its export descriptors are reconciled against the destination endpoint, so the bytes one
subscriber receives mean nothing to another, and that needs an export produced per request rather
than the single export the module stores at registration. Its place in the type and the
descriptor-blob field are reserved, not implemented.

## A publisher owns a list of regions, not one

Sizing a region is a guess about a payload size the publisher may not know in advance, so rather
than fail a borrow that does not fit, a publisher grows: it allocates another region and the message
names whichever region its payload went into. Additional regions need no coordination, precisely
because a subscriber maps an unseen region on first receipt.

Three policies keep that from growing without bound:

- **Slots are sized by powers of two.** Sizing them to the exact requested capacity would mean a new
  region for every new payload size. Bucketing means a payload that grows keeps reusing its region
  until it doubles, at a cost of less than 2x in device memory.
- **Slot count comes from QoS depth.** "How many messages may be in flight at once" is what depth
  already expresses, so a region holds depth slots plus one being filled.
- **The number of regions is capped**, because growth is driven by a userspace process and each
  region pins device memory plus a descriptor the module holds.

Reaching the cap is not terminal: a region holding no message can be released to make room for a
differently sized one. Only the publisher can know a region is empty — the module never sees which
region a message was written into — and it knows it from a fact it already has: every slot free means
every message that used the region has been destroyed, which for a published message means the
module released its entry, which in turn means every subscriber had dropped its reference.

## The stream belongs to the submission, not to the message

GPU work is submitted through a call that owns the stream and passes it to the caller's callable, so
a message and the work touching it cannot end up associated with different streams. Designs that
take the stream through a publisher or a message allow writing on one stream and publishing against
another, sending out unfinished data.

That call returns only after the submitted work has completed. This is what preserves the host-side
invariant that a message is finished when its reference count reaches zero: on the GPU a read is
asynchronous, so a callback that merely launched a kernel would return with the device still
reading. Waiting costs the node its own GPU time — a quantity its schedulability analysis already
accounts for — and adds no coupling to other nodes.

Transfers are declared rather than written inside the callable so the library owns them: it can
check that host buffers are page-locked and can time transfers separately from kernels. Only an
"upload first, download last" shape is expressible; interleaved copies belong in the callable, where
they count as GPU work.

Declaring the messages a submission touches is load-bearing rather than documentation: declaring a
message as read is what establishes the mapping for a publisher this process has not seen before.

## Interaction with the shared-memory mempool

Everything allocated between borrowing a message and publishing it comes from the process's
shared-memory mempool — that is how a message's payload gets there — and CUDA does not distinguish
its own host allocations from a message's. A first submission inside that window therefore parks the
driver's one-time bookkeeping in the mempool, where it stays for the life of the process. None of it
is a leak and none of it is per message, but it is accounted against message memory and is visible
to every subscriber that maps the segment.

A publishing node avoids this by submitting empty work once, outside any window, on each thread that
will submit; the samples do exactly that. Two one-time allocations are deliberately left inside the
window, since hoisting them would mean either exposing the steps as API or suspending the mempool: a
payload's first stream-ordered device allocation, and the first frame from each publisher mapping
its region. Both are bounded — once per process and once per publisher.

## Limits this design accepts

- **A publisher and its subscribers must be on the same GPU.** A region is tagged with its device
  UUID rather than an ordinal, since `CUDA_VISIBLE_DEVICES` makes ordinals process-relative, and an
  import across devices is refused rather than attempted. On a MIG-partitioned GPU the UUID is the
  compute instance's, so an import across that boundary is refused too.
- **A GPU topic does not bridge to ROS 2.** A GPU message type is not a generated ROS type, so it
  registers with an empty type name and the Agnocast-ROS 2 bridge cannot carry it.
- **Device work must not outlive a region.** Releasing a region synchronizes the context first, so a
  violation is reported at a deterministic point rather than faulting later, but asynchronous work
  still referencing a region being released is a bug in the node.
- **A publisher's own mapping is released only when the region is.** A region still holding a
  message outlives its publisher's teardown, by design; its address space is reclaimed at process
  exit.
