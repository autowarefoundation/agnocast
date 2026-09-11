# GPU IPC Design

Agnocast's host data plane keeps a message's bytes in shared memory so that no process copies them.
For a payload a GPU produces or consumes, the same argument applies one level down: a point cloud
filtered on the device and then read by two more nodes should not travel to host memory and back to
say so. GPU IPC puts such a payload in shared GPU device memory.

This document is the design rationale for that mechanism. Source comments are kept to what is local
to the code they sit next to; the reasoning behind the shape of the thing is here.

| Component | Where |
|---|---|
| Region registry and authorization | `agnocast_kmod/agnocast_ioctl.c` (`*_gpu_region`) |
| Liveness reference, released on teardown | `agnocast_kmod/agnocast_internal.{c,h}` (`gpu_region_info`) |
| Message handle and its declarations | `src/agnocastlib/include/agnocast/internal/gpu_message.hpp` |
| Region table, slot pool | `src/agnocastlib/src/internal/gpu_{region_registry,slot_pool}.cpp` |
| Allocation mechanisms (CUDA VMM) | `src/agnocast_gpu/src/{vmm_backend,cuda_driver_loader,register_backend}.cpp` |
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

Two properties follow. A region id is unique for the module's lifetime and never reused — the
counter skips the reserved value 0 if it ever wraps — so a stale id resolves to nothing rather than
to some unrelated region. And once a region is mapped, the id alone selects the mapping: *resolving*
a slot needs no knowledge of which topic or publisher produced the message. Establishing the mapping
in the first place does, which is why a message also carries its publisher's id; when the receive
path starts telling a subscriber which publisher a message came from, that field goes away.

Everything the pair is checked against — slot count, slot size, mapped size — arrives from another
process. An out-of-range slot index, or a payload larger than a slot, yields a null pointer rather
than a wild device address, and that check is applied on every resolution.

What those numbers are *not* checked against is the kernel's own view of the allocation: nothing
compares them with the size of the file behind the handle. Both the module and the importer apply
the same self-consistency arithmetic — the slots must fit within the declared mapping — so a
registration whose three numbers agree with each other is accepted. A payload size derived from a
message's ROS fields is a second, independent source of truth, so `gpu_data_size()` is capped by
what the handle actually reserved; the handle's own `size()` is the authoritative extent.

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

- A registration must be internally consistent — a mechanism whose handle is a descriptor must
  bring one and no descriptor blob, and vice versa, and the slots must fit the declared mapping.
  This is a presence test, not a type test: no in-kernel interface would let the module confirm that
  a descriptor is GPU memory, so it checks only that it is not its own device, whose file would pin
  the module.
- Only the process that owns a publisher may register or remove memory under its name.
- A descriptor is handed out only to a caller naming a subscriber of that topic which belongs to the
  calling process. Without that check the module would be a general descriptor-passing channel keyed
  by topic name, where the host data plane requires a registered subscription before it maps
  anything.

So the boundary is membership of the topic, exactly as it already is for the host shared memory a
subscriber maps. A peer on the topic is trusted; a process that merely knows a topic name is not.

Imports are mapped read-only, and a subscriber's message handle yields a `const` device pointer to
match, so a *buggy* subscriber cannot corrupt a payload others are still reading — a write through
such a mapping faults. It is not a guarantee against a hostile one: the module hands over the
publisher's own open file unchanged, and an importer holding the imported allocation can grant
itself write access on its own mapping. "The publisher writes, nobody else does" is enforced within
the library, not by the kernel.

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

- **Slots are sized by powers of two**, above a floor of the 256-byte alignment a device pointer
  from `cudaMalloc` would have — so slot *k*, which begins at *k* × slot size, is aligned too.
  Sizing slots to the exact requested capacity would mean a new region for every new payload size;
  bucketing means a payload that grows keeps reusing its region until it doubles, at a cost of less
  than 2x in device memory. Past 2 GiB a power of two no longer fits in a slot size, so the size
  tracks the capacity and is rounded to the alignment instead.
- **Slot count comes from QoS depth**, which is what "how many messages may be in flight at once"
  already expresses: a region holds depth slots plus one being filled. KeepAll is the exception —
  it reports a depth of 0, so it gets two slots and then grows by adding regions.
- **The number of regions is capped per publisher**, because growth is driven by a userspace process
  and each region pins device memory plus a descriptor the module holds. A process with publishers
  on many topics is not bounded by that cap.

Reaching the cap is not terminal: a region holding no message can be released to make room for a
differently sized one. Only the publisher can know a region is empty — the module never sees which
region a message was written into — and it knows it from a fact it already has: every slot free means
every message that used the region has been destroyed, which for a published message means the
module released its entry, which in turn means every subscriber had dropped its reference.

## A subscriber reclaims a departed publisher's regions

The mapping a subscriber makes on first receipt is the other half of the lifetime question. Keeping
it is what makes every later frame free, but keeping it *forever* would mean a subscriber accumulates
a mapping — and, because an imported handle holds its own driver reference, a share of device memory
that can never be freed — for every region of every publisher it has ever seen. A publisher
restarting under a supervisor is enough to grow that without bound.

A subscriber can release an imported region once the module no longer knows the publisher that
exported it. That is exactly the right moment, and it follows from the same entry accounting: while
this process holds a received message it holds a reference on that message's entry, so the
publisher's registration cannot have been dropped; once it has been, no handle to any of its
messages exists here, and none can appear later because region ids are never reused. Reclamation is
therefore done when a new region is imported — growth in one publisher's regions pays for reclaiming
a departed one's — which leaves at most one stale generation mapped in a process that never imports
again.

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

Transfers are declared rather than written inside the callable so the library owns them: it checks
that host buffers are page-locked, and owning them is what would let transfers be timed separately
from kernels later. Only an "upload first, download last" shape is expressible; interleaved copies
belong in the callable, where they count as GPU work. The admission control the waiting argument
above anticipates is not implemented yet — the gate is a pair of empty calls at the points it will
occupy.

Declaring the messages a submission touches is load-bearing rather than documentation: declaring a
message as read is what establishes the mapping for a publisher this process has not seen before.

## Interaction with the shared-memory mempool

Everything allocated between borrowing a message and publishing it comes from the process's
shared-memory mempool — that is how a message's payload gets there — and CUDA does not distinguish
its own host allocations from a message's. A first submission inside that window therefore parks the
driver's one-time bookkeeping in the mempool, where it stays for the life of the process. None of it
is a leak and none of it is per message, but it is accounted against message memory and is visible
to every subscriber that maps the segment.

A publishing node avoids the bulk of this by submitting empty work once, outside any window, on each
thread that will submit; the samples do exactly that on the thread their single-threaded executor
spins. Note the limit: with a multi-threaded or callback-isolated executor the callback threads
belong to the library, and there is no hook on which to prime them, so each pays its own one-time
driver setup inside the window.

Some one-time allocations are deliberately left there in any case, since hoisting them would mean
either exposing the steps as API or suspending the mempool: the GPU backend's own initialization
(two `dlopen`s and the driver's context setup, which land on the normal heap only because the first
borrow reaches them before it opens the window), a payload's first stream-ordered device allocation,
and the first frame from each publisher mapping its region. Each is once per process or once per
publisher. Nothing is allocated there per message — the per-frame path deliberately avoids even
naming the topic, because that copy would be a mempool allocation on the message path.

## Limits this design accepts

- **A publisher and its subscribers must be on the same GPU.** A region is tagged with its device
  UUID rather than an ordinal, since `CUDA_VISIBLE_DEVICES` makes ordinals process-relative, and an
  import across devices is refused rather than attempted. On a MIG-partitioned GPU the UUID is the
  compute instance's, so an import across that boundary is refused too. The module stores the UUID
  but never compares it; the refusal is the importing library's. Within a process the device is
  bound once, by whichever GPU call comes first, and a later publisher created while a different
  device is current still allocates on the bound one.
- **A destroyed context is not recovered.** `cudaDeviceReset()` destroys the retained primary
  context's resources, after which pushing it still succeeds and every call made on it fails; the
  same is true after any sticky device fault. GPU messaging stays broken for the life of the
  process.
- **`kAllocateDeviceAsync` does not track the size it allocated.** A non-null target is reused as
  is, so a transfer whose element count grows would run past the first allocation. The option is
  for a target of fixed size, and the pointer it hands back is the caller's to free.
- **A GPU topic does not bridge to ROS 2.** A GPU message type is not a generated ROS type, so it
  registers with an empty type name and the Agnocast-ROS 2 bridge cannot carry it.
- **Device work must not outlive a region.** Releasing a region synchronizes the context first, so a
  violation is reported at a deterministic point rather than faulting later, but asynchronous work
  still referencing a region being released is a bug in the node.
- **A publisher's own mapping is released only when the region is.** A region still holding a
  message outlives its publisher's teardown, by design; its address space is reclaimed at process
  exit.
- **A GPU message type is not assignable, and its base's `data` member is shadowed, not removed.**
  Assignment between two GPU messages would move a reserved slot from one to the other, so it is
  deleted; but a reference to the `sensor_msgs` base still exposes the host `data` vector, which
  nothing reads. Converting a handle to one of the base types is rejected at compile time, because
  it would delete through a destructor that is not virtual and lose the payload handle with it.
