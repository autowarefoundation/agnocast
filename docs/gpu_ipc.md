# GPU IPC Design

GPU payloads can be shared across processes without copying: messages carry only a region ID and
slot index identifying the payload.

A region is a GPU allocation divided into equal-sized slots.

This identifier pair deliberately replaces a pointer: device addresses are process-local, so each
process resolves the pair into its own address space.

This document records feature-wide architectural decisions; local implementation decisions belong
beside their code.

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

## Messages identify memory by region and slot

Borrowing reserves a slot and records its region ID and slot index in the message.

On receiving a publisher's first message, a subscriber reads these identifiers and maps the region,
then resolves subsequent messages into its own address space.

Region IDs are unique throughout the kernel module's lifetime and never reused, so references to
defunct regions resolve to nothing, never unrelated allocations.

Both identifiers originate in another process and undergo strict bounds checks before address
conversion.

## The kernel module maintains region lifetime

The kernel module operates exclusively in the control plane, never reading or writing device memory.

It performs two critical tasks:

- Holds an allocation liveness reference so memory outlives its creator, remaining valid for
  subscribers reading messages even after a publisher crashes.

- Installs descriptors for importers, eliminating descriptor transfer through interprocess side
  channels.

## Why CUDA IPC is excluded

CUDA IPC (`cudaIpcGetMemHandle`), though seemingly the obvious choice for interprocess device-memory
sharing, is deliberately excluded:

- **Requirement:** Subscribers may still read payloads after their publisher dies, motivating the
  kernel module's liveness reference. Allocations must therefore outlive their creator.

- **Why CUDA VMM and NvSciBuf qualify:** Their backing objects support third-party retention on the
  creator's behalf: file descriptors for CUDA VMM, native reference counting for NvSciBuf. The
  kernel module retains these, preserving memory after process exit. Only CUDA VMM is implemented;
  NvSciBuf holds the reserved mechanism number 2. Its export is reconciled against the destination
  endpoint, so one importer's bytes mean nothing to another, and serving it needs an export produced
  per request rather than the single retained handle the module holds today. That is a different
  shape, so it is left to be designed alongside its implementation.

- **Why CUDA IPC fails:** Its opaque token does not reference a reference-counted kernel object;
  neither the kernel module nor another process can retain ownership. Allocations remain exclusively
  exporter-owned and are freed on exporter exit, leaving importers with dangling device pointers.

- **Required workaround:** Lifetime guarantees would require transferring allocation ownership from
  publishers to a separate, long-lived central daemon executing every `cudaMalloc` and distributing
  handles. It must remain alive while any payload is active.

- **Why not:** Such a daemon would introduce a single point of failure across all GPU topics, yet be
  unnecessary for any backend other than CUDA IPC. These drawbacks, combined with CUDA IPC's legacy
  status, leave little motivation to support it.

## Publishers grow for payload size, never message rate

Allocation sizing requires predicting potentially unknown payload sizes.

When no existing region fits a payload, the publisher allocates another region instead of failing
the borrow.

The message identifies its payload's region. Growth requires no interprocess coordination:
subscribers lazily map unseen regions on first receiving messages referencing them.

Growth serves only this purpose.

Borrowing can also fail when a suitable region exists but every slot holds an in-flight message.

Allocating another region then would be wrong: slot counts derive from publisher QoS depth, so
exhaustion means outstanding messages already meet or exceed that limit.

Growth would silently bypass it, compensating for a lagging consumer by consuming more machine-wide
device memory. Device memory differs from the host path's mempool here: that mempool is reserved
per process up front, so absorbing a lagging consumer costs the process its own reservation, while
a device allocation is taken from every process on the machine.

Instead, the publisher asks the kernel module to release whatever the QoS depth no longer retains,
and borrowing fails if that frees no slot — analogous to failed enqueueing or message dropping in a
full bounded queue.

That release is not an optimization. A slot is returned only by destroying its message, and the
kernel module names releasable messages only when something is published, so a publisher holding no
free slot has nothing to publish and would never learn that its slots had come free. Without it the
first transient excess would stall the publisher permanently instead of costing it a frame.

Size-driven growth must also be bounded: payload sizes may have no known upper limit, but device
memory is finite.

Publishers enforce the bound because only they can determine whether a region contains no live
messages and can be released for a differently sized region.

The kernel module cannot: it never observes which region holds a particular message's payload.

## Reclaiming regions a subscriber can no longer need

Subscribers cache every mapped region. Caching eliminates overhead after the first frame.

Eventual release is necessary because imported handles retain driver references: unreleased mappings
prevent device-memory reclamation, accumulating across every region of every publisher encountered,
without bound under repeated supervised restarts.

A region is released when both hold:

1. No live reference to the region remains within the subscriber process.

2. The kernel module no longer holds the region itself.

These conditions reflect who has the necessary knowledge.

Only subscribers know their locally retained references. Kernel module accounting of in-flight
messages does not track these, and it is dropped by subscription teardown while handles are still
held, so it cannot stand in for the first condition.

Only the kernel module knows whether a region still exists. It is asked about the region rather
than about its exporting publisher, because the two answers differ: a publisher that retires a
region and carries on still exists, and so does a restarted publisher that has been assigned its
predecessor's topic-local id. Both would keep a defunct region mapped forever.

Region IDs are unique throughout the kernel module's lifetime and never reused, so the question
needs no publisher or subscriber to authorize against — which matters, because the case it exists
for is precisely the one where the exporting publisher is already gone and the importing
subscription may be too. For the same reason the answer is final: nothing can bring that ID back.
