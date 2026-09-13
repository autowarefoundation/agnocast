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

## Publishers grow rather than fail a borrow

Allocation sizing requires predicting potentially unknown payload sizes.

When no existing region fits a payload, the publisher allocates another region instead of failing
the borrow.

The message identifies its payload's region. Growth requires no interprocess coordination:
subscribers lazily map unseen regions on first receiving messages referencing them.

Borrowing can also find every slot of a fitting region already holding an in-flight message. That
is the same situation, and it grows too, for the same reason the host path does.

Host publishing cannot fail for want of room: a borrow allocates from the process mempool, so QoS
depth is a retention target the kernel module applies lazily at each publish, never a bound on
borrowing. A subscriber still referencing the oldest entry when the next message is published
leaves the publisher holding more messages than its depth retains, and the host path absorbs that
by allocating. GPU slots are preallocated per region, so the same excess would exhaust them.

Two steps give the excess the outcome it has on the host side, and a borrow takes at most one of
them, because each costs a call into the kernel module.

Growth comes first: it is what the mempool does under the same pressure, bounded by the number of
regions a publisher may hold rather than by the mempool size. It also restores the ability to
publish, and publishing is what drains the backlog.

At that bound, or when the allocation fails, the publisher instead asks the kernel module to
release what QoS depth no longer retains and frees those messages, which returns their slots. This
step is what makes the failure recoverable rather than permanent: a slot is returned only by
destroying its message, the kernel module names releasable messages only when something is
published, and a publisher with no slot has nothing to publish.

Size-driven growth must also be bounded: payload sizes may have no known upper limit, but device
memory is finite.

Publishers enforce the bound because only they can determine whether a region contains no live
messages and can be released for a differently sized region.

The kernel module cannot: it never observes which region holds a particular message's payload.

Reaching the bound with every slot in flight is the one case a borrow still fails. It costs a
frame, not the topic: the release above runs on every later borrow, so publishing resumes as soon
as subscribers let go.

## Reclaiming regions a subscriber can no longer need

Subscribers cache every mapped region. Caching eliminates overhead after the first frame.

Eventual release is necessary because imported handles retain driver references: unreleased mappings
prevent device-memory reclamation, accumulating across every region of every publisher encountered,
without bound under repeated supervised restarts.

No release happens while a live reference to the region remains within the subscriber process. Only
subscribers know their locally retained references; kernel module accounting of in-flight messages
does not track these, and it is dropped by subscription teardown while handles are still held. This
condition gates both rules below.

Beyond it, two rules release a region, because a region stops being reachable in two ways.

**Its publisher is gone.** The kernel module knows whether a publisher exists. Since region IDs are
never reused, a forgotten publisher can never reference that region again, making release final.

**Its publisher retired it.** A publisher bounded to `MAX_GPU_REGION_NUM_PER_PUBLISHER` regions can
name no more than that many in any message it still sends, so a subscriber holding more than that
for one publisher is holding regions the publisher has already released. The bound the publisher
already enforces is therefore the bound the subscriber applies, and the oldest region ID is the one
retired first.

The first rule alone would not suffice. A publisher that retires a region and carries on answers
"still registered" for as long as it lives. The same answer arrives when a restarted publisher is
assigned a topic-local ID its predecessor used, which the kernel module restarts whenever a topic
loses its last endpoint. Both cases leave the subscriber holding a region nothing can reach, and
both are bounded by the second rule rather than detected by the first.

## GPU metadata processing stays outside the shared-memory allocation window

Between borrowing and publishing, intercepted host allocations are redirected to the process's
shared-memory mempool, placing dynamically allocated message payload components in shared memory.

GPU metadata processing during this interval may also trigger internal driver host allocations.

Redirecting these would unnecessarily place long-lived driver bookkeeping in a segment mapped by
every subscriber.

The library therefore temporarily disables shared-memory redirection during GPU metadata
processing.

These allocations belong to the library or driver; only message-owned allocations should enter
shared memory.
