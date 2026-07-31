# agnocast_cuda_heaphook

`LD_PRELOAD` shared library that intercepts CUDA memory allocation so a CUDA
publisher's device buffers come from the Agnocast GPU shared-memory pool instead
of a private `cudaMalloc`, enabling zero-copy GPU sharing between processes.

It also interposes stream creation, purely to detect one configuration that would
otherwise break Agnocast's GPU-side ordering silently (see
[Non-blocking stream detection](#non-blocking-stream-detection)).

## How it works

### Allocation routing

An allocation is served from the pool when **all** of the following hold, and goes to
the real CUDA allocator otherwise:

- `libagnocast_cuda.so` is loaded in the process (the proxy C ABI resolves), and
- the calling thread is inside a CUDA publisher's `borrow_loaned_message() ..
  publish()` window (a thread-local flag set by Agnocast and read through the proxy
  C ABI), and
- we are not already inside a fallback call to a real CUDA allocator (see
  [Re-entrancy](#re-entrancy)), and
- the pool can satisfy the request — otherwise publishing still works, just not
  zero-copy for that message.

A deallocation is returned to the pool when `agnocast_cuda_pool_free` reports that the
pool owns the pointer; otherwise the real deallocator is called.

For the *stream-ordered* deallocators (`cudaFreeAsync`, `cuMemFreeAsync`) the stream is
synchronized first. Returning a slot to the pool is a synchronous, cross-process act — the
daemon may hand it straight to another publisher — while those APIs promise the memory
stays valid for work already enqueued on the stream, and the pool cannot defer a slot
release. Ownership is checked first (`agnocast_cuda_slot_id_from_ptr` reports it without
freeing), so a non-pooled pointer never pays for the synchronization.

Hooked allocation entry points:

| API | Allocate | Free |
|---|---|---|
| Runtime | `cudaMalloc`, `cudaMallocAsync` | `cudaFree`, `cudaFreeAsync` |
| Driver | `cuMemAlloc`, `cuMemAlloc_v2`, `cuMemAllocAsync` | `cuMemFree`, `cuMemFree_v2`, `cuMemFreeAsync` |

Both the plain and `_v2` driver spellings are exported: `cuda.h` rewrites `cuMemAlloc`
to `cuMemAlloc_v2` at compile time, so an application's undefined reference carries the
suffix, while code resolving the name dynamically may not.

Returning a pooled pointer from an *async* allocator is safe and strictly stronger than
the API promises: the pointer is valid immediately rather than in stream order.

`cuMemCreate` is interposed but **not** routed to the pool: the virtual-memory-management
allocator returns an allocation *handle* that the caller maps at an address of its own
choosing, which cannot be substituted with a pooled pointer. The hook passes it through
and warns once if it is called inside a publish window, so the cause is visible before
`publish()` aborts with "was not allocated from the GPU pool".

### Non-blocking stream detection

`cudaStreamCreateWithFlags`, `cudaStreamCreateWithPriority`, `cuStreamCreate` and
`cuStreamCreateWithPriority` are interposed only to notice that the process created a
stream with `cudaStreamNonBlocking` / `CU_STREAM_NON_BLOCKING`, which is reported to
`libagnocast_cuda` through `agnocast_cuda_note_non_blocking_stream()`. All four accept the
flag, so all four have to be covered.

Why this matters: when a publisher or subscription does not declare a stream, Agnocast
records/waits its ordering event on a *default* stream, which implicitly synchronizes
with every **blocking** stream. A non-blocking stream is exempt by definition — that is
exactly what the flag opts out of — so the data-ready event would complete while the
kernel is still writing, and every subscriber's wait would be a no-op. That is
timing-dependent, silent data corruption, so Agnocast aborts with an actionable message
naming the fix instead. This hook is how it learns the combination is present.

### Re-entrancy

A hooked runtime function that falls back to the real implementation may internally
reach a hooked *driver* function. A thread-local guard makes every hook a pass-through
for the duration of such a call, so the CUDA runtime is never handed a pooled pointer
for its own internal bookkeeping. The guard saves and restores the previous value rather
than clearing it, so it nests correctly — an inner pass-through must not re-enable pooling
for the outer call that is still running — and it is restored on unwind.

### Symbol resolution

The real CUDA functions are resolved via `RTLD_NEXT`. The proxy C ABI symbols live in
`libagnocast_cuda.so` and are resolved lazily via `RTLD_DEFAULT`. If they are absent
(e.g. a process that does not use Agnocast CUDA messages), the hook transparently falls
back to the real CUDA functions.

## Known gaps

- An application that resolves driver entry points through `cuGetProcAddress` (as
  libcudart does internally) bypasses symbol interposition entirely. Such allocations
  are not pooled, and `publish()` reports it.
- `cudaMallocPitch` / `cudaMalloc3D` and the texture/array allocators are not hooked.

## Usage

```bash
LD_PRELOAD=/path/to/libagnocast_cuda_heaphook.so ./your_cuda_publisher
```
