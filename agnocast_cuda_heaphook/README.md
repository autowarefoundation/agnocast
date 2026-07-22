# agnocast_cuda_heaphook

`LD_PRELOAD` shared library that intercepts CUDA memory allocation so a CUDA
publisher's device buffers come from the Agnocast GPU shared-memory pool instead
of a private `cudaMalloc`, enabling zero-copy GPU sharing between processes.

This first version hooks only `cudaMalloc` and `cudaFree`. Other CUDA runtime and
driver allocation functions will be added later.

## How it works

- **`cudaMalloc`** — if the calling thread is inside a CUDA publisher's
  `borrow_loaned_message() .. publish()` window (a thread-local flag set by
  Agnocast and read through the proxy C ABI), the allocation is served from the
  pool via `agnocast_cuda_pool_allocate`. Otherwise, or if the pool cannot
  satisfy it, the real `cudaMalloc` (resolved via `RTLD_NEXT`) is called.
- **`cudaFree`** — if the pointer was handed out by the pool
  (`agnocast_cuda_pool_free` reports it as pooled), it is returned to the pool.
  Otherwise the real `cudaFree` is called.

The proxy C ABI symbols (`agnocast_cuda_in_publish_window`,
`agnocast_cuda_pool_allocate`, `agnocast_cuda_pool_free`) live in
`libagnocast_cuda.so` and are resolved lazily at runtime via `RTLD_DEFAULT`. If
they are absent (e.g. a process that does not use Agnocast CUDA messages), the
hook transparently falls back to the real CUDA functions.

## Usage

```bash
LD_PRELOAD=/path/to/libagnocast_cuda_heaphook.so ./your_cuda_publisher
```
