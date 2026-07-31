// Declaration of the CUDA stream a publisher / subscription uses for its GPU work.
//
// This header is CUDA-free: agnocastlib has no build-time CUDA dependency, so a
// stream is carried as an opaque `void *` (cudaStream_t is a pointer typedef) plus a
// kind tag. The tag exists because a *null* handle is ambiguous across the CUDA-free
// boundary: libagnocast_cuda resolves the unsuffixed cudaEventRecord through its own
// dlopen'd libcudart, where a null stream always means the LEGACY default stream --
// even when the caller was compiled with `nvcc --default-stream per-thread`. So the
// intent is stated explicitly and never inferred from a null pointer.
#pragma once

#include "agnocast/agnocast_public_api.hpp"

namespace agnocast
{

/**
 * @brief Which CUDA stream Agnocast records / waits its GPU-IPC ordering event on.
 */
AGNOCAST_PUBLIC
enum class CudaStreamKind : int {
  /// No stream declared: use the legacy default stream (`cudaStreamLegacy`).
  /// Correct under both `nvcc --default-stream` modes without knowing which one the
  /// user compiled with, because legacy-stream operations implicitly synchronize
  /// with every *blocking* stream and the per-thread default stream is a blocking
  /// stream. It is a GPU-side barrier, not a host synchronization, so the
  /// no-argument path stays asynchronous.
  kLegacyDefault = 0,
  /// Use the per-thread default stream (`cudaStreamPerThread`).
  ///
  /// **Thread-affine, unlike every other kind.** `cudaStreamPerThread` resolves to a
  /// different stream per calling thread, and two threads' per-thread default streams do
  /// not order against each other (they are both *blocking* streams, which implicitly
  /// synchronize only with the legacy stream). So this kind is correct only when the
  /// message take, the callback, and the drop of the last message reference all happen on
  /// one thread — true for the single-threaded and callback-isolated executors, not for a
  /// multi-threaded one, and not if a callback hands a copy of the message to another
  /// thread. Prefer an explicit stream; that is what makes any executor safe.
  kPerThreadDefault = 1,
  /// Use the handle in CudaStream::handle.
  kExplicit = 2,
};

/**
 * @brief An opaque, CUDA-header-free reference to a CUDA stream.
 *
 * Assign a `cudaStream_t` directly (it converts implicitly to `void *`):
 * @code
 *   cudaStream_t stream;
 *   cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
 *   agnocast::PublisherOptions options;
 *   options.cuda_stream = stream;
 * @endcode
 *
 * Declared streams should be created with `cudaStreamNonBlocking`: in a component
 * container, a stream-unaware node recording on the legacy default stream would
 * otherwise inject a barrier across a stream-aware neighbour's blocking streams.
 */
AGNOCAST_PUBLIC
struct CudaStream
{
  CudaStreamKind kind{CudaStreamKind::kLegacyDefault};
  /// Valid only when `kind == kExplicit`.
  void * handle{nullptr};

  CudaStream() = default;

  /// Declares an explicit stream. Intentionally not `explicit` so that
  /// `options.cuda_stream = my_cuda_stream;` compiles. A null handle means
  /// "nothing declared" and selects the legacy default stream.
  // NOLINTNEXTLINE(google-explicit-constructor)
  CudaStream(void * stream)
  : kind(stream == nullptr ? CudaStreamKind::kLegacyDefault : CudaStreamKind::kExplicit),
    handle(stream)
  {
  }

  static CudaStream legacy_default() { return CudaStream{}; }

  static CudaStream per_thread_default()
  {
    CudaStream stream;
    stream.kind = CudaStreamKind::kPerThreadDefault;
    return stream;
  }

  /// The kind as the plain `int` the C ABI carries.
  int kind_value() const { return static_cast<int>(kind); }
};

}  // namespace agnocast
