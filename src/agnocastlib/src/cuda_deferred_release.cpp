#include "agnocast/cuda_deferred_release.hpp"

#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <atomic>
#include <mutex>
#include <vector>

namespace agnocast
{

namespace
{

struct PendingRelease
{
  std::string topic_name;
  topic_local_id_t pubsub_id = 0;
  int64_t entry_id = 0;
  void * token = nullptr;
};

struct Registry
{
  std::mutex mutex;
  std::vector<PendingRelease> pending;
  GpuDoneOps ops{};
  // Read without the mutex so drain() is nearly free when nothing is deferred, which
  // matters because executors call it after every callback.
  std::atomic<size_t> pending_count{0};
};

// Deliberately leaked: flush_deferred_subscriber_releases() runs from Agnocast's
// static-destruction cleanup, which may execute after a function-local static would
// already have been destroyed.
Registry & registry()
{
  static Registry * instance = new Registry();
  return *instance;
}

bool ops_complete(const GpuDoneOps & ops)
{
  return ops.record != nullptr && ops.query != nullptr && ops.wait != nullptr;
}

// Releases the kernel-side references collected by a drain pass. Called with the
// registry mutex NOT held: release_subscriber_reference() issues an ioctl, and holding
// the mutex across it would serialize unrelated subscriber threads.
void release_all(const std::vector<PendingRelease> & completed)
{
  for (const auto & entry : completed) {
    release_subscriber_reference(entry.topic_name, entry.pubsub_id, entry.entry_id);
  }
}

}  // namespace

void set_gpu_done_ops(const GpuDoneOps & ops)
{
  if (!ops_complete(ops)) {
    return;
  }
  Registry & reg = registry();
  std::lock_guard<std::mutex> lock(reg.mutex);
  if (!ops_complete(reg.ops)) {
    reg.ops = ops;
  }
}

bool defer_subscriber_release(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id,
  const int stream_kind, void * stream)
{
  Registry & reg = registry();

  // Opportunistically retire finished markers first: it keeps the ring shallow and
  // costs one query per pending entry.
  drain_deferred_subscriber_releases();

  GpuDoneOps ops;
  {
    std::lock_guard<std::mutex> lock(reg.mutex);
    ops = reg.ops;
  }
  if (!ops_complete(ops)) {
    return false;
  }

  void * token = nullptr;
  if (ops.record(stream_kind, stream, &token) != 1 || token == nullptr) {
    return false;
  }

  // Ring full: wait for the oldest marker rather than release a reference whose reads
  // may still be in flight. This is the graceful-degradation path — a slowdown under
  // pressure instead of silent corruption.
  //
  // The wait MUST happen with reg.mutex released. It is an unbounded GPU wait, and
  // holding the mutex across it would block every other subscriber thread's release and
  // every executor's drain behind one subscription's stream — and could deadlock, since
  // this runs from ~ipc_shared_ptr and the marker's stream may be waiting on an event
  // this very thread has yet to record.
  std::vector<PendingRelease> completed;
  for (;;) {
    PendingRelease oldest;
    bool evicted = false;
    {
      std::lock_guard<std::mutex> lock(reg.mutex);
      if (reg.pending.size() < MAX_DEFERRED_SUBSCRIBER_RELEASES) {
        reg.pending.push_back(PendingRelease{topic_name, pubsub_id, entry_id, token});
        reg.pending_count.store(reg.pending.size(), std::memory_order_release);
        break;
      }
      oldest = reg.pending.front();
      reg.pending.erase(reg.pending.begin());
      reg.pending_count.store(reg.pending.size(), std::memory_order_release);
      evicted = true;
    }
    if (evicted) {
      static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
      RCLCPP_WARN_THROTTLE(
        logger, throttle_clock, 1000,
        "Agnocast GPU-IPC: %zu subscriber releases are deferred waiting for GPU reads to finish; "
        "blocking on the oldest. Consider a shallower QoS depth or fewer in-flight messages.",
        static_cast<size_t>(MAX_DEFERRED_SUBSCRIBER_RELEASES));
      ops.wait(oldest.token);
      completed.push_back(std::move(oldest));
    }
  }

  release_all(completed);
  return true;
}

void drain_deferred_subscriber_releases()
{
  Registry & reg = registry();
  if (reg.pending_count.load(std::memory_order_acquire) == 0) {
    return;
  }

  std::vector<PendingRelease> completed;
  {
    std::lock_guard<std::mutex> lock(reg.mutex);
    if (!ops_complete(reg.ops)) {
      return;
    }
    for (auto it = reg.pending.begin(); it != reg.pending.end();) {
      // Markers of different subscriptions live on different streams, so completion
      // is not FIFO; scan the whole (bounded) list.
      if (reg.ops.query(it->token) == 1) {
        completed.push_back(std::move(*it));
        it = reg.pending.erase(it);
      } else {
        ++it;
      }
    }
    reg.pending_count.store(reg.pending.size(), std::memory_order_release);
  }

  release_all(completed);
}

void flush_deferred_subscriber_releases()
{
  Registry & reg = registry();

  std::vector<PendingRelease> remaining;
  {
    std::lock_guard<std::mutex> lock(reg.mutex);
    remaining.swap(reg.pending);
    reg.pending_count.store(0, std::memory_order_release);
    // Drop the hooks: nothing may call into libagnocast_cuda after a flush (see below).
    reg.ops = GpuDoneOps{};
  }

  // Deliberately NOT polling the markers here.
  //
  // This runs from Agnocast's static-destruction cleanup, and libagnocast_cuda's
  // GpuSharedMemoryPoolProxy is a function-local static created lazily on first use —
  // i.e. constructed *after* that cleanup object, therefore destroyed *before* it. So by
  // the time we get here the proxy, its mutexes and its marker free list are already
  // destroyed, and querying a marker through the hooks would be undefined behaviour.
  //
  // Releasing without waiting is safe at this point: any GPU work still reading these
  // buffers belongs to this exiting process, so nobody will consume its results, and the
  // CUDA context is torn down moments later. What matters is that the kernel-side
  // references are dropped so the publisher's pool slots go back to the daemon.
  if (!remaining.empty()) {
    RCLCPP_WARN(
      logger,
      "Agnocast GPU-IPC: releasing %zu message reference(s) at shutdown whose GPU reads had not "
      "reported completion.",
      remaining.size());
  }
  release_all(remaining);
}

size_t deferred_subscriber_release_count()
{
  return registry().pending_count.load(std::memory_order_acquire);
}

}  // namespace agnocast
