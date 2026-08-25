// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace agnocast
{

// Duration of the most recent publish/receive metadata round trip issued by the
// calling thread. The instrumentation sits at the `agnocast_ipc_*` call site,
// which is the single point where the two backends diverge, so the kernel-module
// number (an ioctl on /dev/agnocast) and the user-daemon number (a Unix socket
// round trip) measure the same logical operation and are directly comparable.
//
// Both getters return 0 unless agnocastlib was built with
// -DAGNOCAST_BENCH_TIMING=ON. They are always declared so that benchmark code
// compiles against either build.
int64_t get_last_publish_ipc_ns();
int64_t get_last_receive_ipc_ns();

// Non-overlapping segments of one metadata round trip, in nanoseconds.
//
// req + up + lock + work + down + post == total, so the segments can be stacked
// directly. `send_syscall` is a nested measurement inside `up` and is reported
// separately rather than added.
//
// Only the user-daemon backend can fill this in: the split relies on stamps
// taken inside the daemon and carried back in the response trailer. Under the
// kernel module the metadata work runs in the caller's own context with no
// second scheduling entity, so there is nothing to attribute from userspace and
// every field stays 0 (use ftrace or bpftrace to look inside the ioctl).
struct IpcBreakdown
{
  int64_t total_ns;         // whole agnocast_ipc_* call
  int64_t req_ns;           // marshal the request and zero the response struct
  int64_t send_syscall_ns;  // sendmsg() itself; nested inside up_ns
  int64_t up_ns;            // sendmsg .. daemon's recv() returning (transport + wakeup)
  int64_t lock_ns;          // daemon: dispatch, topic lookup, lock acquisition
  int64_t work_ns;          // daemon: handler body and response fill
  int64_t down_ns;          // daemon's reply .. client's recvmsg returning
  int64_t post_ns;          // unmarshal the response into the caller's args
  // 1 when the daemon supplied its stamps. When 0, up_ns holds the entire
  // sendmsg-to-recvmsg span and lock/work/down are 0, so the segments still sum
  // to total_ns but the daemon-internal split is unavailable.
  int64_t daemon_stamped;
};

const IpcBreakdown & get_last_publish_ipc_breakdown();
const IpcBreakdown & get_last_receive_ipc_breakdown();

#ifdef AGNOCAST_BENCH_TIMING

extern thread_local int64_t last_publish_ipc_ns;
extern thread_local int64_t last_receive_ipc_ns;

extern thread_local IpcBreakdown last_publish_ipc_breakdown;
extern thread_local IpcBreakdown last_receive_ipc_breakdown;

int64_t bench_timing_now_ns();

class BenchTimingScope
{
  int64_t * dst_;
  int64_t start_ns_;

public:
  explicit BenchTimingScope(int64_t * dst) : dst_(dst), start_ns_(bench_timing_now_ns()) {}
  ~BenchTimingScope() { *dst_ = bench_timing_now_ns() - start_ns_; }

  BenchTimingScope(const BenchTimingScope &) = delete;
  BenchTimingScope & operator=(const BenchTimingScope &) = delete;
  BenchTimingScope(BenchTimingScope &&) = delete;
  BenchTimingScope & operator=(BenchTimingScope &&) = delete;
};

#endif  // AGNOCAST_BENCH_TIMING

}  // namespace agnocast

#ifdef AGNOCAST_BENCH_TIMING
#define AGNOCAST_BENCH_TIME_PUBLISH_IPC()                         \
  const ::agnocast::BenchTimingScope agnocast_bench_timing_scope_ \
  {                                                               \
    &::agnocast::last_publish_ipc_ns                              \
  }
#define AGNOCAST_BENCH_TIME_RECEIVE_IPC()                         \
  const ::agnocast::BenchTimingScope agnocast_bench_timing_scope_ \
  {                                                               \
    &::agnocast::last_receive_ipc_ns                              \
  }
#else
#define AGNOCAST_BENCH_TIME_PUBLISH_IPC() static_cast<void>(0)
#define AGNOCAST_BENCH_TIME_RECEIVE_IPC() static_cast<void>(0)
#endif
