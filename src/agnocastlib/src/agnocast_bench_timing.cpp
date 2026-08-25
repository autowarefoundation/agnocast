// SPDX-License-Identifier: Apache-2.0
#include "agnocast/agnocast_bench_timing.hpp"

#ifdef AGNOCAST_BENCH_TIMING
#include <ctime>
#endif

namespace agnocast
{

#ifdef AGNOCAST_BENCH_TIMING

thread_local int64_t last_publish_ipc_ns = 0;
thread_local int64_t last_receive_ipc_ns = 0;

thread_local IpcBreakdown last_publish_ipc_breakdown = {};
thread_local IpcBreakdown last_receive_ipc_breakdown = {};

int64_t bench_timing_now_ns()
{
  struct timespec ts
  {
  };
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

int64_t get_last_publish_ipc_ns()
{
  return last_publish_ipc_ns;
}

int64_t get_last_receive_ipc_ns()
{
  return last_receive_ipc_ns;
}

const IpcBreakdown & get_last_publish_ipc_breakdown()
{
  return last_publish_ipc_breakdown;
}

const IpcBreakdown & get_last_receive_ipc_breakdown()
{
  return last_receive_ipc_breakdown;
}

#else

int64_t get_last_publish_ipc_ns()
{
  return 0;
}

int64_t get_last_receive_ipc_ns()
{
  return 0;
}

const IpcBreakdown & get_last_publish_ipc_breakdown()
{
  static const IpcBreakdown empty = {};
  return empty;
}

const IpcBreakdown & get_last_receive_ipc_breakdown()
{
  static const IpcBreakdown empty = {};
  return empty;
}

#endif

}  // namespace agnocast
