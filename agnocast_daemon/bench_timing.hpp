// SPDX-License-Identifier: Apache-2.0
#pragma once

// Daemon-side stamps for the benchmark timing trailer.
//
// One connection is served by one dedicated thread, so per-thread storage holds
// exactly the stamps of the request currently being served. The stamps are read
// back in send_response(), which is the last thing every handler does.
//
// Everything here compiles away unless the daemon is built with
// -DAGNOCAST_BENCH_TIMING.

#include "protocol.h"

#ifdef AGNOCAST_BENCH_TIMING

#include <cstdint>
#include <ctime>

namespace agnocast_daemon_bench
{

inline int64_t now_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

inline thread_local int64_t t_recv_ns = 0;
inline thread_local int64_t t_work_ns = 0;

inline void fill_trailer(BenchTimingTrailer & trailer)
{
  trailer.magic = AGNOCAST_BENCH_TRAILER_MAGIC;
  trailer._pad = 0;
  trailer.recv_ns = t_recv_ns;
  trailer.work_ns = t_work_ns;
  trailer.send_ns = now_ns();
}

}  // namespace agnocast_daemon_bench

// Stamped right after recv() returns: marks when this thread was actually
// scheduled with the client's request in hand.
#define AGNOCAST_DAEMON_BENCH_STAMP_RECV() \
  (::agnocast_daemon_bench::t_recv_ns = ::agnocast_daemon_bench::now_ns())

// Stamped by a handler once it is past dispatch and holds its locks, so that
// lock acquisition is separated from the metadata work itself.
#define AGNOCAST_DAEMON_BENCH_STAMP_WORK() \
  (::agnocast_daemon_bench::t_work_ns = ::agnocast_daemon_bench::now_ns())

// Handlers that do not stamp their work start report 0, which the client reads
// as "not decomposed"; clear it so a previous request cannot leak into this one.
#define AGNOCAST_DAEMON_BENCH_CLEAR_WORK() (::agnocast_daemon_bench::t_work_ns = 0)

#else  // AGNOCAST_BENCH_TIMING

#define AGNOCAST_DAEMON_BENCH_STAMP_RECV() static_cast<void>(0)
#define AGNOCAST_DAEMON_BENCH_STAMP_WORK() static_cast<void>(0)
#define AGNOCAST_DAEMON_BENCH_CLEAR_WORK() static_cast<void>(0)

#endif  // AGNOCAST_BENCH_TIMING
