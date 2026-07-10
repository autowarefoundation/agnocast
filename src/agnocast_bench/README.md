# agnocast_bench — publish-notification latency probe (eventfd vs MQ)

Local benchmarking tool used to measure the latency improvement of the
**eventfd publish-notification** change ([#1432]) versus the POSIX-MQ baseline.
Not for merge — it is a measurement harness.

## What it measures

A single publisher and *N* subscribers on one topic (`StaticSizeArray`,
`id` + `timestamp` + 1 KB payload). Two fair metrics:

- **publish latency** — publisher wall-clock around `publish()`. This is exactly
  where the mechanisms differ: MQ does one userspace `mq_send` **per subscriber**
  (N syscalls); eventfd signals all subscribers with **one** in-ioctl call.
- **e2e latency** — publisher `timestamp` (stamped just before `publish()`) to
  subscriber callback entry. Both sides stamp `CLOCK_MONOTONIC` on the same host,
  so `e2e = recv - timestamp` needs no clock sync. Warmup messages are skipped by
  `id`, so no barrier is needed.

The subscriber count is swept (fan-out): the eventfd win grows with N (N→1
syscalls) and converges around N ≈ number of logical CPUs, where the scheduler's
O(N) wakeup cost dominates.

## Why not the original `isorc26_tmp` repo

The previous measurements ([#1272]) used the `isorc26_tmp` benchmark nodes, but
those **no longer run on current Agnocast**:

- They use the **old client API** (`agnocast::init` / `agnocast::Node` /
  `AgnocastOnlySingleThreadedExecutor`). Against current `agnocastlib` this
  registers the process **twice**, so the kmod's `add_process` rejects the second
  call with `-EINVAL` ("process already exists"); when timing lets it through, the
  publish path **segfaults**. Each crash also leaks kmod per-process state, so
  runs poison each other until the module is reloaded.
- The harness also assumes a flat workspace layout (`~/install`), which no longer
  matches where `agnocastlib` builds.

Rather than resurrect the stale harness, this probe is rebuilt on the **current**
API (`rclcpp::Node` + `SingleThreadedAgnocastExecutor`), mirroring the maintained
sample apps in `agnocast_sample_application` — which do run cleanly on current
Agnocast. It is self-contained (one small package) so it can't inherit the old
harness's layout/state issues.

## Fairness

The A/B compares two branches that share base **#1426** and differ **only** by the
notification mechanism:

- `bench/base` — MQ baseline (main-snapshot + timing instrumentation).
- `feat/eventfd-publish-notification-v2` — eventfd (#1432) + the **byte-identical**
  instrumentation.

`run_ab.sh` rebuilds this probe against each branch's `agnocastlib`, loads the
matching kmod, and sweeps N. Same rate/QoS/warmup/measurement, same host,
back-to-back, no RT scheduling (identical on both sides).

## How to run

```bash
bash src/agnocast_bench/run_ab.sh            # builds each branch, loads kmod, sweeps N
python3 src/agnocast_bench/analyze.py ~/eventfd_bench_<timestamp>   # p50/p99 table + Δ
```

[#1272]: https://github.com/autowarefoundation/agnocast/pull/1272
[#1432]: https://github.com/autowarefoundation/agnocast/pull/1432
