# M0 latency benchmark (kmod vs user daemon)

Compares Agnocast metadata round-trip latency under **M0** (daemon runs with default `SCHED_OTHER`, no RT knobs).

## Prerequisites

- ROS 2 Humble sourced
- For **kmod** runs: `sudo insmod agnocast_kmod/agnocast.ko`
- `pip install --user -r scripts/bench/requirements.txt` (plotting; needs `numpy<2` on Ubuntu 22.04)

## Build

Builds isolated colcon trees under `ws/kmod/` and `ws/daemon/` (compile-time backend switch):

```bash
source /opt/ros/humble/setup.bash
scripts/bench/build.bash              # both backends
scripts/bench/build.bash kmod         # kernel module client only
scripts/bench/build.bash daemon       # user daemon client only
```

Daemon backend also requires the metadata daemon:

```bash
scripts/run_daemon.bash               # foreground, or let run_bench start it
```

## Run

Single configuration smoke test:

```bash
scripts/bench/run_bench.bash --backend daemon --topics 1 --subscribers 2 --rate 100
```

Full sweeps (ipc_shared_ptr paper §VII-style):

```bash
sudo scripts/bench/prep_repro_env.sh   # RT bandwidth, governor, MQ limits
scripts/bench/compare.bash --iterations 5
```

Set `AGNOCAST_NO_DISCOVERY_AGENT=1` (handled by runner scripts) to avoid fork storms during multi-process sweeps.

## Metrics

| Metric | Meaning |
|--------|---------|
| `publish_ipc` | `agnocast_ipc_publish_msg` round trip (ioctl vs UDS) |
| `receive_ipc` | `agnocast_ipc_receive_msg` round trip |
| `publish` | `borrow_loaned_message()` through `publish()` |
| `e2e` | publisher stamp to subscriber callback entry |

## IPC breakdown

The two `*_ipc` metrics are also split into non-overlapping segments that sum to
the round trip, so a slow backend can be attributed to scheduling rather than to
work. Both `summary.csv` (rows named `<metric>_<segment>`) and the raw
`ipc_*_ns` columns carry them.

| Segment | Covers |
|---------|--------|
| `req` | client marshals the request and zeroes the response struct |
| `up` | `sendmsg` until the daemon's `recv` returns: transport plus wakeup |
| `lock` | daemon dispatch, topic lookup and lock acquisition |
| `work` | daemon handler body and response fill |
| `down` | daemon's reply until the client's `recvmsg` returns |
| `post` | client unmarshals the response into the caller's args |
| `send_syscall` | `sendmsg` alone; nested inside `up`, never summed |

The split needs stamps taken inside the daemon and returned in a response
trailer, so `agnocast_daemon` must also be built with `AGNOCAST_BENCH_TIMING`
(`build.bash` does this). Under the kernel module the metadata work runs in the
caller's own context with no second scheduling entity, so the segments are all
zero and only the round trip is reported; use ftrace or bpftrace to look inside
the ioctl.

```bash
scripts/bench/breakdown.py results/kmod results/daemon --legend
```
