#!/usr/bin/env bash
set -euo pipefail
#
# Run one benchmark configuration against one backend.
#
#   scripts/run_bench.bash --backend kmod \
#     --num-topics 10 --num-subscribers 4 --rate-hz 100 \
#     --duration 10 --warmup 5 --iterations 5 --output-dir results/foo
#
# Launches one publisher process per topic and one subscriber process per
# (topic, subscriber) pair, releases them all from a file barrier so that every
# process starts measuring at the same instant, and writes per-process CSVs plus
# a per-iteration summary under <output-dir>/iter_<n>/.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.bash
source "${SCRIPT_DIR}/common.bash"

BACKEND=""
NUM_TOPICS=10
NUM_SUBSCRIBERS=4
RATE_HZ=100
DURATION=10
WARMUP=5 # lets RT threads reach steady state after barrier release
QOS_DEPTH=10
ITERATIONS=5
OUTPUT_DIR=""
TOPIC_PREFIX="/bench/topic"
USE_RT=true
PUB_KEEP_ALIVE=3

while [[ $# -gt 0 ]]; do
  case "$1" in
  --backend) BACKEND="$2"; shift 2 ;;
  --num-topics) NUM_TOPICS="$2"; shift 2 ;;
  --num-subscribers) NUM_SUBSCRIBERS="$2"; shift 2 ;;
  --rate-hz) RATE_HZ="$2"; shift 2 ;;
  --duration) DURATION="$2"; shift 2 ;;
  --warmup) WARMUP="$2"; shift 2 ;;
  --qos-depth) QOS_DEPTH="$2"; shift 2 ;;
  --iterations) ITERATIONS="$2"; shift 2 ;;
  --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
  --topic-prefix) TOPIC_PREFIX="$2"; shift 2 ;;
  --no-rt) USE_RT=false; shift ;;
  -h | --help) sed -n '3,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) die "unknown option: $1" ;;
  esac
done

[[ -n "${BACKEND}" ]] || die "--backend kmod|daemon is required"
validate_backend "${BACKEND}"
OUTPUT_DIR="${OUTPUT_DIR:-${BENCH_ROOT}/results/${BACKEND}}"

source_backend_ws "${BACKEND}"
BENCH_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"

PUB_BIN="$(pub_bin "${BACKEND}")"
SUB_BIN="$(sub_bin "${BACKEND}")"
HEAPHOOK_LIB="$(heaphook_lib "${BACKEND}")"
NUM_SUB_PROCS=$((NUM_TOPICS * NUM_SUBSCRIBERS))

# --- Privileges --------------------------------------------------------------
#
# RT mode needs root (chrt, sysctl); --no-rt mode is designed to need nothing,
# so a smoke test works on a machine without passwordless sudo. SUDO is empty in
# that case and the privileged steps below degrade to warnings.

SUDO=""
if [[ "${USE_RT}" == "true" || $EUID -eq 0 ]]; then
  SUDO="sudo"
  sudo -n true 2>/dev/null || {
    echo "This run needs root (RT scheduling, sysctl, cleanup). Prompting for sudo..."
    sudo -v || die "sudo is required; re-run with --no-rt to measure without RT scheduling"
  }
fi

# --- RT scheduling -----------------------------------------------------------
#
# Without SCHED_FIFO, CFS scheduling noise dominates tail latency at high topic
# counts and hides the backend's own scalability. chrt is applied at launch so
# the binary inherits the policy; the in-binary self-promotion after the barrier
# then becomes a no-op.

RT_PREFIX=""
RT_ENV=""
ORIGINAL_RT_RUNTIME=""
if [[ "${USE_RT}" == "true" ]]; then
  RT_PREFIX="sudo chrt -f 80"
  RT_ENV="AGNOCAST_BENCH_RT_PRIORITY=80"
  DAEMON_LAUNCH_PREFIX="sudo"

  # Linux reserves 5% of each second for non-RT tasks by default. That shows up
  # as a ~50 ms spike on every tail-latency plot — an artifact of kernel policy
  # rather than of either backend. 999500us/1000000us leaves 500us of relief,
  # enough for the orchestration shell without distorting the measurement.
  ORIGINAL_RT_RUNTIME=$(cat /proc/sys/kernel/sched_rt_runtime_us)
  if [[ "${ORIGINAL_RT_RUNTIME}" -ne 999500 ]]; then
    sudo sysctl -w kernel.sched_rt_runtime_us=999500 >/dev/null
  fi
fi

cleanup() {
  stop_daemon
  if [[ -n "${ORIGINAL_RT_RUNTIME}" ]]; then
    sudo sysctl -w kernel.sched_rt_runtime_us="${ORIGINAL_RT_RUNTIME}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

# --- System limits -----------------------------------------------------------
#
# Each subscriber process owns one POSIX message queue for receive notification.
# The bench nodes disable parameter services, which would otherwise add six more
# queues per process. The margin covers transient queues during teardown and any
# other Agnocast process on the machine.

REQUIRED_QUEUES=$((NUM_TOPICS * NUM_SUBSCRIBERS * 2 + NUM_TOPICS + 128))
CURRENT_QUEUES_MAX=$(cat /proc/sys/fs/mqueue/queues_max)
if [[ "${REQUIRED_QUEUES}" -gt "${CURRENT_QUEUES_MAX}" ]]; then
  if [[ -n "${SUDO}" ]]; then
    echo "Raising fs.mqueue.queues_max: ${CURRENT_QUEUES_MAX} -> ${REQUIRED_QUEUES}"
    ${SUDO} sysctl -w fs.mqueue.queues_max="${REQUIRED_QUEUES}" >/dev/null
  else
    die "fs.mqueue.queues_max is ${CURRENT_QUEUES_MAX}, this configuration needs ${REQUIRED_QUEUES}.
       Raise it first: sudo bash scripts/prep_repro_env.sh"
  fi
fi

echo "=== Agnocast benchmark: ${BACKEND} ($(backend_label "${BACKEND}")) ==="
echo "  Topics:      ${NUM_TOPICS}"
echo "  Subscribers: ${NUM_SUBSCRIBERS} per topic  (${NUM_SUB_PROCS} subscriber processes)"
echo "  Rate:        ${RATE_HZ} Hz"
echo "  Duration:    ${WARMUP}s warmup + ${DURATION}s measurement + ${PUB_KEEP_ALIVE}s keep-alive"
echo "  Iterations:  ${ITERATIONS}"
echo "  QoS depth:   ${QOS_DEPTH}"
echo "  RT sched:    ${USE_RT}"
echo "  Output:      ${OUTPUT_DIR}"
echo ""

# --- Per-iteration state cleanup ---------------------------------------------

wait_for_clean_state() {
  # Kernel-module mode relies on a poll_for_unlink daemon (forked from the first
  # bench process) to release process_info entries and their mempool slots.
  # Killing it outright makes stale entries accumulate until the mempool is
  # exhausted, so wait for it to drain instead of using pkill as the first move.
  for _ in $(seq 1 60); do
    ${SUDO} pgrep -f '[b]ench_publisher|[b]ench_subscriber' >/dev/null 2>&1 || break
    sleep 0.5
  done
  # Only per-process entries ('agnocast@<pid>') are expected to drain.
  # /dev/shm/agnocast_type_registry is shared and long-lived, so matching a bare
  # 'agnocast*' here would never reach zero and would burn the full timeout on
  # every iteration.
  for _ in $(seq 1 60); do
    local shm mq
    shm=$(find /dev/shm -maxdepth 1 -name 'agnocast@*' 2>/dev/null | wc -l)
    mq=$(find /dev/mqueue -maxdepth 1 -name 'agnocast@*' 2>/dev/null | wc -l)
    [[ "${shm}" -eq 0 && "${mq}" -eq 0 ]] && break
    sleep 0.5
  done

  ${SUDO} pkill -9 -f '[b]ench_publisher|[b]ench_subscriber' 2>/dev/null || true
  sleep 1
  ${SUDO} rm -f /dev/shm/agnocast@* /dev/mqueue/agnocast@* \
    /dev/mqueue/agnocast_bridge_manager@* 2>/dev/null || true
}

# --- Main loop ---------------------------------------------------------------

for iter in $(seq 0 $((ITERATIONS - 1))); do
  echo "--- Iteration $((iter + 1))/${ITERATIONS} ---"

  ITER_DIR="${OUTPUT_DIR}/iter_${iter}"
  LOG_DIR="${ITER_DIR}/logs"
  SYNC_DIR="${ITER_DIR}/.sync"
  mkdir -p "${LOG_DIR}" "${SYNC_DIR}"

  wait_for_clean_state

  # A fresh daemon per iteration is the daemon-mode equivalent of the drained
  # kernel process table above.
  if [[ "${BACKEND}" == "daemon" ]]; then
    restart_daemon
  else
    require_backend kmod
  fi

  sync
  if [[ -n "${SUDO}" ]]; then
    ${SUDO} sh -c 'echo 3 > /proc/sys/vm/drop_caches' 2>/dev/null || true
  fi
  sleep 2

  BARRIER_START_NS=$(date +%s%N)

  # Logs go to per-process files: thousands of processes writing to one PTY
  # would block on a full terminal buffer and stall RT threads.
  PUB_PIDS=()
  for topic_idx in $(seq 0 $((NUM_TOPICS - 1))); do
    # shellcheck disable=SC2086
    ${RT_PREFIX} env LD_LIBRARY_PATH="${BENCH_LD_LIBRARY_PATH}" LD_PRELOAD="${HEAPHOOK_LIB}" AGNOCAST_BRIDGE_MODE=off \
      AGNOCAST_NO_DISCOVERY_AGENT=1 ${RT_ENV} \
      "${PUB_BIN}" --ros-args \
      -p topic_prefix:="${TOPIC_PREFIX}" \
      -p topic_index:="${topic_idx}" \
      -p rate_hz:="${RATE_HZ}.0" \
      -p duration_sec:="${DURATION}.0" \
      -p warmup_sec:="${WARMUP}.0" \
      -p keep_alive_sec:="${PUB_KEEP_ALIVE}.0" \
      -p qos_depth:="${QOS_DEPTH}" \
      -p output_dir:="${ITER_DIR}" \
      -p sync_dir:="${SYNC_DIR}" \
      >"${LOG_DIR}/pub_${topic_idx}.log" 2>&1 &
    PUB_PIDS+=($!)
  done

  echo "  Waiting for ${NUM_TOPICS} publishers..."
  for _ in $(seq 1 6000); do
    [[ "$(find "${SYNC_DIR}" -name 'pub_*.ready' | wc -l)" -ge "${NUM_TOPICS}" ]] && break
    sleep 0.05
  done
  ready=$(find "${SYNC_DIR}" -name 'pub_*.ready' | wc -l)
  [[ "${ready}" -ge "${NUM_TOPICS}" ]] ||
    die "only ${ready}/${NUM_TOPICS} publishers became ready (see ${LOG_DIR})"

  # Subscribers start only after every publisher has registered its topic.
  SUB_PIDS=()
  for sub_idx in $(seq 0 $((NUM_SUBSCRIBERS - 1))); do
    for topic_idx in $(seq 0 $((NUM_TOPICS - 1))); do
      # shellcheck disable=SC2086
      ${RT_PREFIX} env LD_LIBRARY_PATH="${BENCH_LD_LIBRARY_PATH}" LD_PRELOAD="${HEAPHOOK_LIB}" AGNOCAST_BRIDGE_MODE=off \
      AGNOCAST_NO_DISCOVERY_AGENT=1 ${RT_ENV} \
        "${SUB_BIN}" --ros-args \
        -p topic_prefix:="${TOPIC_PREFIX}" \
        -p topic_index:="${topic_idx}" \
        -p subscriber_index:="${sub_idx}" \
        -p rate_hz:="${RATE_HZ}.0" \
        -p duration_sec:="${DURATION}.0" \
        -p warmup_sec:="${WARMUP}.0" \
        -p qos_depth:="${QOS_DEPTH}" \
        -p output_dir:="${ITER_DIR}" \
        -p sync_dir:="${SYNC_DIR}" \
        >"${LOG_DIR}/sub_${sub_idx}_topic_${topic_idx}.log" 2>&1 &
      SUB_PIDS+=($!)
    done
  done

  echo "  Waiting for ${NUM_SUB_PROCS} subscribers..."
  for _ in $(seq 1 6000); do
    [[ "$(find "${SYNC_DIR}" -name 'sub_*.ready' | wc -l)" -ge "${NUM_SUB_PROCS}" ]] && break
    sleep 0.05
  done
  ready=$(find "${SYNC_DIR}" -name 'sub_*.ready' | wc -l)
  [[ "${ready}" -ge "${NUM_SUB_PROCS}" ]] ||
    die "only ${ready}/${NUM_SUB_PROCS} subscribers became ready (see ${LOG_DIR})"

  touch "${SYNC_DIR}/START"
  echo "  Barrier released after $((($(date +%s%N) - BARRIER_START_NS) / 1000000)) ms."

  for pid in "${PUB_PIDS[@]}" "${SUB_PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done

  python3 "${SCRIPT_DIR}/summarize.py" --iter-dir "${ITER_DIR}" --iteration "${iter}" \
    --duration "${DURATION}" --backend "${BACKEND}"
  echo "  Summary: ${ITER_DIR}/summary.csv"
done

echo ""
echo "=== Done. Results in ${OUTPUT_DIR} ==="
