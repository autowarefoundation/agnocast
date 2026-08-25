#!/usr/bin/env bash
set -euo pipefail
#
# Lock the machine into a reproducible state, or restore it afterwards.
#
#   sudo bash scripts/prep_repro_env.sh            # apply
#   sudo bash scripts/prep_repro_env.sh --restore  # undo
#   bash scripts/prep_repro_env.sh --status        # inspect
#
# What it controls:
#   - Turbo Boost off and the frequency pinned to base, so a run's latency does
#     not depend on how warm the package happens to be.
#   - C-states limited to C0/C1 by holding /dev/cpu_dma_latency at 0, which
#     removes wakeup-latency variation from the idle path.
#   - Page cache dropped, so every fresh apply starts from the same state.
#   - POSIX message queue limits raised: Agnocast uses one mqueue per
#     (topic, subscriber) for receive notification, and the stock msg_max of 10
#     is far below what the larger sweep configurations need.
#
# Deliberately untouched: SMT (disabling it would over-subscribe the high-topic
# configurations) and process affinity (handled by the runner).
#
# Prior state is saved to /tmp/agnocast_bench_env_state.json so --restore is safe.

STATE_FILE=/tmp/agnocast_bench_env_state.json
DMA_PID_FILE=/tmp/agnocast_bench_dma_holder.pid
DMA_HOLDER=/tmp/agnocast_bench_dma_holder.py
DMA_LOG=/tmp/agnocast_bench_dma_holder.log

NO_TURBO_FILE=/sys/devices/system/cpu/intel_pstate/no_turbo
BASE_FREQ_FILE=/sys/devices/system/cpu/cpu0/cpufreq/base_frequency

TARGET_MSG_MAX=256
TARGET_QUEUES_MAX=8192

require_root() {
  [[ $EUID -eq 0 ]] || {
    echo "ERROR: this mode requires root. Re-run with sudo." >&2
    exit 1
  }
}

mode=${1:-apply}

# --- status ------------------------------------------------------------------

if [[ "${mode}" == "--status" ]]; then
  echo "no_turbo:        $(cat ${NO_TURBO_FILE} 2>/dev/null || echo n/a)  (1 = Turbo disabled)"
  echo "min_freq:        $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq) kHz"
  echo "max_freq:        $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq) kHz"
  echo "base_freq:       $(cat ${BASE_FREQ_FILE} 2>/dev/null || echo n/a) kHz"
  echo "governor:        $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
  echo "mqueue msg_max:  $(cat /proc/sys/fs/mqueue/msg_max)"
  echo "mqueue queues:   $(cat /proc/sys/fs/mqueue/queues_max)"
  echo "sched_rt_runtime: $(cat /proc/sys/kernel/sched_rt_runtime_us) us"
  if [[ -f "${DMA_PID_FILE}" ]] && kill -0 "$(cat ${DMA_PID_FILE})" 2>/dev/null; then
    echo "cpu_dma_latency: held (PID $(cat ${DMA_PID_FILE}))"
  else
    echo "cpu_dma_latency: not held"
  fi
  echo "saved state:     ${STATE_FILE}$([[ -f ${STATE_FILE} ]] || echo ' <none>')"
  exit 0
fi

# --- restore -----------------------------------------------------------------

if [[ "${mode}" == "--restore" ]]; then
  require_root
  [[ -f "${STATE_FILE}" ]] || {
    echo "No state file at ${STATE_FILE}; nothing to restore."
    exit 0
  }

  read -r ORIG_NO_TURBO ORIG_MIN ORIG_MAX ORIG_MSG_MAX ORIG_QUEUES_MAX < <(
    python3 -c "
import json
d = json.load(open('${STATE_FILE}'))
print(d['no_turbo'], d['min_freq'], d['max_freq'], d['msg_max'], d['queues_max'])
"
  )

  [[ -f "${NO_TURBO_FILE}" ]] && echo "${ORIG_NO_TURBO}" >"${NO_TURBO_FILE}"

  # Raise max before lowering min so min <= max always holds.
  for cpufreq in /sys/devices/system/cpu/cpu*/cpufreq; do
    [[ -d "${cpufreq}" ]] || continue
    echo "${ORIG_MAX}" >"${cpufreq}/scaling_max_freq" 2>/dev/null || true
    echo "${ORIG_MIN}" >"${cpufreq}/scaling_min_freq" 2>/dev/null || true
  done

  sysctl -w fs.mqueue.msg_max="${ORIG_MSG_MAX}" >/dev/null
  sysctl -w fs.mqueue.queues_max="${ORIG_QUEUES_MAX}" >/dev/null

  if [[ -f "${DMA_PID_FILE}" ]]; then
    kill "$(cat ${DMA_PID_FILE})" 2>/dev/null || true
    rm -f "${DMA_PID_FILE}"
  fi
  rm -f "${STATE_FILE}" "${DMA_HOLDER}" "${DMA_LOG}"

  echo "Restored: no_turbo=${ORIG_NO_TURBO}, freq=[${ORIG_MIN},${ORIG_MAX}] kHz,"
  echo "          msg_max=${ORIG_MSG_MAX}, queues_max=${ORIG_QUEUES_MAX}"
  exit 0
fi

# --- apply -------------------------------------------------------------------

require_root

if [[ ! -f "${STATE_FILE}" ]]; then
  python3 - >"${STATE_FILE}" <<EOF
import json
print(json.dumps({
  "no_turbo":   "$(cat "${NO_TURBO_FILE}" 2>/dev/null || echo 0)",
  "min_freq":   "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq)",
  "max_freq":   "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq)",
  "msg_max":    "$(cat /proc/sys/fs/mqueue/msg_max)",
  "queues_max": "$(cat /proc/sys/fs/mqueue/queues_max)",
}, indent=2))
EOF
  echo "Saved prior state -> ${STATE_FILE}"
fi

if [[ -f "${NO_TURBO_FILE}" ]]; then
  echo 1 >"${NO_TURBO_FILE}"
  echo "  Turbo Boost     -> disabled"
else
  echo "  Turbo Boost     -> no intel_pstate sysfs; skipped"
fi

if [[ -f "${BASE_FREQ_FILE}" ]]; then
  TARGET_FREQ_KHZ=$(cat "${BASE_FREQ_FILE}")
else
  TARGET_FREQ_KHZ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq)
  echo "  WARNING: no base_frequency sysfs; pinning to cpuinfo_max_freq instead."
fi
for cpufreq in /sys/devices/system/cpu/cpu*/cpufreq; do
  [[ -d "${cpufreq}" ]] || continue
  echo "${TARGET_FREQ_KHZ}" >"${cpufreq}/scaling_max_freq"
  echo "${TARGET_FREQ_KHZ}" >"${cpufreq}/scaling_min_freq"
done
echo "  CPU frequency   -> pinned at ${TARGET_FREQ_KHZ} kHz"

# The kernel resets the DMA-latency limit once every writer closes its fd, so a
# process has to stay alive holding it open for the duration of the experiment.
cat >"${DMA_HOLDER}" <<'PYEOF'
import os, signal, struct, sys, time

fd = os.open('/dev/cpu_dma_latency', os.O_RDWR)
os.write(fd, struct.pack('i', 0))


def cleanup(*_):
    try:
        os.close(fd)
    finally:
        sys.exit(0)


signal.signal(signal.SIGTERM, cleanup)
signal.signal(signal.SIGINT, cleanup)
print("dma_latency_holder ready", flush=True)
while True:
    time.sleep(3600)
PYEOF

if [[ ! -f "${DMA_PID_FILE}" ]] || ! kill -0 "$(cat ${DMA_PID_FILE} 2>/dev/null)" 2>/dev/null; then
  nohup python3 "${DMA_HOLDER}" >"${DMA_LOG}" 2>&1 &
  echo $! >"${DMA_PID_FILE}"
  for _ in $(seq 1 20); do
    grep -q ready "${DMA_LOG}" 2>/dev/null && break
    sleep 0.1
  done
  echo "  cpu_dma_latency -> 0 (held by PID $(cat ${DMA_PID_FILE}); C-state <= C1)"
else
  echo "  cpu_dma_latency -> already held by PID $(cat ${DMA_PID_FILE})"
fi

sysctl -w fs.mqueue.msg_max="${TARGET_MSG_MAX}" >/dev/null
sysctl -w fs.mqueue.queues_max="${TARGET_QUEUES_MAX}" >/dev/null
echo "  mqueue limits   -> msg_max=${TARGET_MSG_MAX}, queues_max=${TARGET_QUEUES_MAX}"

sync
echo 3 >/proc/sys/vm/drop_caches
echo "  Page cache      -> dropped"

for noisy in chrome firefox code; do
  if pgrep -f "${noisy}" >/dev/null; then
    echo "  WARNING: ${noisy} is running ($(pgrep -fc "${noisy}") procs); it will widen the tail."
  fi
done

echo "Environment locked. Restore with: sudo bash $0 --restore"
