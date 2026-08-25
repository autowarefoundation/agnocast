#!/usr/bin/env bash
set -euo pipefail
#
# Run the full sweep set against one backend.
#
#   scripts/run_sweeps.bash --backend kmod --output-dir results/kmod --iterations 5
#   scripts/run_sweeps.bash --backend daemon --sweeps a,b
#
# Sweeps:
#   A  num_topics        (subscribers=2, 100 Hz)
#   B  num_subscribers   (topics=10, 100 Hz)
#   C  rate_hz           (topics=10, subscribers=4)
#   D  topics x subscribers grid (100 Hz)
#
# Configurations whose event rate exceeds EVENT_BUDGET are skipped, so a sweep
# never reports numbers that are really just a saturated machine. The event rate
# of a configuration is num_topics * (1 + num_subscribers) * (rate_hz + 10); the
# +10 accounts for the 100 ms control timer in every process.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.bash
source "${SCRIPT_DIR}/common.bash"

BACKEND=""
OUTPUT_ROOT=""
ITERATIONS=5
DURATION=10
WARMUP=5
QOS_DEPTH=10
SWEEPS="a,b,c,d"
EXTRA_ARGS=()

# Empirical capacity of this host, derated to 60%. The reference 8C/16T Xeon
# sustained ~140K events/s; this 8C/8T i7-9700K has half the hardware threads,
# so 70K is the working estimate and 42K the budget. Override for other hosts.
EVENT_BUDGET="${EVENT_BUDGET:-42000}"

while [[ $# -gt 0 ]]; do
  case "$1" in
  --backend) BACKEND="$2"; shift 2 ;;
  --output-dir) OUTPUT_ROOT="$2"; shift 2 ;;
  --iterations) ITERATIONS="$2"; shift 2 ;;
  --duration) DURATION="$2"; shift 2 ;;
  --warmup) WARMUP="$2"; shift 2 ;;
  --qos-depth) QOS_DEPTH="$2"; shift 2 ;;
  --sweeps) SWEEPS="$2"; shift 2 ;;
  --event-budget) EVENT_BUDGET="$2"; shift 2 ;;
  --no-rt) EXTRA_ARGS+=(--no-rt); shift ;;
  -h | --help) sed -n '3,19p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) die "unknown option: $1" ;;
  esac
done

[[ -n "${BACKEND}" ]] || die "--backend kmod|daemon is required"
validate_backend "${BACKEND}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${BENCH_ROOT}/results/${BACKEND}}"

wants_sweep() { [[ ",${SWEEPS}," == *",$1,"* ]]; }

within_budget() {
  local nt="$1" ns="$2" rate="$3"
  [[ $((nt * (1 + ns) * (rate + 10))) -le "${EVENT_BUDGET}" ]]
}

run_one() {
  local nt="$1" ns="$2" rate="$3" out="$4"
  local events=$((nt * (1 + ns) * (rate + 10)))

  if ! within_budget "${nt}" "${ns}" "${rate}"; then
    echo "--- SKIP topics=${nt} subs=${ns} rate=${rate} (${events} > ${EVENT_BUDGET} events/s) ---"
    return 0
  fi

  echo "--- topics=${nt} subs=${ns} rate=${rate} (${events} events/s) ---"
  bash "${SCRIPT_DIR}/run_bench.bash" \
    --backend "${BACKEND}" \
    --num-topics "${nt}" \
    --num-subscribers "${ns}" \
    --rate-hz "${rate}" \
    --iterations "${ITERATIONS}" \
    --duration "${DURATION}" \
    --warmup "${WARMUP}" \
    --qos-depth "${QOS_DEPTH}" \
    --output-dir "${out}" \
    ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
}

echo "========================================================"
echo " Sweeps for backend: ${BACKEND} ($(backend_label "${BACKEND}"))"
echo " Output:       ${OUTPUT_ROOT}"
echo " Sweeps:       ${SWEEPS}"
echo " Event budget: ${EVENT_BUDGET} events/s"
echo "========================================================"

# Sweep A uses 2 subscribers rather than 4 so that topic-count scaling is not
# confounded by subscriber fan-out; fan-out is what sweep B measures.
if wants_sweep a; then
  echo "==== Sweep A: num_topics ===="
  for nt in 1 25 50 75 100 125 150; do
    run_one "${nt}" 2 100 "${OUTPUT_ROOT}/sweep_a/num_topics_${nt}"
  done
fi

if wants_sweep b; then
  echo "==== Sweep B: num_subscribers ===="
  for ns in 1 4 8 12 16 20 24 28 32; do
    run_one 10 "${ns}" 100 "${OUTPUT_ROOT}/sweep_b/num_subscribers_${ns}"
  done
fi

if wants_sweep c; then
  echo "==== Sweep C: rate_hz ===="
  for rate in 10 50 100 200 500 1000; do
    run_one 10 4 "${rate}" "${OUTPUT_ROOT}/sweep_c/rate_hz_${rate}"
  done
fi

if wants_sweep d; then
  echo "==== Sweep D: num_topics x num_subscribers ===="
  for nt in 10 20 40 60 80; do
    for ns in 2 4 8 12 16; do
      run_one "${nt}" "${ns}" 100 "${OUTPUT_ROOT}/sweep_d/nt_${nt}_ns_${ns}"
    done
  done
fi

echo ""
echo "=== All sweeps complete. Results in ${OUTPUT_ROOT} ==="
