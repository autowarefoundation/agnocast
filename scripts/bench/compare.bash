#!/usr/bin/env bash
set -euo pipefail
#
# Run the same sweeps against both backends and plot the comparison.
#
#   scripts/compare.bash                        # full sweeps, 5 iterations
#   scripts/compare.bash --sweeps a,b --iterations 3
#
# Backends alternate per sweep set rather than running one after the other in a
# single block, so that slow thermal or background drift shows up as noise in
# both series instead of as a systematic advantage for whichever ran first.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.bash
source "${SCRIPT_DIR}/common.bash"

OUTPUT_ROOT="${BENCH_ROOT}/results/compare_$(date +%Y%m%d_%H%M%S)"
SWEEP_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
  --output-dir) OUTPUT_ROOT="$2"; shift 2 ;;
  -h | --help) sed -n '3,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) SWEEP_ARGS+=("$1"); shift ;;
  esac
done

for backend in kmod daemon; do
  echo ""
  echo "############################################################"
  echo "# Backend: ${backend} ($(backend_label "${backend}"))"
  echo "############################################################"
  bash "${SCRIPT_DIR}/run_sweeps.bash" \
    --backend "${backend}" \
    --output-dir "${OUTPUT_ROOT}/${backend}" \
    ${SWEEP_ARGS[@]+"${SWEEP_ARGS[@]}"}
done

echo ""
echo "==> Generating figures"
python3 "${SCRIPT_DIR}/plot_results.py" \
  --data "${OUTPUT_ROOT}/kmod" --label "kernel module" --color "#1f77b4" \
  --data "${OUTPUT_ROOT}/daemon" --label "user daemon" --color "#d62728" \
  --output-dir "${OUTPUT_ROOT}/figures"

echo ""
echo "=== Comparison complete ==="
echo "  Raw data: ${OUTPUT_ROOT}"
echo "  Figures:  ${OUTPUT_ROOT}/figures"
