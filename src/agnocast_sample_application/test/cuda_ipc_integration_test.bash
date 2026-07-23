#!/usr/bin/env bash
# =============================================================================
# End-to-end integration test for the Agnocast GPU-IPC (CUDA) zero-copy path.
#
# Exercises the full pipeline on a real GPU:
#
#     gpu_shared_memory_daemon   (pre-allocates a pool of GPU slots, serves them
#                                 over a Unix socket derived from the GPU UUID)
#             |            |
#        cuda_talker   cuda_listener
#     (publisher)      (subscriber)
#
# with BOTH heaphooks LD_PRELOADed:
#   * libagnocast_heaphook.so       - routes CPU malloc into Agnocast shared memory
#   * libagnocast_cuda_heaphook.so  - routes the publisher's cudaMalloc into the pool
#
# The publisher fills each GPU buffer with an incrementing byte pattern
#   data[i] = (i + seq) % 256
# so the first four bytes of message `seq` are [seq, seq+1, seq+2, seq+3] (mod 256).
# The subscriber cudaMemcpy's the first bytes back to the host and logs them. This
# test PASSES only if the subscriber observes that exact increasing pattern on
# several messages, which proves the GPU bytes the publisher wrote were read,
# zero-copy, in another process through the pool + CUDA IPC + interprocess event.
#
# Requirements (fails fast with a clear message if unmet):
#   * an NVIDIA GPU + CUDA runtime (libcudart)
#   * the agnocast kernel module loaded (/dev/agnocast present)
#   * a writable socket dir /run/agnocast  (one-time setup, needs root once):
#         sudo mkdir -p /run/agnocast && sudo chown "$(id -u):$(id -g)" /run/agnocast
#
# This test is NOT part of the colcon/CI test suite (the build farm has no GPU);
# run it manually on a GPU machine after `colcon build`.
#
# Exit code: 0 = PASS, non-zero = FAIL/setup-error.
# =============================================================================
set -uo pipefail

# --- Tunables -----------------------------------------------------------------
RUN_SECONDS="${RUN_SECONDS:-5}"       # how long to let pub/sub exchange messages
MIN_RECEIVED="${MIN_RECEIVED:-5}"     # minimum verified messages required to pass
GPU_INDEX="${CUDA_VISIBLE_DEVICES:-0}"

# --- Locate the workspace and package sources --------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# test/ -> agnocast_sample_application/ -> src/ -> <agnocast package root>
AGNOCAST_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Walk up to find the colcon workspace (the dir containing install/setup.bash).
WS_ROOT="${AGNOCAST_ROOT}"
while [[ "${WS_ROOT}" != "/" && ! -f "${WS_ROOT}/install/setup.bash" ]]; do
  WS_ROOT="$(dirname "${WS_ROOT}")"
done

log()  { echo "[cuda-ipc-it] $*"; }
fail() { echo "[cuda-ipc-it] FAIL: $*" >&2; exit 1; }

# --- Preflight ----------------------------------------------------------------
[[ -f "${WS_ROOT}/install/setup.bash" ]] || fail "could not find install/setup.bash above ${AGNOCAST_ROOT} (run colcon build first)"
[[ -c /dev/agnocast ]] || fail "/dev/agnocast not found - load the agnocast kernel module first"
command -v nvidia-smi >/dev/null 2>&1 || fail "nvidia-smi not found - this test needs an NVIDIA GPU"

# Both heaphooks are cargo artifacts (not colcon-installed); reference them by
# absolute path so the test does not depend on them being staged into an install
# tree or on LD_LIBRARY_PATH soname resolution.
CPU_HEAPHOOK="${AGNOCAST_ROOT}/agnocast_heaphook/target/release/libagnocast_heaphook.so"
[[ -f "${CPU_HEAPHOOK}" ]] || fail "CPU heaphook not built: ${CPU_HEAPHOOK}
  build it with:  (cd ${AGNOCAST_ROOT}/agnocast_heaphook && cargo build --release)"

CUDA_HEAPHOOK="${AGNOCAST_ROOT}/agnocast_cuda_heaphook/target/release/libagnocast_cuda_heaphook.so"
[[ -f "${CUDA_HEAPHOOK}" ]] || fail "CUDA heaphook not built: ${CUDA_HEAPHOOK}
  build it with:  (cd ${AGNOCAST_ROOT}/agnocast_cuda_heaphook && cargo build --release)"

POOL_CONFIG="${AGNOCAST_ROOT}/src/agnocast_gpu_shared_memory_daemon/config/pool_config.yaml"
[[ -f "${POOL_CONFIG}" ]] || fail "pool config not found: ${POOL_CONFIG}"

# Launch the node binaries directly (not via `ros2 run`, which forks the node as a
# child so $! would be the wrapper, leaking the real process on kill).
DAEMON_BIN="${WS_ROOT}/install/agnocast_gpu_shared_memory_daemon/lib/agnocast_gpu_shared_memory_daemon/gpu_shared_memory_daemon"
TALKER_BIN="${WS_ROOT}/install/agnocast_sample_application/lib/agnocast_sample_application/cuda_talker"
LISTENER_BIN="${WS_ROOT}/install/agnocast_sample_application/lib/agnocast_sample_application/cuda_listener"
for bin in "${DAEMON_BIN}" "${TALKER_BIN}" "${LISTENER_BIN}"; do
  [[ -x "${bin}" ]] || fail "missing binary (run colcon build): ${bin}"
done

# The daemon needs to create its socket under /run/agnocast; clients derive the
# same path from the GPU UUID, so it cannot be relocated. Check it up front.
if [[ ! -d /run/agnocast || ! -w /run/agnocast ]]; then
  fail "/run/agnocast is missing or not writable. Create it once with:
    sudo mkdir -p /run/agnocast && sudo chown \"\$(id -u):\$(id -g)\" /run/agnocast"
fi

# --- Environment --------------------------------------------------------------
# ROS 2 setup scripts reference unset variables, so relax nounset while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || fail "could not source ROS 2 (humble)"
# shellcheck disable=SC1091
source "${WS_ROOT}/install/setup.bash" 2>/dev/null || fail "could not source workspace install/setup.bash"
set -u
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export CUDA_VISIBLE_DEVICES="${GPU_INDEX}"

PRELOAD="${CPU_HEAPHOOK}:${CUDA_HEAPHOOK}"

WORK_DIR="$(mktemp -d)"
DAEMON_LOG="${WORK_DIR}/daemon.log"
PUB_LOG="${WORK_DIR}/publisher.log"
SUB_LOG="${WORK_DIR}/subscriber.log"

DAEMON_PID="" ; PUB_PID="" ; SUB_PID=""
cleanup() {
  for pid in "${PUB_PID}" "${SUB_PID}" "${DAEMON_PID}"; do
    [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null
  done
  for pid in "${PUB_PID}" "${SUB_PID}" "${DAEMON_PID}"; do
    [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null
  done
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT INT TERM

# --- 1. Start the daemon ------------------------------------------------------
log "workspace : ${WS_ROOT}"
log "GPU index : ${CUDA_VISIBLE_DEVICES}"
log "starting daemon..."
"${DAEMON_BIN}" --config "${POOL_CONFIG}" >"${DAEMON_LOG}" 2>&1 &
DAEMON_PID=$!

# Wait for the daemon to announce its socket (or die).
SOCKET_PATH=""
for _ in $(seq 1 40); do
  if ! kill -0 "${DAEMON_PID}" 2>/dev/null; then
    cat "${DAEMON_LOG}" >&2
    fail "daemon exited during startup"
  fi
  SOCKET_PATH="$(sed -n 's/.* serving GPU .* on \(.*\) (.*/\1/p' "${DAEMON_LOG}" | head -1)"
  [[ -n "${SOCKET_PATH}" && -S "${SOCKET_PATH}" ]] && break
  sleep 0.25
done
[[ -n "${SOCKET_PATH}" && -S "${SOCKET_PATH}" ]] || { cat "${DAEMON_LOG}" >&2; fail "daemon did not create its socket in time"; }
log "daemon serving on ${SOCKET_PATH}"
log "$(grep 'serving GPU' "${DAEMON_LOG}" | head -1)"

# --- 2. Start subscriber, then publisher --------------------------------------
log "starting subscriber (cuda_listener)..."
LD_PRELOAD="${PRELOAD}" "${LISTENER_BIN}" >"${SUB_LOG}" 2>&1 &
SUB_PID=$!
sleep 1  # let the subscriber connect to the daemon and import slots before we publish

log "starting publisher (cuda_talker)..."
LD_PRELOAD="${PRELOAD}" "${TALKER_BIN}" >"${PUB_LOG}" 2>&1 &
PUB_PID=$!

# --- 3. Let them exchange messages -------------------------------------------
log "running for ${RUN_SECONDS}s..."
for _ in $(seq 1 "${RUN_SECONDS}"); do
  sleep 1
  kill -0 "${PUB_PID}" 2>/dev/null || { log "publisher exited early"; break; }
  kill -0 "${SUB_PID}" 2>/dev/null || { log "subscriber exited early"; break; }
done

# --- 4. Stop everything (triggers publisher's slot-reclaim path) --------------
kill -INT "${PUB_PID}" "${SUB_PID}" 2>/dev/null
sleep 1
kill -INT "${DAEMON_PID}" 2>/dev/null
sleep 0.5

# --- 5. Verify ----------------------------------------------------------------
echo "----------------------------------------------------------------------"
log "publisher log (tail):"; tail -n 4 "${PUB_LOG}" | sed 's/^/    /'
log "subscriber log (tail):"; tail -n 4 "${SUB_LOG}" | sed 's/^/    /'
echo "----------------------------------------------------------------------"

# Fatal-error fingerprints from the publish/subscribe integration.
if grep -q "was not allocated from the GPU pool" "${PUB_LOG}"; then
  fail "publisher's cudaMalloc was NOT served by the pool (heaphook/daemon not wired)"
fi
if grep -qE "gpu_data\(\) returned nullptr|SlotId .* not found" "${SUB_LOG}"; then
  fail "subscriber could not resolve the pooled GPU pointer"
fi

PUB_COUNT="$(grep -c "published CUDA PointCloud2" "${PUB_LOG}")"
[[ "${PUB_COUNT}" -gt 0 ]] || { cat "${PUB_LOG}" >&2; fail "publisher published 0 messages"; }
log "publisher published ${PUB_COUNT} messages"

# Every received line must show the incrementing GPU byte pattern written by the
# publisher: first_bytes=[b0,b1,b2,b3] with b_{k} == (b0 + k) % 256.
verified=0
mismatched=0
while IFS= read -r line; do
  bytes="$(sed -n 's/.*first_bytes=\[\([0-9,]*\)\].*/\1/p' <<<"${line}")"
  [[ -n "${bytes}" ]] || continue
  IFS=',' read -r b0 b1 b2 b3 <<<"${bytes}"
  if [[ "${b1}" == "$(( (b0 + 1) % 256 ))" && \
        "${b2}" == "$(( (b0 + 2) % 256 ))" && \
        "${b3}" == "$(( (b0 + 3) % 256 ))" ]]; then
    verified=$((verified + 1))
  else
    mismatched=$((mismatched + 1))
    log "byte-pattern MISMATCH: [${b0},${b1},${b2},${b3}]"
  fi
done < <(grep "received CUDA PointCloud2" "${SUB_LOG}")

log "subscriber received & byte-verified ${verified} messages (${mismatched} mismatched)"

# gpu_size sanity: height(1) * width(1024) * point_step(16) = 16384
if ! grep -q "gpu_size=16384" "${SUB_LOG}"; then
  fail "subscriber never reported the expected gpu_size=16384"
fi

[[ "${mismatched}" -eq 0 ]] || fail "one or more messages had corrupted GPU data"
[[ "${verified}" -ge "${MIN_RECEIVED}" ]] || \
  fail "only ${verified} verified messages (need >= ${MIN_RECEIVED}); GPU data did not cross processes reliably"

echo "======================================================================"
log "PASS: ${verified} messages transferred zero-copy through the GPU pool"
echo "======================================================================"
exit 0
