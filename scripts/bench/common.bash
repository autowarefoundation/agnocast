# shellcheck shell=bash
#
# Shared configuration and backend helpers for the Agnocast benchmark suite.
#
# The two backends are selected at *compile* time (agnocastlib's
# -DAGNOCAST_USE_DAEMON), so each one gets its own colcon build/install tree
# under ws/. Everything below resolves paths relative to those trees.

BENCH_ROOT="${BENCH_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
AGNOCAST_ROOT="${AGNOCAST_ROOT:-${BENCH_ROOT}}"

# Must match AGNOCAST_DAEMON_SOCKET_PATH in agnocast_daemon/protocol.h.
AGNOCAST_DAEMON_SOCKET_PATH="${AGNOCAST_DAEMON_SOCKET_PATH:-/tmp/agnocast_daemon.sock}"
AGNOCAST_DAEMON_BIN="${AGNOCAST_DAEMON_BIN:-${AGNOCAST_ROOT}/agnocast_daemon/build/agnocast_daemon}"

# PID of a daemon started by this shell (empty when we did not start one).
BENCH_DAEMON_PID=""

# Prefix used to launch/kill the daemon. Set to "sudo" by the runner when the
# bench processes themselves run as root under chrt, so that the daemon and its
# clients agree on the ownership of the shared memory they create.
DAEMON_LAUNCH_PREFIX="${DAEMON_LAUNCH_PREFIX:-}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

# --- Path helpers ------------------------------------------------------------

ws_dir() { echo "${BENCH_ROOT}/ws/$1"; }
install_dir() { echo "${BENCH_ROOT}/ws/$1/install"; }
build_dir() { echo "${BENCH_ROOT}/ws/$1/build"; }

heaphook_lib() { echo "$(install_dir "$1")/agnocastlib/lib/libagnocast_heaphook.so"; }
pub_bin() { echo "$(install_dir "$1")/agnocast_bench_publisher/lib/agnocast_bench_publisher/bench_publisher"; }
sub_bin() { echo "$(install_dir "$1")/agnocast_bench_subscriber/lib/agnocast_bench_subscriber/bench_subscriber"; }

validate_backend() {
  case "$1" in
  kmod | daemon) return 0 ;;
  *) die "unknown backend '$1' (expected 'kmod' or 'daemon')" ;;
  esac
}

backend_label() {
  case "$1" in
  kmod) echo "kernel module" ;;
  daemon) echo "user daemon" ;;
  esac
}

# Verify a backend has been built and source its install tree.
source_backend_ws() {
  local backend="$1"
  local setup
  setup="$(install_dir "${backend}")/setup.bash"

  [[ -f "${setup}" ]] || die "workspace for '${backend}' not built. Run: scripts/build.bash ${backend}"
  [[ -x "$(pub_bin "${backend}")" ]] || die "bench_publisher missing for '${backend}'. Run: scripts/build.bash ${backend}"
  [[ -x "$(sub_bin "${backend}")" ]] || die "bench_subscriber missing for '${backend}'. Run: scripts/build.bash ${backend}"
  [[ -f "$(heaphook_lib "${backend}")" ]] || die "libagnocast_heaphook.so missing for '${backend}'. Run: scripts/build.bash ${backend}"

  # ROS setup scripts reference unbound variables.
  set +u
  # shellcheck disable=SC1090
  source "${setup}"
  set -u
}

# --- Daemon lifecycle --------------------------------------------------------

daemon_running() { [[ -S "${AGNOCAST_DAEMON_SOCKET_PATH}" ]]; }

start_daemon() {
  if daemon_running; then
    echo "  agnocast_daemon: reusing running daemon (${AGNOCAST_DAEMON_SOCKET_PATH})"
    return 0
  fi

  [[ -x "${AGNOCAST_DAEMON_BIN}" ]] ||
    die "agnocast_daemon not found at ${AGNOCAST_DAEMON_BIN}. Run: scripts/build.bash daemon"

  mkdir -p "$(ws_dir daemon)"
  # shellcheck disable=SC2086
  ${DAEMON_LAUNCH_PREFIX} "${AGNOCAST_DAEMON_BIN}" >"$(ws_dir daemon)/agnocast_daemon.log" 2>&1 &
  BENCH_DAEMON_PID=$!

  for _ in $(seq 1 50); do
    if daemon_running; then
      echo "  agnocast_daemon: started (pid=${BENCH_DAEMON_PID})"
      return 0
    fi
    kill -0 "${BENCH_DAEMON_PID}" 2>/dev/null ||
      die "agnocast_daemon exited during startup; see ws/daemon/agnocast_daemon.log"
    sleep 0.1
  done

  die "agnocast_daemon did not create its socket within 5s"
}

stop_daemon() {
  if [[ -n "${BENCH_DAEMON_PID}" ]] && kill -0 "${BENCH_DAEMON_PID}" 2>/dev/null; then
    # shellcheck disable=SC2086
    ${DAEMON_LAUNCH_PREFIX} pkill -TERM -P "${BENCH_DAEMON_PID}" 2>/dev/null || true
    kill -TERM "${BENCH_DAEMON_PID}" 2>/dev/null || true
    wait "${BENCH_DAEMON_PID}" 2>/dev/null || true
    echo "  agnocast_daemon: stopped (pid=${BENCH_DAEMON_PID})"
  fi
  BENCH_DAEMON_PID=""
}

# Tear down and bring back up a fresh daemon. Used between iterations so that
# every iteration starts from an empty metadata table, mirroring the
# process-table drain that kernel-module mode gets from poll_for_unlink.
restart_daemon() {
  stop_daemon
  for _ in $(seq 1 50); do
    daemon_running || break
    sleep 0.1
  done
  rm -f "${AGNOCAST_DAEMON_SOCKET_PATH}" 2>/dev/null || true
  start_daemon
}

# Bring up whatever the backend needs. In daemon mode a daemon we started is
# stopped on exit; an externally managed one is left alone.
require_backend() {
  local backend="$1"

  if [[ "${backend}" == "daemon" ]]; then
    if grep -q "^agnocast " /proc/modules 2>/dev/null; then
      echo "  NOTE: the agnocast kernel module is loaded but unused in daemon mode."
    fi
    start_daemon
    return 0
  fi

  grep -q "^agnocast " /proc/modules 2>/dev/null ||
    die "agnocast kernel module is not loaded. Load it: sudo insmod ${AGNOCAST_ROOT}/agnocast_kmod/agnocast.ko"
  if daemon_running; then
    echo "  NOTE: an agnocast_daemon socket exists but is unused in kernel-module mode."
  fi
  return 0
}
