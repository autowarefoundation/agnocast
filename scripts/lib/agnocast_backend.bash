# shellcheck shell=bash
# Shared helpers for selecting the Agnocast metadata-management backend
# (kernel module vs. user daemon) in build/run/test scripts.
#
# The backend is selected via the AGNOCAST_USE_DAEMON environment variable:
#   AGNOCAST_USE_DAEMON=1 (or on/true/yes)  -> user daemon (agnocast_daemon)
#   unset / 0 / off / false / no            -> kernel module (default)
#
# NOTE: agnocastlib picks its backend at *compile* time via the CMake option of
# the same name. Daemon mode therefore only works if the workspace was built
# with -DAGNOCAST_USE_DAEMON=ON (see scripts/dev/build_all_daemon.bash). This
# variable merely tells the scripts which backend to prepare and verify; it does
# not change how an already-built binary behaves.

# Resolve the repository root from this file's location (scripts/lib/..).
_agnocast_backend_lib_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGNOCAST_ROOT="${AGNOCAST_ROOT:-$(cd "${_agnocast_backend_lib_dir}/../.." && pwd)}"

# Must match AGNOCAST_DAEMON_SOCKET_PATH in agnocast_daemon/protocol.h.
AGNOCAST_DAEMON_SOCKET_PATH="${AGNOCAST_DAEMON_SOCKET_PATH:-/tmp/agnocast_daemon.sock}"
AGNOCAST_DAEMON_BIN="${AGNOCAST_DAEMON_BIN:-${AGNOCAST_ROOT}/agnocast_daemon/build/agnocast_daemon}"

# PID of a daemon this shell started (empty if we did not start one).
AGNOCAST_DAEMON_PID=""

# Return 0 when daemon mode is selected, 1 otherwise.
agnocast_daemon_mode() {
    case "$(echo "${AGNOCAST_USE_DAEMON:-0}" | tr '[:upper:]' '[:lower:]')" in
    1 | on | true | yes) return 0 ;;
    *) return 1 ;;
    esac
}

# Human-readable backend name.
agnocast_backend_name() {
    if agnocast_daemon_mode; then
        echo "user daemon"
    else
        echo "kernel module"
    fi
}

# Return 0 if the daemon socket is present (i.e. a daemon is listening).
agnocast_daemon_running() {
    [ -S "$AGNOCAST_DAEMON_SOCKET_PATH" ]
}

# Start agnocast_daemon in the background and wait until its socket appears.
# Records the PID in AGNOCAST_DAEMON_PID. Returns non-zero on failure.
agnocast_start_daemon() {
    if agnocast_daemon_running; then
        echo "agnocast_daemon: reusing running daemon (socket: $AGNOCAST_DAEMON_SOCKET_PATH)"
        return 0
    fi

    if [ ! -x "$AGNOCAST_DAEMON_BIN" ]; then
        echo "ERROR: agnocast_daemon binary not found or not executable:" >&2
        echo "         $AGNOCAST_DAEMON_BIN" >&2
        echo "       Build it first: (cd agnocast_daemon && make)" >&2
        echo "       or run: scripts/dev/build_all_daemon.bash" >&2
        return 1
    fi

    echo "agnocast_daemon: starting ($AGNOCAST_DAEMON_BIN)"
    "$AGNOCAST_DAEMON_BIN" &
    AGNOCAST_DAEMON_PID=$!

    # Wait up to ~5s for the socket to be created.
    for _ in $(seq 1 50); do
        if agnocast_daemon_running; then
            echo "agnocast_daemon: ready (pid=$AGNOCAST_DAEMON_PID)"
            return 0
        fi
        if ! kill -0 "$AGNOCAST_DAEMON_PID" 2>/dev/null; then
            echo "ERROR: agnocast_daemon exited during startup." >&2
            AGNOCAST_DAEMON_PID=""
            return 1
        fi
        sleep 0.1
    done

    echo "ERROR: agnocast_daemon did not create its socket within timeout." >&2
    agnocast_stop_daemon
    return 1
}

# Stop the daemon this shell started (no-op if we did not start one).
agnocast_stop_daemon() {
    if [ -n "$AGNOCAST_DAEMON_PID" ] && kill -0 "$AGNOCAST_DAEMON_PID" 2>/dev/null; then
        kill -TERM "$AGNOCAST_DAEMON_PID" 2>/dev/null
        wait "$AGNOCAST_DAEMON_PID" 2>/dev/null
        echo "agnocast_daemon: stopped (pid=$AGNOCAST_DAEMON_PID)"
    fi
    AGNOCAST_DAEMON_PID=""
}

# Ensure the selected backend is available before running a test.
#   - kernel module mode: verify agnocast.ko is loaded (fatal if not).
#   - daemon mode: reuse a running daemon, or start one and arrange to stop it
#     on script exit.
# Exits the calling script on failure.
agnocast_require_backend() {
    echo "Agnocast backend: $(agnocast_backend_name)"

    if agnocast_daemon_mode; then
        if agnocast_daemon_running; then
            echo "agnocast_daemon: using already-running daemon (socket: $AGNOCAST_DAEMON_SOCKET_PATH)"
            return 0
        fi
        if ! agnocast_start_daemon; then
            exit 1
        fi
        # Stop only the daemon we started; leave externally managed daemons alone.
        trap 'agnocast_stop_daemon' EXIT
        return 0
    fi

    if ! grep -q "^agnocast " /proc/modules; then
        echo "ERROR: agnocast kernel module is not loaded." >&2
        echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
        echo "(Or select the user daemon backend with AGNOCAST_USE_DAEMON=1.)" >&2
        exit 1
    fi
}
