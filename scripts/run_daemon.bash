#!/bin/bash

# Run the Agnocast user-space daemon (agnocast_daemon) in the foreground.
#
# In user daemon mode, this process must be running before any Agnocast
# application starts, because agnocastlib (built with -DAGNOCAST_USE_DAEMON=ON)
# connects to its Unix socket instead of the kernel module's /dev/agnocast.
#
# Build the daemon first with either:
#   (cd agnocast_daemon && make)
#   scripts/dev/build_all_daemon.bash
#
# Press Ctrl-C to stop; the daemon removes its socket file on exit.
#
# Run from the repository root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/agnocast_backend.bash
source "${SCRIPT_DIR}/lib/agnocast_backend.bash"

if [ ! -x "$AGNOCAST_DAEMON_BIN" ]; then
    echo "ERROR: agnocast_daemon binary not found or not executable:" >&2
    echo "         $AGNOCAST_DAEMON_BIN" >&2
    echo "       Build it first: (cd agnocast_daemon && make)" >&2
    exit 1
fi

if agnocast_daemon_running; then
    echo "ERROR: a daemon is already listening on $AGNOCAST_DAEMON_SOCKET_PATH" >&2
    echo "       Stop it before starting a new one." >&2
    exit 1
fi

echo "Starting agnocast_daemon (Ctrl-C to stop)..."
exec "$AGNOCAST_DAEMON_BIN"
