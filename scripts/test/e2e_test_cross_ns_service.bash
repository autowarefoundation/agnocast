#!/bin/bash
# E2E for an Agnocast service shared by two IPC namespaces: the service in one, the client in the
# other. The kmod never pairs them, so the only path between the two is DDS -- an R2A bridge beside
# the service and an A2R bridge beside the client -- and neither manager can observe the pairing
# that makes its own half necessary. Bringing both up is the discovery agent's job.
#
# Only discriminating on Jazzy and newer. Pre-Jazzy rclcpp has no service-client count, so R2A is
# ungated and its ROS 2 service alone satisfies the client side, without the agent ever being
# involved. The test still passes there, it just proves less.
#
# No sudo: `unshare --user` supplies the capability `--ipc` wants. /dev/shm and the network stack
# stay shared, since that is the DDS path the bridges are supposed to use.
#
# Requires the agnocast kernel module loaded and the workspace built.
#
# Usage:
#   bash scripts/test/e2e_test_cross_ns_service.bash
#
# Exit codes: 0 pass, 1 prerequisite failure, 2 assertion failure

set -uo pipefail

ROOT_DIR=$(cd "$(dirname "$0")/../.." && pwd)
DOMAIN="${E2E_DOMAIN_ID:-71}"
CLIENT_SECS="${E2E_CLIENT_SECS:-40}"
# Overridable so the test can run against an out-of-tree install (e.g. a second distro's build).
INSTALL_SETUP="${AGNOCAST_INSTALL_SETUP:-$ROOT_DIR/install/setup.bash}"
SERVER_SECS="${E2E_SERVER_SECS:-$((CLIENT_SECS + 10))}"

# minimal_client.cpp sends 1..100 and 0..99. Assert the sums, not just that something came back.
SUM1=5050
SUM2=4950

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }

if ! grep -q "^agnocast " /proc/modules; then
    red "ERROR: agnocast kernel module is not loaded."
    echo "  -> sudo insmod $ROOT_DIR/agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

if [ ! -f "$INSTALL_SETUP" ]; then
    red "ERROR: workspace not built ($INSTALL_SETUP missing)."
    echo "  -> bash $ROOT_DIR/scripts/dev/build_all.bash" >&2
    exit 1
fi

# With CAP_SYS_ADMIN (root in a container) --ipc is enough; without it, a new user namespace
# supplies the capability, which is what lets an ordinary user run this.
if unshare --ipc --fork true 2>/dev/null; then
    UNSHARE_ARGS=(--ipc --fork)
elif unshare --user --map-root-user --ipc --fork true 2>/dev/null; then
    UNSHARE_ARGS=(--user --map-root-user --ipc --fork)
else
    red "ERROR: cannot create an IPC namespace on this host."
    echo "  -> needs CAP_SYS_ADMIN, or unprivileged user namespaces" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "$INSTALL_SETUP"
set -u

LOG_DIR=$(mktemp -d)
cleanup() {
    pkill -P $$ 2>/dev/null
    rm -rf "$LOG_DIR"
}
trap cleanup EXIT

# start_in_new_ipc_ns <log> <secs> <exec>; leaves the pid in NS_PID. The first log line records the
# namespace so the run can prove the two halves really were isolated.
#
# LD_PRELOAD is applied inside the namespace, never around unshare: the heaphook starts threads on
# load, and unshare(CLONE_NEWUSER) refuses a multi-threaded caller with EINVAL.
start_in_new_ipc_ns() {
    local log="$1" secs="$2" exec_name="$3"
    timeout -s INT -k 5 "$secs" \
        unshare "${UNSHARE_ARGS[@]}" \
        bash -c 'echo "ipc-ns $(readlink /proc/self/ns/ipc)"
                 exec env LD_PRELOAD=libagnocast_heaphook.so \
                     ros2 run agnocast_sample_application "$1"' _ "$exec_name" \
        > "$log" 2>&1 &
    NS_PID=$!
}

export ROS_DOMAIN_ID="$DOMAIN"

server_log="$LOG_DIR/server.log"
client_log="$LOG_DIR/client.log"

echo ">>> server: Agnocast service in its own IPC namespace (~${SERVER_SECS}s)"
start_in_new_ipc_ns "$server_log" "$SERVER_SECS" server
server_pid=$NS_PID
sleep 5

echo ">>> client: Agnocast client in a second IPC namespace (~${CLIENT_SECS}s)"
start_in_new_ipc_ns "$client_log" "$CLIENT_SECS" client
client_pid=$NS_PID

wait "$client_pid" 2>/dev/null
kill -INT "$server_pid" 2>/dev/null
wait "$server_pid" 2>/dev/null

bad=0

server_ns=$(sed -n 's/^ipc-ns //p' "$server_log" | head -1)
client_ns=$(sed -n 's/^ipc-ns //p' "$client_log" | head -1)
echo "  namespaces: server=${server_ns:-?} client=${client_ns:-?}"
if [ -z "$server_ns" ] || [ "$server_ns" = "$client_ns" ]; then
    red "  FAIL: the two halves did not land in distinct IPC namespaces."
    bad=$((bad + 1))
fi

got1=$(grep -cE "Result1: ${SUM1}\$" "$client_log")
got2=$(grep -cE "Result2: ${SUM2}\$" "$client_log")
echo "  client: Result1=${SUM1}x${got1} Result2=${SUM2}x${got2}"
if [ "$got1" -lt 1 ] || [ "$got2" -lt 1 ]; then
    red "  FAIL: the client never got an answer across the namespace boundary."
    bad=$((bad + 1))
fi

served1=$(grep -c "Sending back response: \[${SUM1}\]" "$server_log")
served2=$(grep -c "Sending back response: \[${SUM2}\]" "$server_log")
echo "  server: [${SUM1}]x${served1} [${SUM2}]x${served2} (expected 1 each)"
if [ "$served1" -ne 1 ] || [ "$served2" -ne 1 ]; then
    red "  FAIL: the server served ${served1}/${served2} requests, expected 1 of each."
    bad=$((bad + 1))
fi

if [ "$bad" -ne 0 ]; then
    echo "----- server -----" >&2; sed 's/^/    /' "$server_log" >&2
    echo "----- client -----" >&2; sed 's/^/    /' "$client_log" >&2
    exit 2
fi

green "PASS: the service in ${server_ns} answered the client in ${client_ns}."
