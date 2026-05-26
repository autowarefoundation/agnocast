#!/bin/bash
#
# E2E regression smoke test for the ros2agnocast CLI verbs after they
# learned to subscribe to /_agnocast_discovery and merge cross-NS /
# cross-ECU state.
#
# This script does not actually create multiple IPC namespaces (that
# requires sudo unshare); instead it runs the daemon and the CLI in the
# same namespace and checks that:
#
#   * Each verb accepts `--gossip-timeout`.
#   * Each verb finishes without raising when the daemon is publishing.
#   * Local Agnocast endpoints continue to surface (the ioctl path,
#     unchanged by this PR's gossip wiring).
#
# `collect_announcements()` deliberately drops the caller's own NS from
# gossip, so in this single-NS setup the gossip-derived snapshot list is
# always empty and the topic/node/endpoint info you see below comes from
# the ioctl path. Cross-NS gossip visibility is exercised by separate
# manual unshare-based tests.
#
# Requires:
#   * agnocast kernel module loaded
#   * workspace built (bash scripts/dev/build_all.bash)
#
# Usage:
#   bash scripts/test/e2e_test_cross_ns_cli.bash
#
# Exit codes:
#   0 - all checks passed
#   1 - prerequisite failure
#   2 - assertion failure

ROOT_DIR=$(cd "$(dirname "$0")/../.." && pwd)
GOSSIP_TIMEOUT=${GOSSIP_TIMEOUT:-1.0}
DAEMON_WARMUP_SEC=${DAEMON_WARMUP_SEC:-2}
TALKER_WARMUP_SEC=${TALKER_WARMUP_SEC:-3}

red()    { printf '\033[31m%s\033[0m\n' "$*"; }
green()  { printf '\033[32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[33m%s\033[0m\n' "$*"; }

if ! grep -q "^agnocast " /proc/modules; then
    red "ERROR: agnocast kmod not loaded."
    echo "  -> sudo insmod $ROOT_DIR/agnocast_kmod/agnocast.ko" >&2
    exit 1
fi
if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
    red "ERROR: workspace not built ($ROOT_DIR/install/setup.bash missing)."
    echo "  -> bash $ROOT_DIR/scripts/dev/build_all.bash" >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u
green "✓ kmod loaded and workspace sourced"

LOG_DIR=$(mktemp -d)
cleanup() {
    pkill -P $$ 2>/dev/null || true
    sleep 1
    rm -rf "$LOG_DIR"
}
trap cleanup EXIT

yellow "Starting agnocast_discovery_agent and talker..."
ros2 run ros2agnocast_discovery_agent discovery_agent > "$LOG_DIR/agent.log" 2>&1 &
sleep "$DAEMON_WARMUP_SEC"
ros2 launch agnocast_sample_application talker.launch.xml > "$LOG_DIR/talker.log" 2>&1 &
sleep "$TALKER_WARMUP_SEC"

if ! grep -q "discovery_agent up" "$LOG_DIR/agent.log"; then
    red "ERROR: daemon did not start within ${DAEMON_WARMUP_SEC}s."
    cat "$LOG_DIR/agent.log" >&2
    exit 2
fi
green "✓ daemon + talker running"

fail() {
    red "ERROR: $1"
    [ -n "${2:-}" ] && { echo "----- output -----" >&2; printf '%s\n' "$2" >&2; }
    echo "----- agent log -----" >&2; cat "$LOG_DIR/agent.log" >&2
    echo "----- talker log -----" >&2; cat "$LOG_DIR/talker.log" >&2
    exit 2
}

# ----- topic list_agnocast -----
out=$(ros2 topic list_agnocast --gossip-timeout "$GOSSIP_TIMEOUT" 2>&1) \
    || fail "topic list_agnocast exited non-zero" "$out"
grep -q -- "/my_topic" <<<"$out" \
    || fail "topic list_agnocast did not include /my_topic" "$out"
grep -- "/my_topic" <<<"$out" | grep -q "Agnocast enabled" \
    || fail "/my_topic present but missing '(Agnocast enabled)' annotation" "$out"
green "✓ topic list_agnocast"

# ----- node list_agnocast -----
out=$(ros2 node list_agnocast --gossip-timeout "$GOSSIP_TIMEOUT" 2>&1) \
    || fail "node list_agnocast exited non-zero" "$out"
grep -q -- "/talker_node" <<<"$out" \
    || fail "node list_agnocast did not include /talker_node" "$out"
green "✓ node list_agnocast"

# ----- topic info_agnocast -----
out=$(ros2 topic info_agnocast /my_topic --gossip-timeout "$GOSSIP_TIMEOUT" 2>&1) \
    || fail "topic info_agnocast exited non-zero" "$out"
grep -qE "Agnocast Publisher count:\s*1" <<<"$out" \
    || fail "topic info_agnocast did not report 1 Agnocast publisher" "$out"
green "✓ topic info_agnocast"

# ----- node info_agnocast -----
out=$(ros2 node info_agnocast /talker_node --gossip-timeout "$GOSSIP_TIMEOUT" 2>&1) \
    || fail "node info_agnocast exited non-zero" "$out"
grep -q -- "/my_topic" <<<"$out" \
    || fail "node info_agnocast /talker_node did not list /my_topic" "$out"
green "✓ node info_agnocast"

green ""
green "===== ALL CHECKS PASSED ====="
