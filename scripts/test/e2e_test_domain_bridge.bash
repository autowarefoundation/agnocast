#!/bin/bash
# Same-IPC-namespace cross-domain zero-copy e2e (two ROS domains, one mempool).
#
# Registers a domain bridge rule for the sample topic, then runs the sample
# talker in FROM_DOMAIN and the sample listener in TO_DOMAIN as SEPARATE launches
# and asserts the listener received the talker's messages — cross-domain delivery
# through the kmod with no DDS / bridge node.
#
# Two separate launches (not one launch file) are required: each `ros2 launch`
# runs entirely under one ROS_DOMAIN_ID, including the composable-node loader. A
# single launch can't load nodes into containers on two different domains.
#
# Each launch is run under `timeout` so it self-terminates (SIGINT, then SIGKILL
# after a grace period) — Agnocast container shutdown can stall on SIGINT, so we
# never block on a graceful exit.
#
# Run on a freshly loaded kmod — the rule must be registered before any endpoint
# for the topic joins. This path uses no bridge, so AGNOCAST_BRIDGE_MODE=off.

set -uo pipefail

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

TOPIC="${E2E_TOPIC_NAME:-/my_topic}"   # the sample app's topic
FROM_DOMAIN="${E2E_FROM_DOMAIN:-1}"
TO_DOMAIN="${E2E_TO_DOMAIN:-2}"
RUN_SECONDS="${E2E_RUN_SECONDS:-10}"

echo "Registering domain bridge rule: ${TOPIC} ${FROM_DOMAIN}->${TO_DOMAIN}"
# Register through the same tool production uses, not an inline ioctl call.
BRIDGE_CONFIG="$(mktemp --suffix=.yaml)"
cat > "$BRIDGE_CONFIG" <<YAML
from_domain: ${FROM_DOMAIN}
to_domain: ${TO_DOMAIN}
topics:
  "${TOPIC}":
YAML
if ! ros2 run ros2agnocast_discovery_agent register_domain_bridge --config "$BRIDGE_CONFIG"; then
    echo "Rule registration failed (was the kmod freshly loaded?)." >&2
    rm -f "$BRIDGE_CONFIG"
    exit 1
fi
rm -f "$BRIDGE_CONFIG"

SUBLOG="$(mktemp)"; PUBLOG="$(mktemp)"
LISTEN_SECS=$((5 + RUN_SECONDS + 3))   # listener: startup + run + drain
TALK_SECS=$((RUN_SECONDS + 3))         # talker: starts 5 s later

echo "Starting listener in domain ${TO_DOMAIN} (subscriber first), ~${LISTEN_SECS}s ..."
timeout -s INT -k 5 "$LISTEN_SECS" \
    env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$TO_DOMAIN" \
    ros2 launch agnocast_sample_application listener.launch.xml > "$SUBLOG" 2>&1 &
SUB_PID=$!
sleep 5

echo "Starting talker in domain ${FROM_DOMAIN}, ~${TALK_SECS}s ..."
timeout -s INT -k 5 "$TALK_SECS" \
    env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$FROM_DOMAIN" \
    ros2 launch agnocast_sample_application talker.launch.xml > "$PUBLOG" 2>&1 &
PUB_PID=$!

echo "Running (auto-stop via timeout) ..."
wait "$SUB_PID" "$PUB_PID" 2>/dev/null || true
sleep 1

echo "--- talker log (tail) ---";   tail -n 3 "$PUBLOG"
echo "--- listener log (tail) ---"; tail -n 3 "$SUBLOG"

pub_count=$(grep -c "publish message: id=" "$PUBLOG" || true)
sub_count=$(grep -c "subscribe message: id=" "$SUBLOG" || true)
echo "published=${pub_count}  received(cross-domain)=${sub_count}"

if [ "$pub_count" -gt 0 ] && [ "$sub_count" -gt 0 ]; then
    echo "PASS: domain ${FROM_DOMAIN} publisher reached domain ${TO_DOMAIN} subscriber (cross-domain zero-copy)."
    exit 0
fi
echo "FAIL: cross-domain delivery not observed (published=${pub_count}, received=${sub_count})." >&2
exit 1
