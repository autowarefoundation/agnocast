#!/bin/bash
# Same-IPC-namespace cross-domain zero-copy e2e (two ROS domains, one mempool).
#
# Registers a domain bridge rule for the sample topic, then runs the sample talker
# in one domain and the sample listener in the other as SEPARATE launches, and
# asserts the listener received the talker's messages — cross-domain delivery
# through the kmod with no DDS / bridge node.
#
# Set E2E_REMAP=/some/topic to exercise the cross-domain *rename* path (the rule
# maps ${TOPIC}@from to ${E2E_REMAP}@to). Rename runs TWO exchanges so one run
# covers both notification-MQ cases: the publish-notification MQ name is the
# bridge pair's canonical name = the *lower-numbered domain's* name, so exactly
# one endpoint per exchange has to switch to it. The forward exchange puts the
# source in the lower domain (the subscriber switches); the swapped exchange puts
# the source in the higher domain (the publisher switches). The two use distinct
# cells ({TOPIC@from, REMAP@to} vs {TOPIC@to, REMAP@from}), so both rules and
# their endpoints coexist on one kmod without a reload.
#
# Two separate launches (not one launch file) are required: each `ros2 launch`
# runs entirely under one ROS_DOMAIN_ID, including the composable-node loader. A
# single launch can't load nodes into containers on two different domains.
#
# Each launch is run under `timeout` so it self-terminates (SIGINT, then SIGKILL
# after a grace period) — Agnocast container shutdown can stall on SIGINT, so we
# never block on a graceful exit.
#
# Run on a freshly loaded kmod — a rule must be registered before any endpoint for
# its cells joins. This path uses no bridge, so AGNOCAST_BRIDGE_MODE=off.

set -uo pipefail

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

if ! ros2 pkg prefix ros2agnocast_discovery_agent >/dev/null 2>&1; then
    echo "ERROR: ros2agnocast_discovery_agent not found -- source the workspace first:" >&2
    echo "  source /opt/ros/<distro>/setup.bash && source install/setup.bash" >&2
    exit 1
fi

TOPIC="${E2E_TOPIC_NAME:-/my_topic}"   # the sample app's (source) topic
FROM_DOMAIN="${E2E_FROM_DOMAIN:-1}"
TO_DOMAIN="${E2E_TO_DOMAIN:-2}"
RUN_SECONDS="${E2E_RUN_SECONDS:-10}"
REMAP="${E2E_REMAP:-}"                 # set to a topic name to exercise cross-domain RENAME

LISTEN_SECS=$((5 + RUN_SECONDS + 3))   # listener: startup + run + drain
TALK_SECS=$((RUN_SECONDS + 3))         # talker: starts 5 s later

# run_exchange <from_domain> <to_domain>
#   Talker publishes ${TOPIC} in <from_domain>; listener subscribes ${REMAP:-$TOPIC} in
#   <to_domain>. Registers the rule, runs both under `timeout`, asserts delivery. Returns 0/1.
run_exchange() {
    local from_domain="$1" to_domain="$2"
    local sub_topic="${REMAP:-$TOPIC}"

    if [ -n "$REMAP" ]; then
        echo ">>> RENAME exchange: ${TOPIC}@${from_domain} -> ${REMAP}@${to_domain}"
    else
        echo ">>> exchange: ${TOPIC} ${from_domain}->${to_domain}"
    fi

    # Register through the same tool production uses, not an inline ioctl call.
    local cfg; cfg="$(mktemp --suffix=.yaml)"
    {
        echo "from_domain: ${from_domain}"
        echo "to_domain: ${to_domain}"
        echo "topics:"
        echo "  \"${TOPIC}\":"
        if [ -n "$REMAP" ]; then echo "    remap: \"${REMAP}\""; fi
    } > "$cfg"
    if ! ros2 run ros2agnocast_discovery_agent register_domain_bridge --config "$cfg"; then
        echo "Rule registration failed (fresh kmod? cell already has an endpoint?)." >&2
        rm -f "$cfg"; return 1
    fi
    rm -f "$cfg"

    # The packaged listener (MinimalSubscriber) hardcodes ${TOPIC}; for the rename path the
    # subscriber must be on ${REMAP}. Use NoRclcppSubscriber, which takes the topic as a param
    # (deterministic, no remap ambiguity) and logs the name it actually resolved.
    local listen_launch="" sub_match
    local -a listen_args
    if [ -n "$REMAP" ]; then
        listen_launch="$(mktemp --suffix=.launch.xml)"
        cat > "$listen_launch" <<XML
<launch>
  <node_container pkg="agnocast_components" exec="agnocast_component_container" name="listener_container" namespace="" output="screen">
    <env name="LD_PRELOAD" value="libagnocast_heaphook.so:\$(env LD_PRELOAD '')" />
    <composable_node pkg="agnocast_sample_application" plugin="NoRclcppSubscriber" name="listener_node" namespace="">
      <param name="topic_name" value="${REMAP}"/>
    </composable_node>
  </node_container>
</launch>
XML
        listen_args=("$listen_launch")
        sub_match="I heard dynamic size array message"
    else
        listen_args=(agnocast_sample_application listener.launch.xml)
        sub_match="subscribe message: id="
    fi

    local sublog publog; sublog="$(mktemp)"; publog="$(mktemp)"

    echo "  listener: ${sub_topic}@${to_domain} (subscriber first, ~${LISTEN_SECS}s)"
    timeout -s INT -k 5 "$LISTEN_SECS" \
        env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$to_domain" \
        ros2 launch "${listen_args[@]}" > "$sublog" 2>&1 &
    local sub_pid=$!
    sleep 5

    echo "  talker:   ${TOPIC}@${from_domain} (~${TALK_SECS}s)"
    timeout -s INT -k 5 "$TALK_SECS" \
        env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$from_domain" \
        ros2 launch agnocast_sample_application talker.launch.xml > "$publog" 2>&1 &
    local pub_pid=$!

    wait "$sub_pid" "$pub_pid" 2>/dev/null || true
    sleep 1
    [ -n "$listen_launch" ] && rm -f "$listen_launch"

    if [ -n "$REMAP" ]; then
        grep -E "Topic name \((input|resolved)\)" "$sublog" | sed 's/^/  /' \
            || echo "  (no resolved-topic log found)"
    fi

    local pub_count sub_count
    pub_count=$(grep -c "publish message: id=" "$publog" || true)
    sub_count=$(grep -c "$sub_match" "$sublog" || true)
    echo "  published(${TOPIC}@${from_domain})=${pub_count}  received(${sub_topic}@${to_domain})=${sub_count}"
    rm -f "$sublog" "$publog"

    if [ "$pub_count" -gt 0 ] && [ "$sub_count" -gt 0 ]; then
        echo "  PASS: ${TOPIC}@${from_domain} reached ${sub_topic}@${to_domain} (cross-domain zero-copy)."
        return 0
    fi
    echo "  FAIL: cross-domain delivery not observed (published=${pub_count}, received=${sub_count})." >&2
    return 1
}

if [ -n "$REMAP" ]; then
    # Forward: source in the lower domain -> the subscriber switches to the canonical name.
    run_exchange "$FROM_DOMAIN" "$TO_DOMAIN" || exit 1
    # Swapped: source in the higher domain -> the publisher switches to the canonical name.
    run_exchange "$TO_DOMAIN" "$FROM_DOMAIN" || exit 1
    echo "ALL RENAME EXCHANGES PASSED (subscriber-switch + publisher-switch)."
else
    run_exchange "$FROM_DOMAIN" "$TO_DOMAIN" || exit 1
fi
