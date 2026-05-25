#!/bin/bash
#
# Set up a manual test environment for ros2agnocast CLI commands
# (hz_agnocast, echo_agnocast, delay_agnocast, etc.).
#
# Usage:
#   bash scripts/test/setup_manual_test_env.bash [options]
#
# Options:
#   -h, --help                Show this help message
#   --agnocast-pub            Start an Agnocast publisher (no_rclcpp_pose_talker)
#   --agnocast-pub-isolated   Start the Agnocast publisher in an isolated IPC namespace
#                             (implies --agnocast-pub; you will be prompted for your
#                             sudo password to run unshare)
#   --ros2-pub                Start a ROS 2 publisher (pose_talker)
#   --agnocast-sub            Start an Agnocast subscriber to trigger the R2A bridge
#                             (requires --ros2-pub)
#
# Examples:
#   bash scripts/test/setup_manual_test_env.bash --agnocast-pub
#   bash scripts/test/setup_manual_test_env.bash --agnocast-pub-isolated
#   bash scripts/test/setup_manual_test_env.bash --ros2-pub --agnocast-sub
#   bash scripts/test/setup_manual_test_env.bash --agnocast-pub --ros2-pub
#
# Exit codes:
#   0  - environment started successfully (Ctrl-C to stop)
#   1  - argument or prerequisite error

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")/../.." && pwd)

red()    { printf '\033[31m%s\033[0m\n' "$*"; }
green()  { printf '\033[32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[33m%s\033[0m\n' "$*"; }
cyan()   { printf '\033[36m%s\033[0m\n' "$*"; }
bold()   { printf '\033[1m%s\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
AGNOCAST_PUB=false
AGNOCAST_PUB_ISOLATED=false
ROS2_PUB=false
AGNOCAST_SUB=false

usage() {
    sed -n '/^# Usage:/,/^# Exit codes:/{ /^# Exit codes:/!p }' "$0" | sed 's/^# \?//'
    exit 0
}

if [[ $# -eq 0 ]]; then
    red "ERROR: No options specified."
    echo ""
    usage
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)               usage ;;
        --agnocast-pub)          AGNOCAST_PUB=true; shift ;;
        --agnocast-pub-isolated) AGNOCAST_PUB_ISOLATED=true; AGNOCAST_PUB=true; shift ;;
        --ros2-pub)              ROS2_PUB=true; shift ;;
        --agnocast-sub)          AGNOCAST_SUB=true; shift ;;
        *)
            red "ERROR: Unknown option: $1"
            echo ""
            usage
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
if $AGNOCAST_SUB && ! $ROS2_PUB; then
    red "ERROR: --agnocast-sub requires --ros2-pub (the subscriber bridges a ROS 2 topic into Agnocast)."
    exit 1
fi

if $AGNOCAST_PUB_ISOLATED; then
    yellow "--agnocast-pub-isolated requires sudo. You may be prompted for your password."
    # Pre-authenticate before the background launch so sudo -E unshare does
    # not need a tty.  sudo -E preserves the caller's environment
    # (e.g. RMW_IMPLEMENTATION) into the unshared shell; without it the
    # isolated process may use a different RMW, making data delivery unreliable.
    sudo -v || { red "ERROR: sudo authentication failed."; exit 1; }
fi

# ---------------------------------------------------------------------------
# Prerequisites
# ---------------------------------------------------------------------------
if ! grep -q "^agnocast " /proc/modules; then
    red "ERROR: agnocast kmod not loaded."
    echo "  -> sudo insmod $ROOT_DIR/agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
    red "ERROR: workspace not built ($ROOT_DIR/install/setup.bash missing)."
    echo "  -> bash $ROOT_DIR/scripts/dev/build_all.bash" >&2
    exit 1
fi

# shellcheck disable=SC1091
set +u
source "$ROOT_DIR/install/setup.bash"
set -u
green "✓ kmod loaded and workspace sourced"

# ---------------------------------------------------------------------------
# Determine discovery_agent ownership
# --agnocast-pub and --agnocast-sub share a namespace, so only one of them
# should start the discovery_agent.
# --agnocast-pub-isolated runs in a separate IPC namespace, so its own
# discovery_agent is managed inside the unshare'd shell.
# ---------------------------------------------------------------------------
# Whether the same-namespace Agnocast components need a discovery_agent
NEED_SHARED_DA=false
$AGNOCAST_PUB && ! $AGNOCAST_PUB_ISOLATED && NEED_SHARED_DA=true
$AGNOCAST_SUB && NEED_SHARED_DA=true

# For same-NS launches: pub owns the agent (if both pub+sub, pub gets it)
PUB_STARTS_DA=false
SUB_STARTS_DA=false
if $NEED_SHARED_DA; then
    if $AGNOCAST_PUB && ! $AGNOCAST_PUB_ISOLATED; then
        PUB_STARTS_DA=true
    elif $AGNOCAST_SUB; then
        SUB_STARTS_DA=true
    fi
fi

# ---------------------------------------------------------------------------
# Pre-launch summary
# ---------------------------------------------------------------------------
echo ""
bold "=== Test environment to be launched ==="
echo ""

if $AGNOCAST_PUB; then
    if $AGNOCAST_PUB_ISOLATED; then
        cyan "  [Agnocast publisher]  no_rclcpp_pose_talker  (isolated IPC namespace)"
        echo  "                          └─ discovery_agent: YES"
    else
        cyan "  [Agnocast publisher]  no_rclcpp_pose_talker"
        if $PUB_STARTS_DA; then
            echo "                          └─ discovery_agent: YES"
        else
            echo "                          └─ discovery_agent: NO  (started by subscriber)"
        fi
    fi
fi

if $ROS2_PUB; then
    cyan "  [ROS 2 publisher]     pose_talker"
fi

if $AGNOCAST_SUB; then
    cyan "  [Agnocast subscriber] no_rclcpp_pose_listener  -> triggers R2A bridge"
    if $SUB_STARTS_DA; then
        echo "                          └─ discovery_agent: YES"
    else
        echo "                          └─ discovery_agent: NO  (shared with Agnocast publisher)"
    fi
fi

echo ""
echo "  Topic: /pose_chatter  (geometry_msgs/msg/Pose)"
echo ""
yellow "Starting in 1 second..."
sleep 1

# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------
LOG_DIR=$(mktemp -d)
PIDS=()

cleanup() {
    echo ""
    yellow "Stopping all background processes..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    rm -rf "$LOG_DIR"
    green "Done."
}
trap cleanup EXIT INT TERM

start_bg() {
    local label="$1"; shift
    "$@" > "$LOG_DIR/${label}.log" 2>&1 &
    local pid=$!
    PIDS+=("$pid")
    yellow "  [${label}] PID ${pid}: $*"
}

# ---------------------------------------------------------------------------
# Launch components
# ---------------------------------------------------------------------------
echo ""
yellow "=== Launching components ==="
echo ""

if $AGNOCAST_PUB; then
    if $AGNOCAST_PUB_ISOLATED; then
        start_bg "agnocast_pub_isolated" \
            sudo -E unshare --ipc bash -c \
                "source /opt/ros/${ROS_DISTRO}/setup.bash && source '${ROOT_DIR}/install/setup.bash' && \
                 ros2 launch agnocast_sample_application no_rclcpp_pose_talker.launch.xml"
    else
        if $PUB_STARTS_DA; then
            start_bg "agnocast_pub" \
                ros2 launch agnocast_sample_application no_rclcpp_pose_talker.launch.xml
        else
            start_bg "agnocast_pub" \
                ros2 launch agnocast_sample_application no_rclcpp_pose_talker.launch.xml \
                    discovery_agent:=false
        fi
    fi
fi

if $ROS2_PUB; then
    start_bg "ros2_pub" \
        ros2 launch agnocast_sample_application pose_talker.launch.xml
fi

if $AGNOCAST_SUB; then
    if $SUB_STARTS_DA; then
        start_bg "agnocast_sub" \
            ros2 launch agnocast_sample_application no_rclcpp_pose_listener.launch.xml
    else
        start_bg "agnocast_sub" \
            ros2 launch agnocast_sample_application no_rclcpp_pose_listener.launch.xml \
                discovery_agent:=false
    fi
fi

# ---------------------------------------------------------------------------
# Ready
# ---------------------------------------------------------------------------
echo ""
bold "=== Environment is up. Press Ctrl-C to stop all processes. ==="
echo ""
green "Active components:"
if $AGNOCAST_PUB; then
    if $AGNOCAST_PUB_ISOLATED; then
        green "  * Agnocast publisher  [isolated IPC namespace]"
    else
        green "  * Agnocast publisher"
    fi
fi
$ROS2_PUB     && green "  * ROS 2 publisher"
$AGNOCAST_SUB && green "  * Agnocast subscriber  (R2A bridge active)"
echo ""
yellow "Logs: $LOG_DIR/"
echo ""
yellow "Try one of these commands:"
echo "  ros2 topic hz_agnocast    /pose"
echo "  ros2 topic echo_agnocast  /pose"
echo "  ros2 topic delay_agnocast /pose"
echo ""

wait
