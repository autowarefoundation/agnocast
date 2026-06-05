#!/bin/bash
# Manual test for ros2 bag record_agnocast topic-filtering rules.
#
# Starts three Agnocast talkers in the background, then runs
# "ros2 bag record_agnocast --log-level debug" with various filter options
# and checks which A2R bridge activations appear in the log output.
#
# Topics under test
#   /my_topic       – run_talker.bash (minimal_publisher.cpp)
#   /pose_chatter   – run_no_rclcpp_pose_talker.bash (no_rclcpp_pose_talker.cpp)
#   /string_chatter – run_no_rclcpp_string_talker.bash (no_rclcpp_string_talker.cpp)
#
# Usage (from workspace root):
#   bash scripts/test_bag_record_agnocast_filter.bash

set -euo pipefail

# ─── Colors ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# ─── Constants ────────────────────────────────────────────────────────────────
TOPIC_MY="/my_topic"
TOPIC_POSE="/pose_chatter"
TOPIC_STRING="/string_chatter"

# Seconds to run each ros2 bag record_agnocast invocation before killing it.
# Must be long enough for the discovery gossip to be received (~2-3 s typical).
RECORD_SECS=3

# Seconds to wait after starting talkers before running tests.
INIT_WAIT=1

# ─── State ────────────────────────────────────────────────────────────────────
PASS=0
FAIL=0
TEST_NUM=0
WORK_DIR=$(mktemp -d /tmp/bag_filter_test_XXXXXX)
TALKER_PGIDS=()
RECORD_PID=""  # PID of the current ros2 bag record_agnocast process (for cleanup)

# ─── Logging helpers ──────────────────────────────────────────────────────────
log_info() { echo -e "${BLUE}[INFO]${NC}  $*"; }
log_pass() { echo -e "${GREEN}[PASS]${NC}  $*"; PASS=$((PASS + 1)); }
log_fail() { echo -e "${RED}[FAIL]${NC}  $*"; FAIL=$((FAIL + 1)); }
log_warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }

# ─── Cleanup ──────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    # Kill any still-running recording process first.
    [[ -n "${RECORD_PID:-}" ]] && kill -KILL "$RECORD_PID" 2>/dev/null || true
    log_info "Stopping talkers..."
    for pgid in "${TALKER_PGIDS[@]}"; do
        kill -TERM -"$pgid" 2>/dev/null || true
    done
    sleep 1
    for pgid in "${TALKER_PGIDS[@]}"; do
        kill -KILL -"$pgid" 2>/dev/null || true
    done
    rm -rf "$WORK_DIR"
    echo ""
    echo -e "${BOLD}════ Results: ${GREEN}$PASS passed${NC}${BOLD} / ${RED}$FAIL failed${NC}${BOLD} (total $((PASS + FAIL))) ════${NC}"
    [[ $FAIL -eq 0 ]] \
        && echo -e "${GREEN}All tests passed!${NC}" \
        || echo -e "${RED}Some tests FAILED.${NC}"
}
trap cleanup EXIT
trap 'echo; log_info "Interrupted by user"; exit 130' INT

# ─── Sanity check ─────────────────────────────────────────────────────────────
if [[ ! -f install/setup.bash ]]; then
    echo "ERROR: install/setup.bash not found." >&2
    exit 1
fi
set +u
source install/setup.bash
set -u

# ─── Start talkers ────────────────────────────────────────────────────────────
log_info "Starting Agnocast talkers in the background..."
for script in \
    scripts/sample_application/run_talker.bash \
    scripts/sample_application/run_no_rclcpp_pose_talker.bash \
    scripts/sample_application/run_no_rclcpp_string_talker.bash
do
    setsid bash "$script" \
        > "$WORK_DIR/$(basename "$script" .bash).log" 2>&1 &
    TALKER_PGIDS+=($!)
    log_info "  Started $script (pgid=$!)"
done

log_info "Waiting ${INIT_WAIT}s for talkers to initialize and be discoverable..."
sleep "$INIT_WAIT"

# ─── Test runner ──────────────────────────────────────────────────────────────
# Usage: run_test <name> [OPTIONS...] -- [EXPECTED_TOPICS...]
#
#   OPTIONS         : arguments passed verbatim to ros2 bag record_agnocast
#                     (after --log-level debug and before -o <bag_dir>)
#   --              : separates options from expected topics
#   EXPECTED_TOPICS : zero or more topic names that must appear in the
#                     "A2R bridge requested" log output (and nothing else)
#
# The test passes when the set of activated topics equals the expected set
# exactly (no missing, no extra topics).
run_test() {
    local name="$1"; shift

    # Split args into OPTIONS and EXPECTED_TOPICS at "--"
    local -a opts=()
    local -a expected=()
    local sep_seen=false
    for arg in "$@"; do
        if [[ "$arg" == "--" ]]; then
            sep_seen=true
        elif $sep_seen; then
            expected+=("$arg")
        else
            opts+=("$arg")
        fi
    done

    TEST_NUM=$((TEST_NUM + 1))
    local output="$WORK_DIR/test${TEST_NUM}.txt"
    local bag_dir="$WORK_DIR/bag${TEST_NUM}"

    echo ""
    echo -e "${BOLD}── Test $TEST_NUM: $name ──${NC}"
    if [[ ${#opts[@]} -gt 0 ]]; then
        log_info "  Options  : ${opts[*]}"
    else
        log_info "  Options  : (none)"
    fi
    if [[ ${#expected[@]} -gt 0 ]]; then
        log_info "  Expected : ${expected[*]}"
    else
        log_info "  Expected : (none - no A2R bridges should be activated)"
    fi

    # Run in background, then send SIGINT after RECORD_SECS.
    # SIGINT (not SIGTERM) lets Python raise KeyboardInterrupt → clean shutdown
    # → stdout/stderr buffers are flushed before the process exits.
    ros2 bag record_agnocast --log-level debug "${opts[@]}" -o "$bag_dir" \
        &>"$output" &
    RECORD_PID=$!
    sleep "$RECORD_SECS"
    kill -INT "$RECORD_PID" 2>/dev/null || true
    # Wait up to 3 s for clean exit, then force-kill.
    local _i
    for _i in 1 2 3 4 5 6; do
        kill -0 "$RECORD_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -KILL "$RECORD_PID" 2>/dev/null || true
    wait "$RECORD_PID" 2>/dev/null || true
    RECORD_PID=""

    # Parse the set of activated topics from the log.
    local -a activated=()
    while IFS= read -r topic; do
        [[ -n "$topic" ]] && activated+=("$topic")
    done < <(
        grep "A2R bridge requested" "$output" \
            | sed "s/.*A2R bridge requested: '\([^']*\)'.*/\1/" \
            | sort -u
    )

    if [[ ${#activated[@]} -gt 0 ]]; then
        log_info "  Activated: ${activated[*]}"
    else
        log_info "  Activated: (none)"
    fi

    local ok=true

    # Every expected topic must appear in the activated set.
    for t in "${expected[@]}"; do
        local found=false
        for a in "${activated[@]}"; do
            [[ "$a" == "$t" ]] && found=true && break
        done
        if $found; then
            echo -e "  ${GREEN}✓${NC} '$t' activated (expected)"
        else
            echo -e "  ${RED}✗${NC} '$t' NOT activated (expected)"
            ok=false
        fi
    done

    # No unexpected topic may appear in the activated set.
    for a in "${activated[@]}"; do
        local is_expected=false
        for t in "${expected[@]}"; do
            [[ "$a" == "$t" ]] && is_expected=true && break
        done
        if ! $is_expected; then
            echo -e "  ${RED}✗${NC} '$a' activated (NOT expected)"
            ok=false
        fi
    done

    # Edge case: expected nothing and nothing was activated – explicit success.
    if [[ ${#expected[@]} -eq 0 && ${#activated[@]} -eq 0 ]]; then
        echo -e "  ${GREEN}✓${NC} No topics activated (expected)"
    fi

    if $ok; then
        log_pass "Test $TEST_NUM PASSED: $name"
    else
        log_fail "Test $TEST_NUM FAILED: $name"
    fi
}

# ─── Test cases ───────────────────────────────────────────────────────────────

# ── 1. -a  →  all three topics ───────────────────────────────────────────────
run_test "record all (-a)" \
    -a -- "$TOPIC_MY" "$TOPIC_POSE" "$TOPIC_STRING"

# ── 2. -e '/my'  →  /my_topic only ──────────────────────────────────────────
run_test "regex '/my' → /my_topic only" \
    -e /my -- "$TOPIC_MY"

# ── 3. -e '/pose_chatter'  →  /pose_chatter only ────────────────────────────
run_test "regex '/pose_chatter' → /pose_chatter only" \
    -e /pose_chatter -- "$TOPIC_POSE"

# ── 4. -e '/string_chatter'  →  /string_chatter only ────────────────────────
run_test "regex '/string_chatter' → /string_chatter only" \
    -e /string_chatter -- "$TOPIC_STRING"

# ── 5. -e '/.*chatter'  →  both chatter topics ───────────────────────────────
run_test "regex '/.*chatter' → both chatter topics" \
    -e /.*chatter -- "$TOPIC_POSE" "$TOPIC_STRING"

# ── 6. -e matching all three via alternation ─────────────────────────────────
run_test "regex '/my|chatter' → all 3 topics" \
    -e '/my|chatter' -- "$TOPIC_MY" "$TOPIC_POSE" "$TOPIC_STRING"

# ── 7. -e matching nothing  →  no bridges activated ─────────────────────────
run_test "regex '/nonexistent_xyz' → no topics" \
    -e /nonexistent_xyz --

# ── 8. positional: single topic ──────────────────────────────────────────────
run_test "positional '/my_topic'" \
    /my_topic -- "$TOPIC_MY"

# ── 9. positional: two topics ────────────────────────────────────────────────
run_test "positional '/my_topic /pose_chatter'" \
    /my_topic /pose_chatter -- "$TOPIC_MY" "$TOPIC_POSE"

# ── 10. --topics flag: single topic (Jazzy+ only; --topics does not exist in Humble) ──
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    run_test "--topics /string_chatter" \
        --topics /string_chatter -- "$TOPIC_STRING"
else
    log_warn "Test 10 skipped: --topics is not supported on ROS_DISTRO=humble"
fi
