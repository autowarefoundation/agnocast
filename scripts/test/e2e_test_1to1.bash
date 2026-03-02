#!/bin/bash

# Signal handling: kill all descendant processes on exit
cleanup() {
    trap - SIGINT SIGTERM SIGHUP
    kill -- -$$ 2>/dev/null
    exec 3>&- 2>/dev/null
    rm -f "${COMMANDS_FILE:-}" "${WORKER_SCRIPT:-}" "${SLOT_FIFO:-}" 2>/dev/null
    exit 130
}
trap cleanup SIGINT SIGTERM SIGHUP

# Parsing arguments
OPTIONS=$(getopt -o hscp: --long help,single,continue,parallel: -- "$@")
if [ $? -ne 0 ]; then
    echo "Invalid options provided"
    exit 1
fi
eval set -- "$OPTIONS"

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help         Show this help message"
    echo "  -s, --single       Run only one test case using current config file"
    echo "  -c, --continue     Continue running tests even if one fails"
    echo "  -p, --parallel N   Run tests in parallel with N workers (default: 1 = sequential)"
    exit 0
}

RUN_SINGLE=false
CONTINUE_ON_FAILURE=false
PARALLEL=1
while true; do
    case "$1" in
    -h | --help)
        usage
        ;;
    -s | --single)
        RUN_SINGLE=true
        shift
        ;;
    -c | --continue)
        CONTINUE_ON_FAILURE=true
        shift
        ;;
    -p | --parallel)
        PARALLEL=$2
        shift 2
        ;;
    --)
        shift
        break
        ;;
    *)
        break
        ;;
    esac
done

if ! [[ "$PARALLEL" =~ ^[0-9]+$ ]] || [ "$PARALLEL" -lt 1 ]; then
    echo "Error: --parallel must be a positive integer"
    exit 1
fi

# Check bridge mode
LOWER_BRIDGE_MODE=$(echo "$AGNOCAST_BRIDGE_MODE" | tr '[:upper:]' '[:lower:]')
if [[ "$LOWER_BRIDGE_MODE" =~ ^(0|off)$ ]]; then
    BRIDGE_OFF=true
else
    BRIDGE_OFF=false
fi

# Topic name prefix (can be overridden via E2E_TOPIC_PREFIX)
TOPIC_PREFIX=${E2E_TOPIC_PREFIX:-/test_topic}

# Setup
rm -rf build/agnocast_e2e_test install/e2e_test
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select agnocast_e2e_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

CURRENT_BRIDGE_DISPLAY=${LOWER_BRIDGE_MODE:-"standard (default)"}
echo "Bridge mode: $CURRENT_BRIDGE_DISPLAY" | sudo tee /dev/kmsg

# Run test
CONFIG_FILE="src/agnocast_e2e_test/test/config_test_1to1.yaml"
show_config() {
    echo "----------------------------------------------" | sudo tee /dev/kmsg
    if [ "$BRIDGE_OFF" = true ]; then
        grep -v "use_agnocast_pub" "$CONFIG_FILE" | sudo tee /dev/kmsg
    else
        cat "$CONFIG_FILE" | sudo tee /dev/kmsg
    fi
    echo "----------------------------------------------" | sudo tee /dev/kmsg
}

# Generate test ID from parameters (ROS2 topic-name safe: alphanumerics and underscores only)
# Example: agnopub_pubfirst_pub_depth1_tl_sub_depth1_tl_takesub
generate_test_id() {
    local use_agnocast_pub=$1
    local launch_pub_before_sub=$2
    local pub_qos_depth=$3
    local pub_transient_local=$4
    local sub_qos_depth=$5
    local sub_transient_local=$6
    local use_take_sub=$7

    local pub_type="agnopub"
    [ "$use_agnocast_pub" = "false" ] && pub_type="ros2pub"

    local launch_order="pubfirst"
    [ "$launch_pub_before_sub" = "false" ] && launch_order="subfirst"

    local pub_tl="tl"
    [ "$pub_transient_local" = "false" ] && pub_tl="volatile"

    local sub_tl="tl"
    [ "$sub_transient_local" = "false" ] && sub_tl="volatile"

    local sub_type="callbacksub"
    [ "$use_take_sub" = "true" ] && sub_type="takesub"

    echo "${pub_type}_${launch_order}_pub_depth${pub_qos_depth}_${pub_tl}_sub_depth${sub_qos_depth}_${sub_tl}_${sub_type}"
}

FAILURE_COUNT=0
if [ "$RUN_SINGLE" = true ]; then
    show_config
    launch_test src/agnocast_e2e_test/test/test_1to1.py
    if [ $? -ne 0 ]; then
        echo "Test failed" | sudo tee /dev/kmsg
        exit 1
    fi
elif [ "$PARALLEL" -gt 1 ]; then
    # ===== Parallel execution mode =====
    # When bridge is OFF, only test Agnocast publisher
    if [ "$BRIDGE_OFF" = true ]; then
        USE_AGNOCAST_PUB=(true)
    else
        USE_AGNOCAST_PUB=(true false)
    fi

    LAUNCH_PUB_BEFORE_SUB=(true false)
    PUB_QOS_DEPTH=(1 10)
    PUB_TRANSIENT_LOCAL=(true false)
    SUB_QOS_DEPTH=(1 10)
    SUB_TRANSIENT_LOCAL=(true false)
    USE_TAKE_SUB=(true false)

    TOTAL_TESTS=$((${#USE_AGNOCAST_PUB[@]} * ${#LAUNCH_PUB_BEFORE_SUB[@]} * ${#PUB_QOS_DEPTH[@]} * ${#PUB_TRANSIENT_LOCAL[@]} * ${#SUB_QOS_DEPTH[@]} * ${#SUB_TRANSIENT_LOCAL[@]} * ${#USE_TAKE_SUB[@]}))

    LOG_DIR="/tmp/e2e_1to1_logs"
    rm -rf "$LOG_DIR"
    mkdir -p "$LOG_DIR"

    FAILED_TESTS_FILE=$(mktemp)

    # Generate all test commands (with sequential worker number for ROS_DOMAIN_ID)
    COMMANDS_FILE=$(mktemp)
    WORKER_NUM=0
    # Max parallel workers = 101 - DOMAIN_ID_BASE + 1 (default: 92 with base=10).
    # This is because DDS domain IDs must be in 0-101; higher IDs cause port collisions
    # with OS ephemeral ports. Each worker needs a unique domain ID starting from DOMAIN_ID_BASE.
    DOMAIN_ID_BASE=${ROS_DOMAIN_ID_BASE:-10}
    MAX_PARALLEL=$((101 - DOMAIN_ID_BASE + 1))
    if [ "$PARALLEL" -gt "$MAX_PARALLEL" ]; then
        echo "Error: -p $PARALLEL exceeds max parallel workers ($MAX_PARALLEL) for DOMAIN_ID_BASE=$DOMAIN_ID_BASE." | sudo tee /dev/kmsg
        echo "DDS domain IDs must be in 0-101. Reduce -p or lower ROS_DOMAIN_ID_BASE." | sudo tee /dev/kmsg
        exit 1
    fi
    for use_agnocast_pub in ${USE_AGNOCAST_PUB[@]}; do
        for launch_pub_before_sub in ${LAUNCH_PUB_BEFORE_SUB[@]}; do
            for pub_qos_depth in ${PUB_QOS_DEPTH[@]}; do
                for pub_transient_local in ${PUB_TRANSIENT_LOCAL[@]}; do
                    for sub_qos_depth in ${SUB_QOS_DEPTH[@]}; do
                        for sub_transient_local in ${SUB_TRANSIENT_LOCAL[@]}; do
                            for use_take_sub in ${USE_TAKE_SUB[@]}; do
                                TEST_ID=$(generate_test_id "$use_agnocast_pub" "$launch_pub_before_sub" "$pub_qos_depth" "$pub_transient_local" "$sub_qos_depth" "$sub_transient_local" "$use_take_sub")
                                echo "$WORKER_NUM $TEST_ID $use_agnocast_pub $launch_pub_before_sub $pub_qos_depth $pub_transient_local $sub_qos_depth $sub_transient_local $use_take_sub" >> "$COMMANDS_FILE"
                                WORKER_NUM=$((WORKER_NUM + 1))
                            done
                        done
                    done
                done
            done
        done
    done

    echo "Running $TOTAL_TESTS tests with $PARALLEL parallel workers..." | sudo tee /dev/kmsg

    # Create a FIFO-based slot pool to reuse ROS_DOMAIN_IDs within the safe range (0-101).
    # Each worker acquires a slot (0 to PARALLEL-1) from the FIFO before running, and
    # returns it when done. This keeps ROS_DOMAIN_ID = DOMAIN_ID_BASE + slot, avoiding
    # high domain IDs whose DDS ports collide with OS ephemeral ports.
    SLOT_FIFO=$(mktemp -u /tmp/e2e_1to1_slot_fifo_XXXXXX)
    mkfifo "$SLOT_FIFO"
    exec 3<>"$SLOT_FIFO"
    for ((i = 0; i < PARALLEL; i++)); do
        echo "$i" >&3
    done

    # Export variables needed by worker
    export TOPIC_PREFIX LOG_DIR FAILED_TESTS_FILE DOMAIN_ID_BASE TOTAL_TESTS SLOT_FIFO

    # Create worker script
    WORKER_SCRIPT=$(mktemp /tmp/e2e_1to1_worker_XXXXXX.sh)
    cat > "$WORKER_SCRIPT" << 'WORKER_EOF'
#!/bin/bash
WORKER_NUM=$1
TEST_ID=$2
use_agnocast_pub=$3
launch_pub_before_sub=$4
pub_qos_depth=$5
pub_transient_local=$6
sub_qos_depth=$7
sub_transient_local=$8
use_take_sub=$9

COUNT=$((WORKER_NUM + 1))

# Acquire a slot from the FIFO pool for DDS domain isolation
read -u 3 SLOT
export ROS_DOMAIN_ID=$((DOMAIN_ID_BASE + SLOT))

echo "[START  $COUNT/$TOTAL_TESTS] $TEST_ID (domain=$ROS_DOMAIN_ID)" | sudo tee /dev/kmsg

CONFIG_TMP="/tmp/e2e_1to1_${COUNT}_${TEST_ID}.yaml"
cat > "$CONFIG_TMP" << CFG_EOF
use_agnocast_pub: $use_agnocast_pub
launch_pub_before_sub: $launch_pub_before_sub
pub_qos_depth: $pub_qos_depth
pub_transient_local: $pub_transient_local
sub_qos_depth: $sub_qos_depth
sub_transient_local: $sub_transient_local
use_take_sub: $use_take_sub
CFG_EOF

LOG_FILE="${LOG_DIR}/${COUNT}_${TEST_ID}.log"
E2E_TOPIC_NAME="${TOPIC_PREFIX}_${TEST_ID}" E2E_CONFIG_PATH="$CONFIG_TMP" launch_test src/agnocast_e2e_test/test/test_1to1.py > "$LOG_FILE" 2>&1
EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "[FAILED $COUNT/$TOTAL_TESTS] $TEST_ID" | sudo tee /dev/kmsg >&2
    echo "${COUNT}_${TEST_ID}" >> "$FAILED_TESTS_FILE"
else
    echo "[PASSED $COUNT/$TOTAL_TESTS] $TEST_ID" | sudo tee /dev/kmsg
fi

rm -f "$CONFIG_TMP"
sleep 2

# Return the slot to the pool
echo "$SLOT" >&3
WORKER_EOF
    chmod +x "$WORKER_SCRIPT"

    # Run tests in parallel with job control
    while IFS= read -r line; do
        # Stop launching new tests if any test has failed (unless -c is specified)
        if [ "$CONTINUE_ON_FAILURE" != "true" ] && [ -s "$FAILED_TESTS_FILE" ]; then
            echo "Failure detected, stopping new test launches..." | sudo tee /dev/kmsg
            break
        fi
        bash "$WORKER_SCRIPT" $line &
        while [ "$(jobs -rp | wc -l)" -ge "$PARALLEL" ]; do
            wait -n
        done
    done < "$COMMANDS_FILE"
    wait

    exec 3>&-
    rm -f "$COMMANDS_FILE" "$WORKER_SCRIPT" "$SLOT_FIFO"

    # Summary
    if [ -s "$FAILED_TESTS_FILE" ]; then
        FAILURE_COUNT=$(wc -l < "$FAILED_TESTS_FILE")
        echo "" | sudo tee /dev/kmsg
        echo "====== FAILURE SUMMARY ======" | sudo tee /dev/kmsg
        echo "$FAILURE_COUNT / $TOTAL_TESTS tests failed:" | sudo tee /dev/kmsg
        while IFS= read -r test_id; do
            echo "  FAILED: $test_id (log: ${LOG_DIR}/${test_id}.log)" | sudo tee /dev/kmsg
        done < <(sort -n "$FAILED_TESTS_FILE")
        echo "=============================" | sudo tee /dev/kmsg
        rm -f "$FAILED_TESTS_FILE"
        exit 1
    else
        echo "All $TOTAL_TESTS tests passed!!" | sudo tee /dev/kmsg
        rm -f "$FAILED_TESTS_FILE"
    fi
else
    # ===== Sequential execution mode (original behavior) =====
    # When bridge is OFF, only test Agnocast publisher
    if [ "$BRIDGE_OFF" = true ]; then
        USE_AGNOCAST_PUB=(true)
    else
        USE_AGNOCAST_PUB=(true false)
    fi

    LAUNCH_PUB_BEFORE_SUB=(true false)
    PUB_QOS_DEPTH=(1 10)
    PUB_TRANSIENT_LOCAL=(true false)
    SUB_QOS_DEPTH=(1 10)
    SUB_TRANSIENT_LOCAL=(true false)
    USE_TAKE_SUB=(true false)

    COUNT=0
    TOTAL_TESTS=$((${#USE_AGNOCAST_PUB[@]} * ${#LAUNCH_PUB_BEFORE_SUB[@]} * ${#PUB_QOS_DEPTH[@]} * ${#PUB_TRANSIENT_LOCAL[@]} * ${#SUB_QOS_DEPTH[@]} * ${#SUB_TRANSIENT_LOCAL[@]} * ${#USE_TAKE_SUB[@]}))

    for use_agnocast_pub in ${USE_AGNOCAST_PUB[@]}; do
        for launch_pub_before_sub in ${LAUNCH_PUB_BEFORE_SUB[@]}; do
            for pub_qos_depth in ${PUB_QOS_DEPTH[@]}; do
                for pub_transient_local in ${PUB_TRANSIENT_LOCAL[@]}; do
                    for sub_qos_depth in ${SUB_QOS_DEPTH[@]}; do
                        for sub_transient_local in ${SUB_TRANSIENT_LOCAL[@]}; do
                            for use_take_sub in ${USE_TAKE_SUB[@]}; do
                                COUNT=$((COUNT + 1))
                                sed -i "s/use_agnocast_pub:.*/use_agnocast_pub: $use_agnocast_pub/g" $CONFIG_FILE
                                sed -i "s/launch_pub_before_sub:.*/launch_pub_before_sub: $launch_pub_before_sub/g" $CONFIG_FILE
                                sed -i "s/pub_qos_depth:.*/pub_qos_depth: $pub_qos_depth/g" $CONFIG_FILE
                                sed -i "s/pub_transient_local:.*/pub_transient_local: $pub_transient_local/g" $CONFIG_FILE
                                sed -i "s/sub_qos_depth:.*/sub_qos_depth: $sub_qos_depth/g" $CONFIG_FILE
                                sed -i "s/sub_transient_local:.*/sub_transient_local: $sub_transient_local/g" $CONFIG_FILE
                                sed -i "s/use_take_sub:.*/use_take_sub: $use_take_sub/g" $CONFIG_FILE
                                echo "====================== $COUNT / $TOTAL_TESTS ======================" | sudo tee /dev/kmsg
                                show_config
                                launch_test src/agnocast_e2e_test/test/test_1to1.py

                                if [ $? -ne 0 ]; then
                                    echo "Test failed." | sudo tee /dev/kmsg
                                    if [ "$CONTINUE_ON_FAILURE" = true ]; then
                                        FAILURE_COUNT=$((FAILURE_COUNT + 1))
                                    else
                                        exit 1
                                    fi
                                fi
                                # Add a 2s safety margin to account for bridge cleanup (expected max ~1s).
                                sleep 2
                            done
                        done
                    done
                done
            done
        done
    done

    if [ "$FAILURE_COUNT" -gt 0 ]; then
        echo "$FAILURE_COUNT / $COUNT tests failed" | sudo tee /dev/kmsg
        exit 1
    else
        echo "All tests passed!!" | sudo tee /dev/kmsg
    fi

    # Reset config file
    git checkout -- "$CONFIG_FILE"
fi
