#!/bin/bash

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

# Signal handling: kill all descendant processes on exit
cleanup() {
    trap - SIGINT SIGTERM SIGHUP
    kill -- -$$ 2>/dev/null
    exec 3>&- 2>/dev/null
    rm -f "${COMMANDS_FILE:-}" "${WORKER_SCRIPT:-}" "${SLOT_FIFO:-}" 2>/dev/null
    rm -rf "${CONFIG_DIR:-}" 2>/dev/null
    exit 130
}
trap cleanup SIGINT SIGTERM SIGHUP

# Parsing arguments
OPTIONS=$(getopt -o hscbp: --long help,single,continue,bridge-only,parallel: -- "$@")
if [ $? -ne 0 ]; then
    echo "Invalid options provided"
    exit 1
fi
eval set -- "$OPTIONS"

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help         Show this help message"
    echo "  -s, --single       Run only one test case (Native mode, current config)"
    echo "  -c, --continue     Continue running tests even if one fails"
    echo "  -b, --bridge-only  Run only bridge tests (ros2agno, agno2ros) with AGNOCAST_BRIDGE_MODE=on"
    echo "  -p, --parallel N   Run tests in parallel with N workers (default: 1 = sequential)"
    exit 0
}

RUN_SINGLE=false
CONTINUE_ON_FAILURE=false
BRIDGE_ONLY=false
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
    -b | --bridge-only)
        BRIDGE_ONLY=true
        export AGNOCAST_BRIDGE_MODE=on
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

# Topic name prefix (can be overridden via E2E_TOPIC_PREFIX)
TOPIC_PREFIX=${E2E_TOPIC_PREFIX:-/test_topic}

# Setup
rm -rf build/agnocast_e2e_test install/e2e_test
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select agnocast_e2e_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

LOWER_BRIDGE_MODE=$(echo "$AGNOCAST_BRIDGE_MODE" | tr '[:upper:]' '[:lower:]')
CURRENT_BRIDGE_DISPLAY=${LOWER_BRIDGE_MODE:-"on (default)"}
echo "Bridge mode: $CURRENT_BRIDGE_DISPLAY" | sudo tee /dev/kmsg

if [ "$LOWER_BRIDGE_MODE" != "off" ] && [ "$LOWER_BRIDGE_MODE" != "0" ]; then
    echo "Bridge cleanup: checking agno_pbr_* processes" | sudo tee /dev/kmsg
    # Kill stale bridge processes directly by PID.
    mapfile -t AGNO_PBR_PIDS < <(ps -eo pid=,comm= | awk '$2 ~ /^agno_pbr_/ {print $1}')
    if [ "${#AGNO_PBR_PIDS[@]}" -gt 0 ]; then
        KILL_FAILED=false
        for pid in "${AGNO_PBR_PIDS[@]}"; do
            if kill "$pid" 2>/dev/null; then
                echo "Killed process: $pid" | sudo tee /dev/kmsg
            elif kill -0 "$pid" 2>/dev/null; then
                echo "ERROR: failed to kill process: $pid" | sudo tee /dev/kmsg
                KILL_FAILED=true
            else
                echo "Process already exited before kill: $pid" | sudo tee /dev/kmsg
            fi
        done
        if [ "$KILL_FAILED" = true ]; then
            exit 1
        fi
    fi

    sleep 1
fi

# Run test
CONFIG_FILE=src/agnocast_e2e_test/test/config_test_2to2.yaml
show_config() {
    echo "----------------------------------------------" | sudo tee /dev/kmsg
    echo "Test Mode: $TEST_MODE" | sudo tee /dev/kmsg
    cat $CONFIG_FILE | sudo tee /dev/kmsg
    echo "----------------------------------------------" | sudo tee /dev/kmsg
}

# Print the node list of the given container in a layout (e.g. "PP|SS" 1 -> "s,s")
container_nodes() {
    local layout=$1
    local index=$2
    local containers
    IFS='|' read -ra containers <<<"$layout"
    if [ "$index" -lt "${#containers[@]}" ]; then
        echo "${containers[$index]}" | sed 's/./\L&,/g; s/,$//'
    fi
}

# Generate test ID from parameters (ROS2 topic-name safe: alphanumerics and underscores only)
# Example: agno2ros_pp_ss
generate_test_id() {
    local mode=$1
    local layout=$2
    echo "${mode}_$(echo "$layout" | tr '|' '_' | tr '[:upper:]' '[:lower:]')"
}

TEST_MODES=("agno2agno" "ros2agno" "agno2ros")
if [ "$LOWER_BRIDGE_MODE" = "0" ] || [ "$LOWER_BRIDGE_MODE" = "off" ]; then
    TEST_MODES=("agno2agno")
fi
if [ "$BRIDGE_ONLY" = true ]; then
    TEST_MODES=("ros2agno" "agno2ros")
fi
CONTAINER_LAYOUT=("PPSS" "PP|SS" "P|PSS" "PPS|S" "P|P|SS" "P|PS|S" "PP|S|S" "P|P|S|S")

TOTAL_TESTS=$((${#TEST_MODES[@]} * ${#CONTAINER_LAYOUT[@]}))

FAILURE_COUNT=0
if [ "$RUN_SINGLE" = true ]; then
    CURRENT_YAML_MODE=$(grep "^test_mode:" "$CONFIG_FILE" | awk '{print $2}' | tr -d '"')
    export TEST_MODE=${CURRENT_YAML_MODE}
    show_config
    launch_test src/agnocast_e2e_test/test/test_2to2.py test_mode:="$TEST_MODE"
    if [ $? -ne 0 ]; then
        echo "Test failed" | sudo tee /dev/kmsg
        exit 1
    fi
elif [ "$PARALLEL" -gt 1 ]; then
    # ===== Parallel execution mode =====
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

    LOG_DIR="/tmp/e2e_2to2_logs"
    rm -rf "$LOG_DIR"
    mkdir -p "$LOG_DIR"

    FAILED_TESTS_FILE=$(mktemp)

    # Generate a config file and a command line for every test case
    CONFIG_DIR=$(mktemp -d /tmp/e2e_2to2_configs_XXXXXX)
    COMMANDS_FILE=$(mktemp)
    COUNT=0
    for mode in "${TEST_MODES[@]}"; do
        for layout in "${CONTAINER_LAYOUT[@]}"; do
            COUNT=$((COUNT + 1))
            TEST_ID=$(generate_test_id "$mode" "$layout")
            CONFIG_TMP="${CONFIG_DIR}/${COUNT}_${TEST_ID}.yaml"
            {
                echo "test_mode: \"$mode\""
                for i in {0..3}; do # 4 containers
                    echo "container$i: [$(container_nodes "$layout" "$i")]"
                done
            } > "$CONFIG_TMP"
            echo "$COUNT $TEST_ID $mode $CONFIG_TMP" >> "$COMMANDS_FILE"
        done
    done

    echo "Running $TOTAL_TESTS tests with $PARALLEL parallel workers..." | sudo tee /dev/kmsg

    # Create a FIFO-based slot pool to reuse ROS_DOMAIN_IDs within the safe range (0-101).
    # Each worker acquires a slot (0 to PARALLEL-1) from the FIFO before running, and
    # returns it when done. This keeps ROS_DOMAIN_ID = DOMAIN_ID_BASE + slot, avoiding
    # high domain IDs whose DDS ports collide with OS ephemeral ports.
    SLOT_FIFO=$(mktemp -u /tmp/e2e_2to2_slot_fifo_XXXXXX)
    mkfifo "$SLOT_FIFO"
    exec 3<>"$SLOT_FIFO"
    for ((i = 0; i < PARALLEL; i++)); do
        echo "$i" >&3
    done

    # Export variables needed by worker
    export TOPIC_PREFIX LOG_DIR FAILED_TESTS_FILE DOMAIN_ID_BASE TOTAL_TESTS

    # Create worker script
    WORKER_SCRIPT=$(mktemp /tmp/e2e_2to2_worker_XXXXXX.sh)
    cat > "$WORKER_SCRIPT" << 'WORKER_EOF'
#!/bin/bash
COUNT=$1
TEST_ID=$2
export TEST_MODE=$3
CONFIG_TMP=$4

# Acquire a slot from the FIFO pool for DDS domain isolation
read -u 3 SLOT
export ROS_DOMAIN_ID=$((DOMAIN_ID_BASE + SLOT))

echo "[START  $COUNT/$TOTAL_TESTS] $TEST_ID (domain=$ROS_DOMAIN_ID)" | sudo tee /dev/kmsg

LOG_FILE="${LOG_DIR}/${COUNT}_${TEST_ID}.log"
E2E_TOPIC_NAME="${TOPIC_PREFIX}_${TEST_ID}" E2E_CONFIG_PATH="$CONFIG_TMP" launch_test src/agnocast_e2e_test/test/test_2to2.py test_mode:="$TEST_MODE" > "$LOG_FILE" 2>&1
EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "[FAILED $COUNT/$TOTAL_TESTS] $TEST_ID" | sudo tee /dev/kmsg >&2
    echo "${COUNT}_${TEST_ID}" >> "$FAILED_TESTS_FILE"
else
    echo "[PASSED $COUNT/$TOTAL_TESTS] $TEST_ID" | sudo tee /dev/kmsg
fi

# Add a 2s safety margin to account for bridge cleanup (expected max ~1s).
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
    rm -rf "$CONFIG_DIR"

    # Summary
    if [ -s "$FAILED_TESTS_FILE" ]; then
        FAILURE_COUNT=$(wc -l < "$FAILED_TESTS_FILE")
        echo "" | sudo tee /dev/kmsg
        echo "====== FAILURE SUMMARY ======" | sudo tee /dev/kmsg
        while IFS= read -r test_id; do
            echo "  FAILED: $test_id (log: ${LOG_DIR}/${test_id}.log)" | sudo tee /dev/kmsg
        done < <(sort -n "$FAILED_TESTS_FILE")
        echo "=============================" | sudo tee /dev/kmsg
    fi
    rm -f "$FAILED_TESTS_FILE"
else
    # ===== Sequential execution mode (original behavior) =====
    CURRENT_COUNT=0

    for mode in "${TEST_MODES[@]}"; do
        export TEST_MODE=$mode
        sed -i "s|^test_mode:.*|test_mode: \"$mode\"|" "$CONFIG_FILE"

        for layout in "${CONTAINER_LAYOUT[@]}"; do
            CURRENT_COUNT=$((CURRENT_COUNT + 1))

            for i in {0..3}; do # 4 containers
                sed -i "s|^container$i:.*|container$i: [$(container_nodes "$layout" "$i")]|" "$CONFIG_FILE"
            done

            echo "====================== $CURRENT_COUNT / $TOTAL_TESTS (Mode: $mode) ======================" | sudo tee /dev/kmsg
            show_config
            launch_test src/agnocast_e2e_test/test/test_2to2.py test_mode:=$mode

            if [ $? -ne 0 ]; then
                echo "Test failed at Mode: $mode, Layout: $layout" | sudo tee /dev/kmsg
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
fi

if [ "$FAILURE_COUNT" -gt 0 ]; then
    echo "$FAILURE_COUNT / $TOTAL_TESTS tests failed" | sudo tee /dev/kmsg
    exit 1
else

    echo "All tests passed!!" | sudo tee /dev/kmsg
fi

# Reset config file
git checkout -- "$CONFIG_FILE"
