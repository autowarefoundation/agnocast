#!/bin/bash

# Parsing arguments
OPTIONS=$(getopt -o hsc --long help,single,continue -- "$@")
if [ $? -ne 0 ]; then
    echo "Invalid options provided"
    exit 1
fi
eval set -- "$OPTIONS"

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help      Show this help message"
    echo "  -s, --single    Run only one test case (Native mode, current config)"
    echo "  -c, --continue  Continue running tests even if one fails"
    exit 0
}

RUN_SINGLE=false
CONTINUE_ON_FAILURE=false
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
    --)
        shift
        break
        ;;
    *)
        break
        ;;
    esac
done

# Setup
rm -rf build/agnocast_e2e_test install/e2e_test
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select agnocast_e2e_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

LOWER_BRIDGE_MODE=$(echo "$AGNOCAST_BRIDGE_MODE" | tr '[:upper:]' '[:lower:]')
CURRENT_BRIDGE_DISPLAY=${LOWER_BRIDGE_MODE:-"standard (default)"}
echo "Bridge mode: $CURRENT_BRIDGE_DISPLAY" | sudo tee /dev/kmsg

# Run test
CONFIG_FILE=src/agnocast_e2e_test/test/config_test_2to2.yaml
show_config() {
    echo "----------------------------------------------" | sudo tee /dev/kmsg
    echo "Test Mode: $TEST_MODE" | sudo tee /dev/kmsg
    cat $CONFIG_FILE | sudo tee /dev/kmsg
    echo "----------------------------------------------" | sudo tee /dev/kmsg
}

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
else
    TEST_MODES=("agno2agno" "ros2agno" "agno2ros")
    if [ "$LOWER_BRIDGE_MODE" = "0" ] || [ "$LOWER_BRIDGE_MODE" = "off" ]; then
        TEST_MODES=("agno2agno")
    fi
    CONTAINER_LAYOUT=("PPSS" "PP|SS" "P|PSS" "PPS|S" "P|P|SS" "P|PS|S" "PP|S|S" "P|P|S|S")

    TOTAL_TESTS=$((${#TEST_MODES[@]} * ${#CONTAINER_LAYOUT[@]}))
    CURRENT_COUNT=0

    for mode in "${TEST_MODES[@]}"; do
        export TEST_MODE=$mode
        sed -i "s|^test_mode:.*|test_mode: \"$mode\"|" "$CONFIG_FILE"

        for layout in "${CONTAINER_LAYOUT[@]}"; do
            CURRENT_COUNT=$((CURRENT_COUNT + 1))

            IFS='|' read -ra containers <<<"$layout"
            for i in {0..3}; do # 4 containers
                if [ "$i" -lt "${#containers[@]}" ]; then
                    list=$(echo "${containers[$i]}" | sed 's/./\L&,/g; s/,$//') # PPSS" -> "p,p,s,s"
                else
                    list=""
                fi
                sed -i "s|^container$i:.*|container$i: [$list]|" "$CONFIG_FILE"
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
