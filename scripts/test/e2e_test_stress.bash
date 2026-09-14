#!/bin/bash

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

set -e

PERCENTAGES=($(seq 5 5 95))   # You can change this
TIMEOUT_EACH_TEST_CASE_S="60" # You can change this

NUM_PERCENTAGES=${#PERCENTAGES[@]}
TIMEOUT=680s # based on the measurement time of e2e tests

cleanup() {
    echo "Stopping stress-ng..."
    pkill -P $$ # Kill all child processes
    exit 1
}

trap cleanup SIGINT

run-stress-ng() {
    if [ $1 -lt $NUM_PERCENTAGES ]; then
        echo "Run stress-ng with CPU load ${PERCENTAGES[$1]}%" | sudo tee /dev/kmsg
        stress-ng --cpu $(nproc) --cpu-load ${PERCENTAGES[$1]} --timeout $TIMEOUT &

    else
        index=$(($1 - NUM_PERCENTAGES))
        echo "Run stress-ng with VM ${PERCENTAGES[$index]}%" | sudo tee /dev/kmsg
        stress-ng --vm 1 --vm-bytes ${PERCENTAGES[$index]}% --timeout $TIMEOUT &
    fi
}

NUM_LOOP=$((NUM_PERCENTAGES * 2)) # cpu_load, vm
for i in $(seq 1 $NUM_LOOP); do
    echo "============================================================================" | sudo tee /dev/kmsg
    echo "============================ Outer Loop $i / $NUM_LOOP ============================" | sudo tee /dev/kmsg
    echo "============================================================================" | sudo tee /dev/kmsg

    run-stress-ng $(($i - 1))
    STRESS_TEST_TIMEOUT=$TIMEOUT_EACH_TEST_CASE_S ./scripts/test/e2e_test_1to1.bash
    STRESS_TEST_TIMEOUT=$TIMEOUT_EACH_TEST_CASE_S ./scripts/test/e2e_test_2to2.bash

    wait
done
