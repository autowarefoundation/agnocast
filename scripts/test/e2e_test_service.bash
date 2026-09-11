#!/bin/bash
set -eo pipefail

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

# Signal handling: kill all descendant processes on exit
cleanup() {
    trap - SIGINT SIGTERM SIGHUP
    kill -- -$$ 2>/dev/null
    exit 130
}
trap cleanup SIGINT SIGTERM SIGHUP

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select agnocast_e2e_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

if launch_test src/agnocast_e2e_test/test/test_service.py; then
    echo "All tests passed!!"
else
    echo "Test failed" >&2
    exit 1
fi
