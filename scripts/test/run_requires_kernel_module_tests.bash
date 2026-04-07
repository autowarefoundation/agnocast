#!/bin/bash
set -eo pipefail

# Pre-flight check: kernel module must be loaded before anything else
if ! grep -q "^agnocast " /proc/modules; then
  echo "ERROR: agnocast kernel module is not loaded." >&2
  echo "Load it first: sudo modprobe agnocast" >&2
  exit 1
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --packages-up-to agnocast_e2e_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

# Pre-flight check: heaphook library must exist
HEAPHOOK_PATH="${COLCON_PREFIX_PATH}/agnocastlib/lib/libagnocast_heaphook.so"
if [ ! -f "${HEAPHOOK_PATH}" ]; then
  echo "ERROR: ${HEAPHOOK_PATH} not found." >&2
  echo "Build agnocast_heaphook first: cd agnocast_heaphook && cargo build --release && cp target/release/libagnocast_heaphook.so ${COLCON_PREFIX_PATH}/lib/" >&2
  exit 1
fi

set -u
colcon test --event-handlers console_direct+ --return-code-on-test-failure --ctest-args -L requires_kernel_module
