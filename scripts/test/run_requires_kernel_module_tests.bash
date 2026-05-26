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

# Pre-flight check: heaphook library must exist somewhere on COLCON_PREFIX_PATH.
# COLCON_PREFIX_PATH is colon-separated when multiple workspaces are sourced
# (e.g. an Autoware underlay plus this Agnocast overlay), so we have to walk
# the list instead of dereferencing it as a single path — otherwise the test
# fails out before colcon even runs, with an ERROR pointing at a path that
# literally contains a `:`.
HEAPHOOK_RELATIVE="agnocastlib/lib/libagnocast_heaphook.so"
HEAPHOOK_PATH=""
IFS=':' read -r -a _PREFIX_ENTRIES <<< "${COLCON_PREFIX_PATH}"
for _prefix in "${_PREFIX_ENTRIES[@]}"; do
  candidate="${_prefix}/${HEAPHOOK_RELATIVE}"
  if [ -f "${candidate}" ]; then
    HEAPHOOK_PATH="${candidate}"
    break
  fi
done
if [ -z "${HEAPHOOK_PATH}" ]; then
  echo "ERROR: ${HEAPHOOK_RELATIVE} not found under any COLCON_PREFIX_PATH entry:" >&2
  for _prefix in "${_PREFIX_ENTRIES[@]}"; do
    echo "  - ${_prefix}/${HEAPHOOK_RELATIVE}" >&2
  done
  echo "Build agnocast_heaphook first: cd agnocast_heaphook && cargo build --release && cp target/release/libagnocast_heaphook.so install/agnocastlib/lib/" >&2
  exit 1
fi

set -u
colcon test --event-handlers console_direct+ --return-code-on-test-failure --ctest-args -L requires_kernel_module
