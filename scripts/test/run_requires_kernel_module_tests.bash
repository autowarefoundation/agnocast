#!/bin/bash
set -eo pipefail

source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --packages-up-to agnocast_e2e_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

set -u
colcon test --event-handlers console_direct+ --return-code-on-test-failure --ctest-args -L requires_kernel_module
