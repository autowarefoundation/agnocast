#!/bin/bash

# Set up the agnocast CIE thread configurator.
# Reference: https://autowarefoundation.github.io/agnocast_doc/callback-isolated-executor/integration-guide/#step-2-set-up-the-thread-configurator
#
# Usage:
#   source /opt/ros/<distro>/setup.bash
#   source <workspace>/install/setup.bash
#   ./scripts/setup_thread_configurator.bash
#
# This script automates:
#   1. Restricting execution of thread_configurator_node to its owner and the
#      invoking user's primary group, then granting CAP_SYS_NICE (scheduling
#      syscalls) and CAP_DAC_OVERRIDE (writes to root-owned
#      /proc/irq/<N>/smp_affinity_list) to it.
#   2. Registering required library paths in /etc/ld.so.conf.d/agnocast-cie.conf.

set -euo pipefail

# --- Prerequisites ---------------------------------------------------------

if [ -z "${ROS_DISTRO:-}" ]; then
	echo "Error: ROS_DISTRO is not set. Please source your ROS 2 environment first:"
	echo "  source /opt/ros/<distro>/setup.bash"
	exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
	echo "Error: 'ros2' command not found. Please source your ROS 2 environment first:"
	echo "  source /opt/ros/\$ROS_DISTRO/setup.bash"
	exit 1
fi

if ! command -v dpkg-architecture >/dev/null 2>&1; then
	echo "Error: 'dpkg-architecture' not found. Install it with: sudo apt-get install dpkg-dev"
	exit 1
fi

if ! command -v setcap >/dev/null 2>&1; then
	echo "Error: 'setcap' not found. Install it with: sudo apt-get install libcap2-bin"
	exit 1
fi

if ! command -v ldconfig >/dev/null 2>&1; then
	echo "Error: 'ldconfig' not found. Install it with: sudo apt-get install libc-bin"
	exit 1
fi

if ! thread_configurator_prefix=$(ros2 pkg prefix agnocast_cie_thread_configurator 2>/dev/null); then
	echo "Error: Package 'agnocast_cie_thread_configurator' not found."
	echo "       Build the workspace and source install/setup.bash first."
	exit 1
fi

if ! config_msgs_prefix=$(ros2 pkg prefix agnocast_cie_config_msgs 2>/dev/null); then
	echo "Error: Package 'agnocast_cie_config_msgs' not found."
	echo "       Build the workspace and source install/setup.bash first."
	exit 1
fi

# --- Step 1: Restrict execution and grant capabilities ----------------------

run_group=$(id -gn "${SUDO_USER:-$(id -un)}")

echo "[1/2] Restrict thread_configurator_node to group '${run_group}' and grant CAP_SYS_NICE and CAP_DAC_OVERRIDE"

bin_path=$(readlink -f "${thread_configurator_prefix}/lib/agnocast_cie_thread_configurator/thread_configurator_node")

if [ ! -f "$bin_path" ]; then
	echo "  Error: binary not found at: $bin_path"
	exit 1
fi

# CAP_DAC_OVERRIDE bypasses file permission checks, so only the owner and the
# invoking user's group may execute the binary. chgrp clears file capabilities,
# so setcap runs last, and all three run every time to also fix older installs.
echo "  Target: $bin_path"
sudo chgrp "$run_group" "$bin_path"
sudo chmod 0750 "$bin_path"
sudo setcap "cap_sys_nice,cap_dac_override=eip" "$bin_path"
echo "  Done: $(stat -c '%A %U:%G' "$bin_path") $(getcap "$bin_path")"

# --- Step 2: Configure library paths ---------------------------------------

echo "[2/2] Configure library paths (/etc/ld.so.conf.d/agnocast-cie.conf)"

conf_file=/etc/ld.so.conf.d/agnocast-cie.conf
ros_lib_dir="/opt/ros/${ROS_DISTRO}/lib"
ros_multiarch_lib_dir="/opt/ros/${ROS_DISTRO}/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
config_msgs_lib_dir="${config_msgs_prefix}/lib"
# The binary links libagnocast_thread_configurator.so from its own package.
thread_configurator_lib_dir="${thread_configurator_prefix}/lib"
# Workspace directories come first, matching the overlay order of setup.bash.
expected_content=$(printf '%s\n' "$thread_configurator_lib_dir" "$config_msgs_lib_dir" "$ros_lib_dir" "$ros_multiarch_lib_dir")

if [ -f "$conf_file" ] && [ "$(cat "$conf_file")" = "$expected_content" ]; then
	echo "  Already configured ($conf_file). Skipping."
else
	echo "  Writing: $conf_file"
	echo "  Content:"
	sudo tee "$conf_file" <<<"$expected_content" | sed 's/^/    /'
	sudo ldconfig
	echo "  Done."
fi

echo "All steps completed."
