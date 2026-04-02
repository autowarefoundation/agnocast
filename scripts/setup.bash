#!/bin/bash

set -e

# Validate ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
  echo "Error: ROS_DISTRO is not set. Please source your ROS 2 environment first:"
  echo "  source /opt/ros/<distro>/setup.bash"
  exit 1
fi

# Check if apt-installed agnocast packages exist (conflict with source build)
conflicting_pkgs=()
for pkg in $(dpkg -l 'agnocast-heaphook-v*' 'agnocast-kmod-v*' 2>/dev/null | grep '^ii' | awk '{print $2}'); do
  conflicting_pkgs+=("$pkg")
done

# Check for apt-installed ROS agnocast packages
ros_pkg_names=(
  agnocast
  agnocast-cie-config-msgs
  agnocast-cie-thread-configurator
  agnocast-components
  agnocast-e2e-test
  agnocast-ioctl-wrapper
  agnocastlib
  agnocast-sample-application
  agnocast-sample-interfaces
  ros2agnocast
)
for pkg_name in "${ros_pkg_names[@]}"; do
  apt_pkg="ros-${ROS_DISTRO}-${pkg_name}"
  if dpkg -l "$apt_pkg" 2>/dev/null | grep -q '^ii'; then
    conflicting_pkgs+=("$apt_pkg")
  fi
done

if [ ${#conflicting_pkgs[@]} -gt 0 ]; then
  echo "Error: The following apt-installed agnocast packages were found:"
  for pkg in "${conflicting_pkgs[@]}"; do
    echo "  - $pkg"
  done
  echo "These conflict with a source build. Please remove them first:"
  echo "  sudo apt-get remove -- ${conflicting_pkgs[*]}"
  exit 1
fi

# ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Rust
rustup toolchain install 1.75.0
rustup default 1.75.0
rustup component add clippy rustfmt
