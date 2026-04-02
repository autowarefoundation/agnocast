#!/bin/bash

# ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Rust
rustup toolchain install 1.75.0
rustup default 1.75.0
rustup component add clippy rustfmt
