#!/bin/bash

source install/setup.bash
export AGNOCAST_BRIDGE_MODE=0
ros2 launch agnocast_sample_application tracepoint_verifier.launch.xml
