#!/bin/bash

AGNOCAST_PROCESS_NUM=100
OTHER_PROCESS_NUM=100
WAIT_TIME=5

source install/setup.bash
export LD_PRELOAD="libagnocast_heaphook.so:$LD_PRELOAD"
for i in $(seq 1 $AGNOCAST_PROCESS_NUM); do
    ros2 run agnocast_sample_application talker --ros-args --remap __node:=talker_node$i --remap /my_topic:=/topic$i &
done

for i in $(seq 1 $OTHER_PROCESS_NUM); do
    sleep "$(echo "scale=3; $WAIT_TIME + $i * 0.001" | bc)" &
done

sleep $WAIT_TIME

pkill -2 -f talker

wait
