#!/bin/bash

if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi
