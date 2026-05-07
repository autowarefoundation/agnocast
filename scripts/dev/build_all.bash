#!/bin/bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cd agnocast_kmod
make

cd ../agnocast_heaphook
cargo vendor
cargo build --release
cd ..
cp agnocast_heaphook/target/release/libagnocast_heaphook.so install/agnocastlib/lib
