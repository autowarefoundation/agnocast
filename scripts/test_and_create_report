#!/bin/bash

# Remove agnocast-heaphook if it is installed, for unit tests
HEAPHOOK_FOUND=0
if ls "/usr/lib/" | grep -q "libagnocast_heaphook.so"; then
    sudo apt-get -y remove agnocast-heaphook
    HEAPHOOK_FOUND=1
fi

rm -rf build install log
colcon build --cmake-args -DCOVERAGE=ON

# Build and install heaphook for tests
cd agnocast_heaphook
cargo build --release
cd ..
cp agnocast_heaphook/target/release/libagnocast_heaphook.so install/agnocastlib/lib/

colcon test --event-handlers console_direct+
lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/test/*' '*gtest*' '*gmock*' '/opt/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_report_agnocastlib
rm -rf coverage.info coverage_filtered.info

# Reinstall agnocast-heaphook if it was installed
if [ $HEAPHOOK_FOUND -eq 1 ]; then
    cd src/agnocast_heaphook
    cargo deb --install
fi

echo -e "\nPlease open coverage_report_agnocastlib/index.html"
