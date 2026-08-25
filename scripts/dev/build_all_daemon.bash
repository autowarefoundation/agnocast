#!/bin/bash

# Build the whole workspace in USER DAEMON mode.
#
# This mirrors scripts/dev/build_all.bash but:
#   - builds the standalone agnocast_daemon executable instead of the kernel module, and
#   - passes -DAGNOCAST_USE_DAEMON=ON so agnocastlib (and every downstream
#     package that inherits its exported interface) uses the Unix-socket client
#     instead of ioctl on /dev/agnocast.
#
# The heaphook is still required in daemon mode: it redirects heap allocations
# into shared memory, which is orthogonal to how metadata is managed.
#
# After building, start the daemon before launching any Agnocast process:
#   scripts/run_daemon.bash
#
# Run from the repository root.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGNOCAST_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$AGNOCAST_ROOT"

echo "==> Building agnocast_daemon"
(cd agnocast_daemon && make)

echo "==> Building workspace with -DAGNOCAST_USE_DAEMON=ON"
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAGNOCAST_USE_DAEMON=ON

echo "==> Building agnocast_heaphook"
cd agnocast_heaphook
cargo vendor
cargo build --release
cd ..
cp agnocast_heaphook/target/release/libagnocast_heaphook.so install/agnocastlib/lib

echo ""
echo "Done. Start the daemon before running Agnocast processes:"
echo "    scripts/run_daemon.bash"
