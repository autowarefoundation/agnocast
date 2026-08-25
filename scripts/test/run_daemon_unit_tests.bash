#!/bin/bash
set -euo pipefail

AGNOCAST_DIR=$(realpath "$(dirname "$(readlink -f "$0")")/../..")
DAEMON_DIR="$AGNOCAST_DIR/agnocast_daemon"

if [ ! -d "$DAEMON_DIR" ]; then
  echo "agnocast_daemon directory not found: $DAEMON_DIR"
  exit 1
fi

cd "$DAEMON_DIR"

# Keep behavior aligned with kmod run_kunit.bash (build from clean state).
make clean
make test
