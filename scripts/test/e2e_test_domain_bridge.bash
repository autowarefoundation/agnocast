#!/bin/bash
# Case 2 (same IPC namespace, cross-domain) zero-copy delivery e2e.
#
# Registers a domain bridge rule with the kmod, then runs an Agnocast publisher
# in FROM_DOMAIN and subscriber in TO_DOMAIN (same IPC namespace) and asserts the
# subscriber receives every message — proving kmod cross-domain matching without
# any DDS/bridge node. Run after building + sourcing the workspace and loading
# the kmod. Pure Case 2 uses no bridge, so AGNOCAST_BRIDGE_MODE=off is fine.

set -eo pipefail

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

TOPIC_NAME="${E2E_TOPIC_NAME:-/test_domain_bridge_topic}"
FROM_DOMAIN="${E2E_FROM_DOMAIN:-1}"
TO_DOMAIN="${E2E_TO_DOMAIN:-2}"

# Register the rule before any endpoint joins. The rule is per-IPC-namespace, so
# registering it from this shell (same namespace as the launch_test) suffices;
# no daemon is needed for pure Case 2.
echo "Registering domain bridge rule: ${TOPIC_NAME} ${FROM_DOMAIN}->${TO_DOMAIN}"
python3 - "$TOPIC_NAME" "$FROM_DOMAIN" "$TO_DOMAIN" <<'PY'
import ctypes
import sys

topic, from_domain, to_domain = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
lib.add_agnocast_domain_bridge_rule.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
lib.add_agnocast_domain_bridge_rule.restype = ctypes.c_int
rc = lib.add_agnocast_domain_bridge_rule(topic.encode('utf-8'), from_domain, to_domain)
if rc != 0:
    sys.stderr.write(f'add_agnocast_domain_bridge_rule failed (rc={rc})\n')
sys.exit(0 if rc == 0 else 1)
PY

echo "Running Case 2 cross-domain delivery test..."
E2E_TOPIC_NAME="$TOPIC_NAME" E2E_FROM_DOMAIN="$FROM_DOMAIN" E2E_TO_DOMAIN="$TO_DOMAIN" \
    launch_test src/agnocast_e2e_test/test/test_domain_bridge.py
