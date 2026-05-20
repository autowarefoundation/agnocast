"""Check that the per-IPC-namespace Agnocast discovery agent is alive.

The agent is responsible for (a) publishing the local Agnocast state on
``/_agnocast_discovery`` for cross-namespace observability and (b)
dispatching `MqMsgDaemonBridge` requests so that cross-IPC-NS bridges
are auto-generated. If the daemon is not running (or running in a
different IPC namespace), both features silently stop working — this
verb gives the operator a single place to confirm liveness.

Checks performed (each prints OK / NG with detail):

  * **process** — any `ros2agnocast_discovery_agent` process is running
    in **this** IPC namespace (matched by `/proc/<pid>/ns/ipc`).
  * **gossip** — `/_agnocast_discovery` has at least one DDS publisher
    on the current `ROS_DOMAIN_ID` and a snapshot is received within
    the timeout.
  * **type_registry** — the tmpfs directory
    `/run/agnocast/<ipc_ns_inode>/` exists and contains at least one
    `<pid>.txt` (proves at least one Agnocast process has registered).

Exit code:

  * 0 — all OK
  * 1 — at least one NG (operator should investigate)
"""

import os
import time

import rclpy
from rclpy.node import Node
from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState


_GOSSIP_TOPIC = '/_agnocast_discovery'
_TYPE_REGISTRY_BASE = '/run/agnocast'


def _self_ipc_ns_inode():
    return os.stat('/proc/self/ns/ipc').st_ino


def _check_daemon_process(my_ns_inode):
    """Return (ok, detail) for the daemon-process check."""
    found_pids = []
    for pid_str in os.listdir('/proc'):
        if not pid_str.isdigit():
            continue
        pid = int(pid_str)
        try:
            with open(f'/proc/{pid}/comm') as fp:
                comm = fp.read().strip()
        except (FileNotFoundError, PermissionError):
            continue
        # The discovery agent runs as a python script; its `comm` is the
        # python interpreter. We check cmdline for the agent's module name.
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fp:
                cmdline = fp.read().replace(b'\0', b' ').decode('utf-8', errors='replace')
        except (FileNotFoundError, PermissionError):
            continue
        if 'ros2agnocast_discovery_agent' not in cmdline:
            continue
        # Must be in the same IPC namespace as the caller.
        try:
            their_ns = os.stat(f'/proc/{pid}/ns/ipc').st_ino
        except (FileNotFoundError, PermissionError):
            continue
        if their_ns != my_ns_inode:
            continue
        found_pids.append(pid)

    if not found_pids:
        return False, 'no discovery_agent process found in this IPC namespace'
    if len(found_pids) > 1:
        return True, f'pid(s)={found_pids} (warning: multiple daemons running in this NS)'
    return True, f'pid={found_pids[0]}'


def _check_gossip(timeout_sec=2.0):
    """Return (ok, detail) for the gossip-subscription check."""
    rclpy_was_initialized = rclpy.ok()
    if not rclpy_was_initialized:
        rclpy.init()
    try:
        with NodeStrategy(None) as node:
            from ros2agnocast.discovery import gossip_qos
            received = []

            def cb(msg: AgnocastDaemonState) -> None:
                received.append(msg)

            sub = node.create_subscription(
                AgnocastDaemonState, _GOSSIP_TOPIC, cb, gossip_qos())
            try:
                deadline = time.monotonic() + timeout_sec
                while time.monotonic() < deadline and not received:
                    rclpy.spin_once(node, timeout_sec=0.05)
            finally:
                node.destroy_subscription(sub)

            if not received:
                return False, (
                    f'no AgnocastDaemonState received on {_GOSSIP_TOPIC} within '
                    f'{timeout_sec}s')
            return True, f'received {len(received)} snapshot(s) on {_GOSSIP_TOPIC}'
    finally:
        # Only shutdown rclpy if we initialized it here. Otherwise leave
        # the caller's context intact.
        if not rclpy_was_initialized and rclpy.ok():
            rclpy.shutdown()


def _check_type_registry(my_ns_inode):
    """Return (ok, detail) for the tmpfs type registry check."""
    ns_dir = os.path.join(_TYPE_REGISTRY_BASE, str(my_ns_inode))
    if not os.path.isdir(ns_dir):
        return False, f'directory missing: {ns_dir}'
    files = [f for f in os.listdir(ns_dir) if f.endswith('.txt')]
    if not files:
        return False, f'{ns_dir} contains no <pid>.txt registrations'
    return True, f'{ns_dir} has {len(files)} registration file(s)'


class DiscoveryDaemonStatusVerb(VerbExtension):
    """Check the per-IPC-namespace Agnocast discovery agent's liveness."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--gossip-timeout', type=float, default=2.0,
            help='Seconds to wait for a /_agnocast_discovery snapshot (default: 2.0)')

    def main(self, *, args):
        my_ns_inode = _self_ipc_ns_inode()
        print(f'IPC namespace inode: {my_ns_inode}')

        any_ng = False
        for name, fn in [
            ('process       ', lambda: _check_daemon_process(my_ns_inode)),
            ('gossip        ', lambda: _check_gossip(args.gossip_timeout)),
            ('type_registry ', lambda: _check_type_registry(my_ns_inode)),
        ]:
            ok, detail = fn()
            status = 'OK' if ok else 'NG'
            print(f'  {name}{status}: {detail}')
            if not ok:
                any_ng = True

        return 1 if any_ng else 0
