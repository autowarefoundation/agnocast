"""Check that the per-IPC-namespace Agnocast discovery agent is alive.

The agent publishes the local Agnocast state on ``/_agnocast_discovery``
for cross-namespace observability. If the agent is not running (or
running in a different IPC namespace), that observability silently stops
working — this verb gives the operator a single place to confirm liveness.

This command only inspects the **current** IPC namespace (the one the
command itself runs in); run it inside the namespace you want to check.

Checks performed (each prints OK / NG with detail):

  * **process** — any ``ros2agnocast_discovery_agent`` process is running
    in **this** IPC namespace (matched by ``/proc/<pid>/ns/ipc``).
  * **gossip** — a snapshot from **this** IPC namespace is received on
    ``/_agnocast_discovery`` within the timeout (snapshots from other
    namespaces sharing the topic don't count).
  * **type_registry** — the tmpfs directory
    ``${AGNOCAST_TMPFS_DIR:-/dev/shm}/agnocast_type_registry/<ipc_ns_inode>/``
    holds at least one ``<pid>.txt`` whose process is still alive (proves
    a live Agnocast process has registered).

Exit code:

  * 0 — all OK
  * 1 — at least one NG (operator should investigate)
"""

import os

from ros2cli.node.strategy import NodeStrategy
from ros2cli.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    collect_announcements,
    GOSSIP_TOPIC,
    warn_if_gossip_timeout_overridden,
)


def _type_registry_base() -> str:
    """Resolve the tmpfs root, honoring ``AGNOCAST_TMPFS_DIR`` like the writer."""
    root = os.environ.get('AGNOCAST_TMPFS_DIR') or '/dev/shm'
    return os.path.join(root, 'agnocast_type_registry')


def _self_ipc_ns_inode():
    return os.stat('/proc/self/ns/ipc').st_ino


def _check_daemon_process(my_ns_inode):
    """Return (ok, detail) for the daemon-process check."""
    found_pids = []
    for pid_str in os.listdir('/proc'):
        if not pid_str.isdigit():
            continue

        pid = int(pid_str)
        # The discovery agent runs as a python script — its ``comm`` is the
        # python interpreter — so match on cmdline instead.
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fp:
                cmdline = fp.read().replace(b'\0', b' ').decode('utf-8', errors='replace')
        except (FileNotFoundError, PermissionError):
            continue

        # Match the actual agent binary path so we don't also pick up
        # ``ros2 run ros2agnocast_discovery_agent discovery_agent``
        # wrapper processes (whose cmdline contains the same package name
        # as an argv slot rather than as the executable path).
        if '/ros2agnocast_discovery_agent/lib/ros2agnocast_discovery_agent/discovery_agent' \
                not in cmdline:
            continue

        # Must be in the same IPC namespace as the caller.
        try:
            their_ns_inode = os.stat(f'/proc/{pid}/ns/ipc').st_ino
        except (FileNotFoundError, PermissionError):
            continue

        if their_ns_inode != my_ns_inode:
            continue

        found_pids.append(pid)

    if not found_pids:
        return False, 'no discovery_agent process found in this IPC namespace'

    if len(found_pids) > 1:
        return True, f'pid(s)={found_pids} (warning: multiple daemons running in this NS)'

    return True, f'pid={found_pids[0]}'


def _check_gossip(my_ns_inode, timeout_sec):
    """Return (ok, detail) for the gossip-subscription check.

    ``collect_announcements`` aggregates every namespace publishing on the
    shared gossip topic, so we filter for a snapshot tagged with our own
    ``ipc_ns_inode`` — a snapshot from another namespace must not pass this
    check. ``NodeStrategy`` initializes rclpy itself; do not call
    ``rclpy.init`` here or the second call raises ``Context.init() must
    only be called once``.
    """
    with NodeStrategy(None) as node:
        snapshots = collect_announcements(node, timeout_sec)

    seen_my_ns = any(s.ipc_ns_inode == my_ns_inode for s in snapshots)
    if seen_my_ns:
        return True, f'received a snapshot from this IPC namespace on {GOSSIP_TOPIC}'

    if snapshots:
        return False, (
            f'no snapshot from this IPC namespace (inode={my_ns_inode}) within '
            f'{timeout_sec}s; saw {len(snapshots)} from other namespace(s)')

    return False, f'no AgnocastDaemonState received on {GOSSIP_TOPIC} within {timeout_sec}s'


def _check_type_registry(my_ns_inode):
    """Return (ok, detail) for the tmpfs type registry check.

    A ``<pid>.txt`` file alone isn't enough — a process that died without
    cleaning up leaves a stale file. We require at least one file whose
    name is a PID with a live ``/proc/<pid>`` entry, matching the writer's
    "one live process has registered" contract.
    """
    ns_dir = os.path.join(_type_registry_base(), str(my_ns_inode))
    if not os.path.isdir(ns_dir):
        return False, f'directory missing: {ns_dir}'

    live = 0
    stale = 0
    for name in os.listdir(ns_dir):
        if not name.endswith('.txt'):
            continue

        pid_str = name[:-len('.txt')]
        if not pid_str.isdigit():
            continue

        if os.path.exists(f'/proc/{pid_str}'):
            live += 1
        else:
            stale += 1

    if live:
        return True, f'{ns_dir} has {live} live registration file(s)'

    detail = f'{ns_dir} has no registration from a live process'
    if stale:
        detail += f' ({stale} stale <pid>.txt awaiting daemon cleanup)'
    return False, detail


class DiscoveryDaemonStatusVerb(VerbExtension):
    """Check the current IPC namespace's Agnocast discovery agent liveness."""

    def add_arguments(self, parser, cli_name):
        add_gossip_timeout_arg(parser)

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)

        my_ns_inode = _self_ipc_ns_inode()
        print(f'IPC namespace inode: {my_ns_inode}')

        any_ng = False
        for name, fn in [
            ('process       ', lambda: _check_daemon_process(my_ns_inode)),
            ('gossip        ', lambda: _check_gossip(my_ns_inode, args.gossip_timeout)),
            ('type_registry ', lambda: _check_type_registry(my_ns_inode)),
        ]:
            ok, detail = fn()
            status = 'OK' if ok else 'NG'
            print(f'  {name}{status}: {detail}')
            if not ok:
                any_ng = True

        return 1 if any_ng else 0
