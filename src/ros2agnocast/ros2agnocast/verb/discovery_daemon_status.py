"""Check that the per-IPC-namespace Agnocast discovery agent is alive.

The agent publishes the local Agnocast state on ``/_agnocast_discovery``
for cross-namespace observability. If the agent is not running (or
running in a different IPC namespace), that observability silently stops
working — this verb gives the operator a single place to confirm liveness.

This command only inspects the **current** IPC namespace (the one the
command itself runs in); run it inside the namespace you want to check.

Checks performed (each prints OK / NG with detail):

  * **process** — the discovery agent for **this** IPC namespace is alive,
    detected by probing the exclusive ``flock(2)`` it holds on its per-NS
    singleton lock file for its whole lifetime. The lock path already encodes
    the IPC namespace, so this needs no executable-path matching. (NG if the
    lock is free or its file is absent.)
  * **gossip** — a snapshot from **this** IPC namespace is received on
    ``/_agnocast_discovery`` within the timeout (snapshots from other
    namespaces sharing the topic don't count).
  * **type_registry** — *informational only.* Reports how many live Agnocast
    processes have registered in this namespace under
    ``${AGNOCAST_TMPFS_DIR:-/dev/shm}/agnocast_type_registry/<ipc_ns_inode>/``.
    An empty or absent registry just means nothing has registered yet, so this
    never counts as NG.

Exit code:

  * 0 — all OK
  * 1 — at least one NG (operator should investigate)
"""

import fcntl
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


def _singleton_lock_path(my_ns_inode) -> str:
    """Path of the agent's per-IPC-namespace singleton lock.

    Must match ``_singleton_lock_path`` in
    ``ros2agnocast_discovery_agent.agent``, including the ``AGNOCAST_TMPFS_DIR``
    override, so this verb probes the same file the agent locks.
    """
    root = os.environ.get('AGNOCAST_TMPFS_DIR') or '/dev/shm'
    return os.path.join(root, f'agnocast_discovery_agent_{my_ns_inode}.lock')


def _check_daemon_process(my_ns_inode):
    """Return (ok, detail) for the daemon-liveness check.

    The agent holds an exclusive ``flock(2)`` on its per-NS lock file for its
    whole lifetime. We probe that lock with a non-blocking ``LOCK_EX``: if we
    cannot take it, a live agent in this namespace is holding it. This is more
    robust than matching the agent's executable path, and the lock path already
    encodes the IPC namespace. (Closing our fd drops any lock we did take, so a
    successful probe leaves no lock behind.)
    """
    lock_path = _singleton_lock_path(my_ns_inode)
    if not os.path.exists(lock_path):
        return False, f'no agent lock at {lock_path}; agent never started in this NS'

    try:
        fd = os.open(lock_path, os.O_RDONLY | os.O_CLOEXEC)
    except OSError as e:
        return False, f'cannot open agent lock {lock_path}: {e}'

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True, f'agent holds the singleton lock ({lock_path})'
    except OSError as e:
        return False, f'cannot probe agent lock {lock_path}: {e}'
    finally:
        os.close(fd)

    # We acquired (and, via close above, released) the lock — nobody held it.
    return False, f'agent lock is free ({lock_path}); no live agent in this NS'


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


def _describe_type_registry(my_ns_inode) -> str:
    """Return a one-line description of the tmpfs type registry.

    This is *informational*, not a liveness signal: an empty or absent
    registry just means no Agnocast publisher/subscriber has registered in
    this namespace yet, which is normal and not an agent fault. So it returns
    a plain description (no OK/NG) of how many live registrations exist. Stale
    ``<pid>.txt`` files (process gone) are counted separately and don't count
    as live.
    """
    ns_dir = os.path.join(_type_registry_base(), str(my_ns_inode))
    if not os.path.isdir(ns_dir):
        return f'no Agnocast process has registered yet ({ns_dir} absent)'

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
        detail = f'{live} live registration(s) in {ns_dir}'
    else:
        detail = f'no Agnocast process has registered yet in {ns_dir}'
    if stale:
        detail += f' ({stale} stale <pid>.txt awaiting daemon cleanup)'
    return detail


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
            ('process', lambda: _check_daemon_process(my_ns_inode)),
            ('gossip', lambda: _check_gossip(my_ns_inode, args.gossip_timeout)),
        ]:
            ok, detail = fn()
            status = 'OK' if ok else 'NG'
            print(f'  {name:<14}{status}: {detail}')
            if not ok:
                any_ng = True

        # Informational only — does not affect the exit code.
        print(f'  {"type_registry":<14}INFO: {_describe_type_registry(my_ns_inode)}')

        return 1 if any_ng else 0
