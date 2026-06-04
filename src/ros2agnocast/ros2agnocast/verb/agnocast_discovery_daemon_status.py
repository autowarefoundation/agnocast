"""Check that the per-IPC-namespace Agnocast discovery agent is alive.

The agent publishes the local Agnocast state on ``/_agnocast_discovery``
for cross-namespace observability. If the agent is not running (or
running in a different IPC namespace), that observability silently stops
working — this verb gives the operator a single place to confirm liveness.

This command only inspects the **current** IPC namespace (the one the
command itself runs in); run it inside the namespace you want to check.

Default output is a single one-line verdict — nothing else:

  * ``OK. The discovery agent is running.``
  * ``NG. The discovery agent is not running.``
  * ``NG. The discovery agent is running but not publishing.``

(``--verbose`` adds the reason behind the verdict; see below.)

The verdict is driven by two internal checks:

  * **process** — the agent holds an exclusive ``flock(2)`` on its per-NS
    singleton lock file for its whole lifetime, so we probe that lock. The
    lock path encodes the IPC namespace, so this needs no executable-path
    matching.
  * **gossip** — a snapshot from this IPC namespace is received on
    ``/_agnocast_discovery`` within the timeout. ``gossip`` OK implies
    ``process`` OK (an agent that publishes is alive), so it is the primary,
    end-to-end signal; ``process`` only distinguishes "not running" from
    "running but not publishing" when ``gossip`` is NG.

``--verbose`` additionally prints the IPC namespace inode, each check's
result, and a ``type_registry`` line (how many live Agnocast processes have
registered). None of those affect the exit code — they are context, not part
of the verdict.

Exit code: 0 when the agent is running (gossip OK), 1 otherwise.
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
    """Return (ok, reason) for the daemon-liveness check.

    The agent holds an exclusive ``flock(2)`` on its per-NS lock file for its
    whole lifetime. We probe that lock with a non-blocking ``LOCK_EX``: if we
    cannot take it, a live agent in this namespace is holding it. The kernel
    releases the lock when the holder dies, so a lock we *can* take (or a
    missing file) means no live agent. ``reason`` is a short phrase for the
    verdict breakdown, not a full sentence.
    """
    lock_path = _singleton_lock_path(my_ns_inode)
    if not os.path.exists(lock_path):
        return False, f'no singleton lock file ({lock_path})'

    try:
        fd = os.open(lock_path, os.O_RDONLY | os.O_CLOEXEC)
    except OSError as e:
        return False, f'cannot open lock {lock_path}: {e}'

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Could NOT take the lock -> a live agent is holding it.
        return True, f'holds the singleton lock ({lock_path})'
    except OSError as e:
        return False, f'cannot probe lock {lock_path}: {e}'
    finally:
        os.close(fd)

    # We took the lock (closing the fd above released it) -> nobody held it.
    return False, f'singleton lock is free ({lock_path})'


def _check_gossip(my_ns_inode, timeout_sec):
    """Return (ok, reason) for the gossip-subscription check.

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
        return True, f'snapshot received on {GOSSIP_TOPIC}'

    if snapshots:
        return False, (
            f'no snapshot from this namespace within {timeout_sec}s '
            f'({len(snapshots)} from other namespace(s))')

    return False, f'no snapshot on {GOSSIP_TOPIC} within {timeout_sec}s'


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
        parser.add_argument(
            '-v', '--verbose', action='store_true',
            help='also print the IPC namespace inode, each internal check, and '
                 'the type_registry info line (none affect the exit code)')

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)

        my_ns_inode = _self_ipc_ns_inode()

        proc_ok, proc_reason = _check_daemon_process(my_ns_inode)
        # gossip is the end-to-end signal, but it only matters if a process is
        # up; skip its timeout wait when the agent clearly isn't running.
        if proc_ok:
            gossip_ok, gossip_reason = _check_gossip(my_ns_inode, args.gossip_timeout)
        else:
            gossip_ok, gossip_reason = None, None

        if args.verbose:
            print(f'IPC namespace inode: {my_ns_inode}')
            print(f'  process:       {"OK" if proc_ok else "NG"} ({proc_reason})')
            if gossip_ok is None:
                print('  gossip:        skipped (agent not running)')
            else:
                print(f'  gossip:        {"OK" if gossip_ok else "NG"} ({gossip_reason})')
            print(f'  type_registry: {_describe_type_registry(my_ns_inode)}')
            print('')

        # Default output is the verdict only — the per-check reasons are
        # diagnostic detail, available via --verbose above.
        if proc_ok and gossip_ok:
            print('OK. The discovery agent is running.')
            return 0

        if not proc_ok:
            print('NG. The discovery agent is not running.')
            return 1

        print('NG. The discovery agent is running but not publishing.')
        return 1
