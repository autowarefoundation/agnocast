"""Check that this (IPC namespace, ROS_DOMAIN_ID)'s Agnocast discovery agent is alive.

The agent publishes the local Agnocast state on ``/_agnocast_discovery``
for cross-namespace observability. If the agent is not running (or
running in a different IPC namespace), that observability silently stops
working — this verb gives the operator a single place to confirm liveness.

The agent runs one instance per (IPC namespace, ROS_DOMAIN_ID), so this
command only inspects the **current** namespace and domain (the ones the
command itself runs in); run it inside the namespace and with the
ROS_DOMAIN_ID you want to check.

Default output is a single one-line verdict — nothing else:

  * ``OK. The discovery agent is running.``
  * ``NG. The discovery agent is not running.``
  * ``NG. The discovery agent is running but not publishing.``

(``--verbose`` adds the reason behind the verdict; see below.)

The verdict is driven by two internal checks:

  * **process** — the kmod holds a per-(ns, domain) discovery-agent
    registry entry for the agent's whole lifetime, so we query it. The kmod
    scopes the query to the caller's IPC namespace and the given domain, so
    this needs no executable-path matching.
  * **gossip** — a snapshot from this IPC namespace is received on
    ``/_agnocast_discovery`` within the timeout. ``gossip`` OK implies
    ``process`` OK (an agent that publishes is alive), so it is the primary,
    end-to-end signal; ``process`` only distinguishes "not running" from
    "running but not publishing" when ``gossip`` is NG.

``--verbose`` additionally prints the IPC namespace inode, the ROS_DOMAIN_ID,
each check's result, and a ``type_registry`` line (how many live Agnocast processes have
registered). None of those affect the exit code — they are context, not part
of the verdict.

Exit code: 0 when the agent is running (gossip OK), 1 otherwise.
"""

import ctypes
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


def _read_ros_domain_id() -> int:
    """Return ROS_DOMAIN_ID from the environment (0 if unset, empty, or
    unparsable), matching ``ros2agnocast_discovery_agent.agent._read_ros_domain_id``
    so this verb resolves the same domain the agent runs under.
    """
    raw = os.environ.get('ROS_DOMAIN_ID')
    if not raw:
        return 0
    try:
        return int(raw)
    except ValueError:
        return 0


def _check_daemon_process(domain_id):
    """Return (ok, reason) for the daemon-liveness check.

    The kmod owns the discovery-agent registry -- one entry per (IPC namespace, domain), removed
    the moment the agent exits -- so a read-only ioctl is the authoritative "is the agent alive?"
    query for the current namespace and the given domain (no stale state). ``reason``
    is a short phrase for the verdict breakdown, not a full sentence.
    """
    try:
        lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    except OSError as e:
        return False, f'cannot load the ioctl wrapper: {e}'
    try:
        lib.agnocast_discovery_agent_exists.argtypes = [ctypes.c_uint32]
        lib.agnocast_discovery_agent_exists.restype = ctypes.c_int
        ret = lib.agnocast_discovery_agent_exists(domain_id)
    except AttributeError:
        # Older/mismatched wrapper without the symbol: report NG instead of crashing the CLI.
        return False, 'ioctl wrapper lacks agnocast_discovery_agent_exists (version skew?)'
    if ret < 0:
        return False, 'kmod query failed (is the agnocast module loaded?)'
    if ret == 1:
        return True, f'registered in the kmod (domain {domain_id})'
    return False, f'no agent registered in the kmod (domain {domain_id})'


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
        my_domain_id = _read_ros_domain_id()

        proc_ok, proc_reason = _check_daemon_process(my_domain_id)
        # gossip is the end-to-end signal, but it only matters if a process is
        # up; skip its timeout wait when the agent clearly isn't running.
        if proc_ok:
            gossip_ok, gossip_reason = _check_gossip(my_ns_inode, args.gossip_timeout)
        else:
            gossip_ok, gossip_reason = None, None

        if args.verbose:
            print(f'IPC namespace inode: {my_ns_inode}, ROS_DOMAIN_ID: {my_domain_id}')
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
