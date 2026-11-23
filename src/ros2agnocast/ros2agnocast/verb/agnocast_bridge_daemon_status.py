"""Check that this (IPC namespace, ROS_DOMAIN_ID)'s bridge daemon is alive.

The bridge manager binds an abstract status socket named after that pair
(``\\0agnocast_bridge_status_<ipc_ns_inode>[_d<domain>]``). This verb, like
other ``ros2`` commands, only inspects the **current** namespace and domain (the
ones the command itself runs in); run it inside the namespace and with the
ROS_DOMAIN_ID you want to check.

There are four possible verdicts:

  * The bridge daemon is running.
  * The bridge daemon is running but not responsive.
  * The bridge daemon is running but incompatible.
  * The bridge daemon is not running.

Exit code: 0 when the daemon is running, 1 otherwise.
"""

import json
import os
import socket
from dataclasses import dataclass

from ros2cli.verb import VerbExtension

DEFAULT_TIMEOUT_SEC = 1.0


def _self_ipc_ns_inode() -> int:
    return os.stat('/proc/self/ns/ipc').st_ino


def _read_ros_domain_id() -> int:
    raw = os.environ.get('ROS_DOMAIN_ID')
    if not raw:
        return 0
    try:
        value = int(raw)
    except ValueError:
        return 0
    if value < 0 or value > 0xFFFFFFFF:
        return 0
    return value


def _bridge_status_socket_addr(ipc_ns_inode: int, domain_id: int) -> str:
    """Return the status-socket address for an (IPC namespace, domain) pair.

    Must stay in sync with ``agnocast::create_uds_addr_for_bridge_status()``.
    """
    name = f'\x00agnocast_bridge_status_{ipc_ns_inode}'
    if domain_id:
        name += f'_d{domain_id}'
    return name


@dataclass
class RecvMsgOk:
    pid: int

@dataclass
class RecvMsgNotRunning:
    pass

@dataclass
class RecvMsgUnresponsive:
    pass

@dataclass
class RecvMsgParseError:
    pass

RecvMsgResult = RecvMsgOk | RecvMsgNotRunning | RecvMsgUnresponsive | RecvMsgParseError

class BridgeControlSocket:
    """Wraps a UNIX domain socket path to the bridge status socket."""

    def __init__(self, sock_path: str) -> None:
        self._sock_path = sock_path

    def recv_msg(self, timeout_sec: float) -> RecvMsgResult:
        """Connect to the socket, receive the JSON payload, and return a typed result.

        The daemon sends a JSON payload immediately on connection:
        ``{"pid":<int>}``
        """
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.settimeout(timeout_sec)
            try:
                sock.connect(self._sock_path)
            except OSError:
                return RecvMsgNotRunning()

            chunks: list[bytes] = []
            try:
                while True:
                    chunk = sock.recv(256)
                    if not chunk:
                        break
                    chunks.append(chunk)
            except OSError:
                return RecvMsgUnresponsive()
        finally:
            sock.close()

        if not chunks:
            return RecvMsgUnresponsive()

        try:
            data = json.loads(b''.join(chunks).decode())
            return RecvMsgOk(pid=data['pid'])
        except Exception:
            return RecvMsgParseError()


class BridgeDaemonStatusVerb(VerbExtension):
    """Check liveness of the current (IPC namespace, ROS_DOMAIN_ID)'s bridge daemon."""

    _socket_class: type[BridgeControlSocket] = BridgeControlSocket

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--timeout',
            type=float,
            default=DEFAULT_TIMEOUT_SEC,
            metavar='SECONDS',
            help=f'Timeout in seconds for each ping (default: {DEFAULT_TIMEOUT_SEC})',
        )
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            default=False,
            help='Print the IPC namespace inode and the ROS_DOMAIN_ID',
        )

    def main(self, *, args):
        ipc_inode = _self_ipc_ns_inode()
        domain_id = _read_ros_domain_id()
        sock_addr = _bridge_status_socket_addr(ipc_inode, domain_id)
        recv_result = self._socket_class(sock_addr).recv_msg(args.timeout)

        if args.verbose:
            print(f'IPC namespace inode: {ipc_inode}')
            print(f'ROS_DOMAIN_ID: {domain_id}')
            print('')

        if isinstance(recv_result, RecvMsgOk):
            print(f'OK. The bridge daemon is running. (PID: {recv_result.pid})')
            return 0

        if isinstance(recv_result, RecvMsgNotRunning):
            print('NG. The bridge daemon is not running.')
        elif isinstance(recv_result, RecvMsgUnresponsive):
            print(f'NG. The bridge daemon is running but not responsive (timed out after {args.timeout} s).')
        elif isinstance(recv_result, RecvMsgParseError):
            print('NG. The bridge daemon is running but incompatible. Ensure the CLI and agnocastlib versions match.')
        return 1
