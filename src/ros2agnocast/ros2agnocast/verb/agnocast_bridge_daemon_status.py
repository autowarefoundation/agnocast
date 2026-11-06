import json
import os
import socket
from dataclasses import dataclass

from ros2cli.verb import VerbExtension

_ABSTRACT_SOCKET_PREFIX = 'agnocast_bridge'
DEFAULT_TIMEOUT_SEC = 1.0


def _get_self_ipc_namespace_inode() -> int:
    return os.stat('/proc/self/ns/ipc').st_ino


def _find_bridge_pids(ipc_inode: int) -> list[int]:
    """Find all bridge process PIDs via /proc/net/unix.

    Abstract sockets are listed with an '@' prefix in the Path column.
    Only entries matching our prefix and ipc_inode are considered.
    Because abstract sockets are released by the OS on process exit,
    every entry found here belongs to a living process.
    """
    target_prefix = f'@{_ABSTRACT_SOCKET_PREFIX}/{ipc_inode}/'
    pids: list[int] = []
    try:
        with open('/proc/net/unix') as f:
            for line in f:
                parts = line.split()
                if not parts:
                    continue
                path = parts[-1]
                if path.startswith(target_prefix):
                    pid_str = path[len(target_prefix):]
                    try:
                        pids.append(int(pid_str))
                    except ValueError:
                        pass
    except OSError:
        return []
    return sorted(pids)


def _abstract_socket_addr_for_pid(ipc_inode: int, pid: int) -> str:
    """Return the abstract socket address for a given IPC namespace inode and PID.

    Python's socket module treats addresses starting with '\\0' as abstract.
    """
    return f'\x00{_ABSTRACT_SOCKET_PREFIX}/{ipc_inode}/{pid}'


@dataclass
class RecvMsgOk:
    type: str
    ipc_ns: int
    domain_id: int
    pid: int

@dataclass
class RecvMsgFailed:
    pass

@dataclass
class RecvMsgParseError:
    pass

RecvMsgResult = RecvMsgOk | RecvMsgFailed | RecvMsgParseError

class BridgeControlSocket:
    """Wraps a UNIX domain socket path to the bridge control socket."""

    def __init__(self, sock_path: str) -> None:
        self._sock_path = sock_path

    def recv_msg(self, timeout_sec: float) -> RecvMsgResult:
        """Connect to the socket, receive the JSON payload, and return a typed result.

        The daemon sends a JSON payload immediately on connection:
        ``{"type":"bridge","ipc_ns":<int>,"domain_id":<int>,"pid":<int>}``
        """
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        chunks: list[bytes] = []
        try:
            sock.settimeout(timeout_sec)
            sock.connect(self._sock_path)
            while True:
                chunk = sock.recv(256)
                if not chunk:
                    break
                chunks.append(chunk)
        except OSError:
            # Treat any connect/recv error as a liveness failure.
            pass
        finally:
            sock.close()

        if not chunks:
            return RecvMsgFailed()

        try:
            data = json.loads(b''.join(chunks).decode())
            return RecvMsgOk(
                type=data['type'],
                ipc_ns=data['ipc_ns'],
                domain_id=data['domain_id'],
                pid=data['pid'],
            )
        except Exception:
            return RecvMsgParseError()


_NG_MSG_RECV_FAILED = "Couldn't recv message via UNIX socket."
_NG_MSG_PARSE_ERROR = (
    'Failed to parse the response. '
    'Please ensure the CLI and agnocastlib versions are compatible.'
)
_NG_MSG_UNKNOWN_TYPE = (
    'Response contains an unknown bridge type. '
    'Please ensure the CLI and agnocastlib versions are compatible.'
)
_NG_MSG_LEGACY_TYPE = (
    'Response contains a legacy bridge type, which is no longer supported. '
    'Please ensure the CLI and agnocastlib versions are compatible.'
)
_LEGACY_BRIDGE_TYPES = ('standard', 'performance')


class BridgeDaemonStatusVerb(VerbExtension):
    """Check liveness of Agnocast Bridge daemon processes via UDS control socket."""

    _socket_class: type[BridgeControlSocket] = BridgeControlSocket

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--timeout',
            type=float,
            default=DEFAULT_TIMEOUT_SEC,
            metavar='SECONDS',
            help=f'Timeout in seconds for each ping (default: {DEFAULT_TIMEOUT_SEC})',
        )

    def _collect_bridge_results(
        self,
        alive_pids: list[int],
        ipc_inode: int,
        timeout_sec: float,
    ) -> tuple[list[str], bool]:
        results: list[str] = []
        ok = bool(alive_pids)
        domains = set()

        for pid in alive_pids:
            sock_addr = _abstract_socket_addr_for_pid(ipc_inode, pid)
            recv_result = self._socket_class(sock_addr).recv_msg(timeout_sec)

            if isinstance(recv_result, RecvMsgFailed):
                ok = False
                results.append(
                    f'  PID {pid} NG: {_NG_MSG_RECV_FAILED}'
                )
                continue

            if isinstance(recv_result, RecvMsgParseError):
                ok = False
                results.append(
                    f'  PID {pid} NG: {_NG_MSG_PARSE_ERROR}'
                )
                continue

            bridge_type = recv_result.type
            if bridge_type in _LEGACY_BRIDGE_TYPES:
                ok = False
                results.append(
                    f'  PID {pid} NG: {_NG_MSG_LEGACY_TYPE}'
                )
                continue

            if bridge_type != 'bridge':
                ok = False
                results.append(
                    f'  PID {pid} NG: {_NG_MSG_UNKNOWN_TYPE}'
                )
                continue

            if recv_result.pid != pid or recv_result.ipc_ns != ipc_inode:
                ok = False
                mismatch_message = (
                    f'Response contains mismatched data: '
                    f'pid={recv_result.pid}, ipc_ns={recv_result.ipc_ns}'
                )
                results.append(
                    f'  PID {pid} NG: {mismatch_message}'
                )
                continue

            if recv_result.domain_id in domains:
                ok = False
                duplicate_message = (
                    f'Multiple bridge daemons are running for ROS domain '
                    f'{recv_result.domain_id}.'
                )
                results.append(
                    f'  PID {pid} NG: {duplicate_message}'
                )
                continue

            domains.add(recv_result.domain_id)
            results.append(f'  PID {pid} (ROS domain {recv_result.domain_id}) OK')

        return results, ok

    def main(self, *, args):
        ipc_inode = _get_self_ipc_namespace_inode()
        timeout_sec = args.timeout

        bridge_pids = _find_bridge_pids(ipc_inode)
        results, ok = self._collect_bridge_results(bridge_pids, ipc_inode, timeout_sec)

        print(f'IPC namespace inode: {ipc_inode}')
        print()

        print('Bridge Status:')
        if results:
            print('\n'.join(results))
        else:
            print('  No bridge daemons are running.')

        return 0 if ok else 1
