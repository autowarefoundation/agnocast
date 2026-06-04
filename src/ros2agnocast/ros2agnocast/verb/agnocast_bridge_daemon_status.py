import json
import os
import socket
from dataclasses import dataclass, field

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
        ``{"type":"standard"|"performance","ipc_ns":<int>,"pid":<int>}``
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

@dataclass
class _BridgeResult:
    pid: int
    ipc_ns: int
    bridge_type: str  # 'performance', 'standard', or 'unknown'
    is_ok: bool
    ng_message: str = field(default='')


def _build_summary(results: list[_BridgeResult]) -> tuple[str, bool]:
    perf_results = [r for r in results if r.bridge_type == 'performance']
    std_results = [r for r in results if r.bridge_type == 'standard']

    if not results:
        return 'NG. There is no bridge daemon process.', True
    if len(perf_results) >= 2:
        pids = ', '.join(str(r.pid) for r in perf_results)
        return f'NG. There are multiple performance bridge processes. (PIDs: {pids})', True
    if perf_results and std_results:
        pids = ', '.join(str(r.pid) for r in results)
        return f'NG. Both performance and standard bridge processes exist. (PIDs: {pids})', True

    first_ng_result = next((r for r in results if not r.is_ok), None)
    if first_ng_result is not None:
        pid = first_ng_result.pid
        if first_ng_result.bridge_type == 'performance':
            return f'NG. The performance bridge process is not working. (PID: {pid})', True
        if first_ng_result.bridge_type == 'standard':
            return f'NG. A standard bridge process is not working. (PID: {pid})', True
        return f'NG. A bridge process is not working. (PID: {pid})', True

    if perf_results:
        return f'OK. A performance bridge is running. (PID: {perf_results[0].pid})', False
    if len(std_results) == 1:
        return f'OK. A standard bridge is running. (PID: {std_results[0].pid})', False
    pids = ', '.join(str(r.pid) for r in std_results)
    return f'OK. Standard bridges are running. (PIDs: {pids})', False


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
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            default=False,
            help='Print detailed output including bridge status',
        )

    def _collect_bridge_results(
        self,
        alive_pids: list[int],
        ipc_inode: int,
        timeout_sec: float,
    ) -> list[_BridgeResult]:
        results: list[_BridgeResult] = []
        for pid in alive_pids:
            sock_addr = _abstract_socket_addr_for_pid(ipc_inode, pid)
            recv_result = self._socket_class(sock_addr).recv_msg(timeout_sec)

            if isinstance(recv_result, RecvMsgFailed):
                results.append(_BridgeResult(
                    pid=pid,
                    ipc_ns=ipc_inode,
                    bridge_type='unknown',
                    is_ok=False,
                    ng_message=_NG_MSG_RECV_FAILED,
                ))
                continue

            if isinstance(recv_result, RecvMsgParseError):
                results.append(_BridgeResult(
                    pid=pid,
                    ipc_ns=ipc_inode,
                    bridge_type='unknown',
                    is_ok=False,
                    ng_message=_NG_MSG_PARSE_ERROR,
                ))
                continue

            bridge_type = recv_result.type
            if bridge_type not in ('performance', 'standard'):
                results.append(_BridgeResult(
                    pid=pid,
                    ipc_ns=ipc_inode,
                    bridge_type='unknown',
                    is_ok=False,
                    ng_message=_NG_MSG_UNKNOWN_TYPE,
                ))
                continue

            if recv_result.pid != pid or recv_result.ipc_ns != ipc_inode:
                results.append(_BridgeResult(
                    pid=pid,
                    ipc_ns=ipc_inode,
                    bridge_type=bridge_type,
                    is_ok=False,
                    ng_message=(
                        f'Response contains mismatched data: '
                        f'pid={recv_result.pid}, ipc_ns={recv_result.ipc_ns}'
                    ),
                ))
                continue

            results.append(_BridgeResult(
                pid=pid,
                ipc_ns=ipc_inode,
                bridge_type=bridge_type,
                is_ok=True,
            ))
        return results

    def main(self, *, args):
        ipc_inode = _get_self_ipc_namespace_inode()
        timeout_sec = args.timeout
        verbose = args.verbose

        bridge_pids = _find_bridge_pids(ipc_inode)

        results = self._collect_bridge_results(bridge_pids, ipc_inode, timeout_sec)

        summary, is_ng = _build_summary(results)

        # output
        if verbose:
            print(f'IPC namespace inode: {ipc_inode}')
            print()

        if verbose:
            print('Summary:')
            print(f'  {summary}')
        else:
            print(summary)

        if verbose and results:
            print()
            print('Bridge Status:')
            type_width = max(len(f'({r.bridge_type.capitalize()})') for r in results)
            pid_width = max(len(str(r.pid)) for r in results)
            for r in results:
                type_label = f'({r.bridge_type.capitalize()})'.ljust(type_width)
                pid_str = str(r.pid).rjust(pid_width)
                if r.is_ok:
                    print(f'  PID {pid_str} {type_label} OK')
                else:
                    print(f'  PID {pid_str} {type_label} NG: {r.ng_message}')

        return 1 if is_ng else 0
