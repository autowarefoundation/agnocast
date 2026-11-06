"""Pure-logic tests for the bridge verb.

The verb itself talks to /proc and Unix domain sockets, but the helpers are
exercised here with mocked /proc/net/unix content and in-process abstract
socket servers in place of the real bridge daemon.
"""

import os
import socket
import threading
from types import SimpleNamespace
from unittest.mock import mock_open, patch

import pytest

from ros2agnocast.verb import agnocast_bridge_daemon_status as br
from ros2agnocast.verb.agnocast_bridge_daemon_status import (
    _find_bridge_pids,
    BridgeControlSocket,
    BridgeDaemonStatusVerb,
    RecvMsgFailed,
    RecvMsgOk,
    RecvMsgParseError,
)


# ---------------------------------------------------------------------------
# _find_bridge_pids  (parses /proc/net/unix)
# ---------------------------------------------------------------------------

_PROC_UNIX_HEADER = 'Num       RefCount Protocol Flags    Type St Inode Path\n'


def _make_proc_unix(*entries: str) -> str:
    """Build a fake /proc/net/unix content with the given Path entries."""
    lines = [_PROC_UNIX_HEADER]
    for path in entries:
        lines.append(f'0000000000000000: 00000002 00000000 00010000 0001 01 12345 {path}\n')
    return ''.join(lines)


def test_find_bridge_pids_empty_when_no_matching_entries():
    ns_inode = 12345
    content = _make_proc_unix('@other/12345/100', '@agnocast_bridge/99999/200')
    with patch('builtins.open', mock_open(read_data=content)):
        pids = _find_bridge_pids(ns_inode)
    assert pids == []


def test_find_bridge_pids_returns_sorted_pids():
    ns_inode = 12345
    content = _make_proc_unix(
        f'@agnocast_bridge/{ns_inode}/300',
        f'@agnocast_bridge/{ns_inode}/100',
        f'@agnocast_bridge/{ns_inode}/200',
    )
    with patch('builtins.open', mock_open(read_data=content)):
        pids = _find_bridge_pids(ns_inode)
    assert pids == [100, 200, 300]


def test_find_bridge_pids_skips_non_numeric_pid_suffix():
    ns_inode = 12345
    content = _make_proc_unix(
        f'@agnocast_bridge/{ns_inode}/123',
        f'@agnocast_bridge/{ns_inode}/abc',
    )
    with patch('builtins.open', mock_open(read_data=content)):
        pids = _find_bridge_pids(ns_inode)
    assert pids == [123]


def test_find_bridge_pids_empty_on_oserror():
    ns_inode = 12345
    with patch('builtins.open', side_effect=OSError):
        pids = _find_bridge_pids(ns_inode)
    assert pids == []


# ---------------------------------------------------------------------------
# BridgeControlSocket.recv_msg  (real UNIX socket)
# ---------------------------------------------------------------------------

def _start_server(abstract_addr: str, payload: bytes | None) -> threading.Thread:
    """Start a minimal one-shot server on an abstract socket that accepts one connection."""
    ready = threading.Event()

    def _run() -> None:
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(abstract_addr)
        srv.listen(1)
        ready.set()
        conn, _ = srv.accept()
        if payload is not None:
            conn.sendall(payload)
        conn.close()
        srv.close()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    ready.wait(timeout=2.0)
    return t


def _unique_abstract_addr(suffix: str = '') -> str:
    """Return a unique abstract socket address for testing."""
    return f'\x00agnocast_bridge_test_{os.getpid()}_{threading.get_ident()}{suffix}'


def test_recv_msg_returns_ok_for_valid_bridge_json():
    addr = _unique_abstract_addr('_bridge')
    payload = b'{"type":"bridge","ipc_ns":99,"domain_id":7,"pid":456}'
    t = _start_server(addr, payload)
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgOk)
    assert result.type == 'bridge'
    assert result.ipc_ns == 99
    assert result.domain_id == 7
    assert result.pid == 456


def test_recv_msg_returns_failed_when_no_server():
    addr = _unique_abstract_addr('_noserver')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=0.1)
    assert isinstance(result, RecvMsgFailed)


def test_recv_msg_returns_failed_when_server_sends_nothing():
    addr = _unique_abstract_addr('_empty')
    t = _start_server(addr, payload=None)
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgFailed)


def test_recv_msg_returns_parse_error_for_invalid_json():
    addr = _unique_abstract_addr('_badjson')
    t = _start_server(addr, payload=b'not-json')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


def test_recv_msg_returns_parse_error_for_missing_fields():
    addr = _unique_abstract_addr('_missingfields')
    t = _start_server(addr, payload=b'{"type":"bridge"}')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


# ---------------------------------------------------------------------------
# BridgeDaemonStatusVerb._collect_bridge_results
# ---------------------------------------------------------------------------

def _make_verb(result_map: dict) -> BridgeDaemonStatusVerb:
    """Build a verb whose _socket_class returns pre-set recv results keyed by abstract addr."""
    class _MockSocketClass:
        def __init__(self, sock_addr: str) -> None:
            self._result = result_map[sock_addr]

        def recv_msg(self, timeout_sec: float):
            return self._result

    verb = BridgeDaemonStatusVerb()
    verb._socket_class = _MockSocketClass
    return verb


def test_collect_bridge_results_ok_bridge():
    ipc_inode, pid, domain_id = 100, 200, 7
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    recv_msg = RecvMsgOk(type='bridge', ipc_ns=ipc_inode, domain_id=domain_id, pid=pid)
    verb = _make_verb({sock_addr: recv_msg})
    _, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is True


@pytest.mark.parametrize('legacy_type', ['standard', 'performance'])
def test_collect_bridge_results_legacy_type_gives_legacy_ng(legacy_type):
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    recv_msg = RecvMsgOk(type=legacy_type, ipc_ns=ipc_inode, domain_id=7, pid=pid)
    verb = _make_verb({sock_addr: recv_msg})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert br._NG_MSG_LEGACY_TYPE in results[0]


def test_collect_bridge_results_recv_failed_gives_ng():
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    verb = _make_verb({sock_addr: RecvMsgFailed()})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert br._NG_MSG_RECV_FAILED in results[0]


def test_collect_bridge_results_parse_error_gives_ng():
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    verb = _make_verb({sock_addr: RecvMsgParseError()})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert br._NG_MSG_PARSE_ERROR in results[0]


def test_collect_bridge_results_unknown_bridge_type_gives_ng():
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    recv_msg = RecvMsgOk(type='future_type', ipc_ns=ipc_inode, domain_id=7, pid=pid)
    verb = _make_verb({sock_addr: recv_msg})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert br._NG_MSG_UNKNOWN_TYPE in results[0]


def test_collect_bridge_results_mismatched_pid_gives_ng():
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    recv_msg = RecvMsgOk(type='bridge', ipc_ns=ipc_inode, domain_id=7, pid=999)
    verb = _make_verb({sock_addr: recv_msg})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert 'pid=999' in results[0]


def test_collect_bridge_results_mismatched_ipc_ns_gives_ng():
    ipc_inode, pid = 100, 200
    sock_addr = br._abstract_socket_addr_for_pid(ipc_inode, pid)
    recv_msg = RecvMsgOk(type='bridge', ipc_ns=999, domain_id=7, pid=pid)
    verb = _make_verb({sock_addr: recv_msg})
    results, ok = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert 'ipc_ns=999' in results[0]


def test_collect_bridge_results_duplicate_ros_domain_gives_ng():
    ipc_inode, first_pid, second_pid, domain_id = 100, 200, 201, 7
    first_addr = br._abstract_socket_addr_for_pid(ipc_inode, first_pid)
    second_addr = br._abstract_socket_addr_for_pid(ipc_inode, second_pid)
    verb = _make_verb({
        first_addr: RecvMsgOk(type='bridge', ipc_ns=ipc_inode, domain_id=domain_id, pid=first_pid),
        second_addr: RecvMsgOk(
            type='bridge', ipc_ns=ipc_inode, domain_id=domain_id, pid=second_pid
        ),
    })
    results, ok = verb._collect_bridge_results([first_pid, second_pid], ipc_inode, timeout_sec=1.0)
    assert ok is False
    assert f'ROS domain {domain_id}' in "".join(results)


def test_collect_bridge_results_different_ros_domains_are_ok():
    ipc_inode, first_pid, second_pid = 100, 200, 201
    first_addr = br._abstract_socket_addr_for_pid(ipc_inode, first_pid)
    second_addr = br._abstract_socket_addr_for_pid(ipc_inode, second_pid)
    verb = _make_verb({
        first_addr: RecvMsgOk(type='bridge', ipc_ns=ipc_inode, domain_id=7, pid=first_pid),
        second_addr: RecvMsgOk(type='bridge', ipc_ns=ipc_inode, domain_id=8, pid=second_pid),
    })
    _, ok = verb._collect_bridge_results([first_pid, second_pid], ipc_inode, timeout_sec=1.0)
    assert ok is True
