"""Pure-logic tests for the bridge verb.

The verb itself talks to /proc and Unix domain sockets, but the helpers are
exercised here with a tmp directory in place of CONTROL_SOCKET_BASE_DIR and
in-process socket servers in place of the real bridge daemon.
"""

import os
import socket
import tempfile
import threading
from unittest.mock import patch

from ros2agnocast.verb import bridge_daemon_status as br
from ros2agnocast.verb.bridge_daemon_status import (
    _BridgeResult,
    _build_summary,
    _find_bridge_pids,
    _partition_pids,
    BridgeControlSocket,
    BridgeDaemonStatusVerb,
    RecvMsgFailed,
    RecvMsgOk,
    RecvMsgParseError,
)


# ---------------------------------------------------------------------------
# _find_bridge_pids
# ---------------------------------------------------------------------------

def test_find_bridge_pids_empty_when_no_sock_files():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        os.makedirs(os.path.join(tmpdir, str(ns_inode)))
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            pids = _find_bridge_pids(ns_inode)
    assert pids == []


def test_find_bridge_pids_returns_sorted_pids():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        ns_dir = os.path.join(tmpdir, str(ns_inode))
        os.makedirs(ns_dir)
        for pid in (300, 100, 200):
            open(os.path.join(ns_dir, f'{pid}.sock'), 'w').close()
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            pids = _find_bridge_pids(ns_inode)
    assert pids == [100, 200, 300]


def test_find_bridge_pids_skips_non_numeric_filenames():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        ns_dir = os.path.join(tmpdir, str(ns_inode))
        os.makedirs(ns_dir)
        open(os.path.join(ns_dir, '123.sock'), 'w').close()
        open(os.path.join(ns_dir, 'abc.sock'), 'w').close()
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            pids = _find_bridge_pids(ns_inode)
    assert pids == [123]


def test_find_bridge_pids_empty_when_ns_dir_missing():
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            pids = _find_bridge_pids(99999)
    assert pids == []


# ---------------------------------------------------------------------------
# _partition_pids
# ---------------------------------------------------------------------------

def test_partition_pids_self_pid_is_alive():
    alive, stale = _partition_pids([os.getpid()])
    assert alive == [os.getpid()]
    assert stale == []


def test_partition_pids_non_existent_pid_is_stale():
    alive, stale = _partition_pids([999999999])
    assert alive == []
    assert stale == [999999999]


def test_partition_pids_mixed():
    alive, stale = _partition_pids([os.getpid(), 999999999])
    assert alive == [os.getpid()]
    assert stale == [999999999]


# ---------------------------------------------------------------------------
# _build_summary
# ---------------------------------------------------------------------------

def _ok(pid: int, bridge_type: str) -> _BridgeResult:
    return _BridgeResult(pid=pid, ipc_ns=1, bridge_type=bridge_type, is_ok=True)


def _ng(pid: int, bridge_type: str, msg: str = 'error') -> _BridgeResult:
    return _BridgeResult(pid=pid, ipc_ns=1, bridge_type=bridge_type, is_ok=False, ng_message=msg)


def test_build_summary_no_results_is_ng():
    summary, is_ng = _build_summary([])
    assert is_ng is True
    assert 'no bridge' in summary.lower()


def test_build_summary_single_performance_ok():
    summary, is_ng = _build_summary([_ok(100, 'performance')])
    assert is_ng is False
    assert 'performance' in summary.lower()
    assert '100' in summary


def test_build_summary_single_standard_ok():
    summary, is_ng = _build_summary([_ok(200, 'standard')])
    assert is_ng is False
    assert 'standard' in summary.lower()
    assert '200' in summary


def test_build_summary_multiple_standard_ok():
    summary, is_ng = _build_summary([_ok(200, 'standard'), _ok(201, 'standard')])
    assert is_ng is False
    assert '200' in summary
    assert '201' in summary


def test_build_summary_multiple_performance_is_ng():
    summary, is_ng = _build_summary([_ok(100, 'performance'), _ok(101, 'performance')])
    assert is_ng is True
    assert 'multiple' in summary.lower()
    assert '100' in summary
    assert '101' in summary


def test_build_summary_mixed_performance_and_standard_is_ng():
    summary, is_ng = _build_summary([_ok(100, 'performance'), _ok(200, 'standard')])
    assert is_ng is True
    assert 'both' in summary.lower()


def test_build_summary_performance_ng():
    summary, is_ng = _build_summary([_ng(100, 'performance')])
    assert is_ng is True
    assert 'performance' in summary.lower()
    assert '100' in summary


def test_build_summary_standard_ng():
    summary, is_ng = _build_summary([_ng(200, 'standard')])
    assert is_ng is True
    assert 'standard' in summary.lower()
    assert '200' in summary


def test_build_summary_unknown_type_ng():
    summary, is_ng = _build_summary([_ng(300, 'unknown')])
    assert is_ng is True
    assert '300' in summary


# ---------------------------------------------------------------------------
# BridgeControlSocket.recv_msg  (real UNIX socket)
# ---------------------------------------------------------------------------

def _start_server(sock_path: str, payload: bytes | None) -> threading.Thread:
    """Start a minimal one-shot server that accepts one connection, sends payload, closes."""
    ready = threading.Event()

    def _run() -> None:
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(sock_path)
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


def test_recv_msg_returns_ok_for_valid_standard_json():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'test.sock')
        payload = b'{"type":"standard","ipc_ns":42,"pid":123}'
        t = _start_server(sock_path, payload)
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=2.0)
        t.join(timeout=2.0)
    assert isinstance(result, RecvMsgOk)
    assert result.type == 'standard'
    assert result.ipc_ns == 42
    assert result.pid == 123


def test_recv_msg_returns_ok_for_valid_performance_json():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'test.sock')
        payload = b'{"type":"performance","ipc_ns":99,"pid":456}'
        t = _start_server(sock_path, payload)
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=2.0)
        t.join(timeout=2.0)
    assert isinstance(result, RecvMsgOk)
    assert result.type == 'performance'
    assert result.ipc_ns == 99
    assert result.pid == 456


def test_recv_msg_returns_failed_when_no_server():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'nonexistent.sock')
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=0.1)
    assert isinstance(result, RecvMsgFailed)


def test_recv_msg_returns_failed_when_server_sends_nothing():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'test.sock')
        t = _start_server(sock_path, payload=None)
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=2.0)
        t.join(timeout=2.0)
    assert isinstance(result, RecvMsgFailed)


def test_recv_msg_returns_parse_error_for_invalid_json():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'test.sock')
        t = _start_server(sock_path, payload=b'not-json')
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=2.0)
        t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


def test_recv_msg_returns_parse_error_for_missing_fields():
    with tempfile.TemporaryDirectory() as tmpdir:
        sock_path = os.path.join(tmpdir, 'test.sock')
        t = _start_server(sock_path, payload=b'{"type":"standard"}')
        result = BridgeControlSocket(sock_path).recv_msg(timeout_sec=2.0)
        t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


# ---------------------------------------------------------------------------
# BridgeDaemonStatusVerb._collect_bridge_results
# ---------------------------------------------------------------------------

def _make_verb(result_map: dict) -> BridgeDaemonStatusVerb:
    """Build a verb whose _socket_class returns pre-set recv results keyed by sock_path."""
    class _MockSocketClass:
        def __init__(self, sock_path: str) -> None:
            self._result = result_map[sock_path]

        def recv_msg(self, timeout_sec: float):
            return self._result

    verb = BridgeDaemonStatusVerb()
    verb._socket_class = _MockSocketClass
    return verb


def test_collect_bridge_results_ok_standard():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb({sock_path: RecvMsgOk(type='standard', ipc_ns=ipc_inode, pid=pid)})
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert len(results) == 1
    assert results[0].is_ok is True
    assert results[0].bridge_type == 'standard'


def test_collect_bridge_results_ok_performance():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb({sock_path: RecvMsgOk(type='performance', ipc_ns=ipc_inode, pid=pid)})
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert len(results) == 1
    assert results[0].is_ok is True
    assert results[0].bridge_type == 'performance'


def test_collect_bridge_results_recv_failed_gives_unknown_ng():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb({sock_path: RecvMsgFailed()})
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert results[0].is_ok is False
    assert results[0].bridge_type == 'unknown'
    assert results[0].ng_message == br._NG_MSG_RECV_FAILED


def test_collect_bridge_results_parse_error_gives_unknown_ng():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb({sock_path: RecvMsgParseError()})
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert results[0].is_ok is False
    assert results[0].bridge_type == 'unknown'
    assert results[0].ng_message == br._NG_MSG_PARSE_ERROR


def test_collect_bridge_results_unknown_bridge_type_gives_ng():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb(
                {sock_path: RecvMsgOk(type='future_type', ipc_ns=ipc_inode, pid=pid)}
            )
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert results[0].is_ok is False
    assert results[0].bridge_type == 'unknown'
    assert results[0].ng_message == br._NG_MSG_UNKNOWN_TYPE


def test_collect_bridge_results_mismatched_pid_gives_ng():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb(
                {sock_path: RecvMsgOk(type='standard', ipc_ns=ipc_inode, pid=999)}
            )
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert results[0].is_ok is False
    assert 'pid=999' in results[0].ng_message


def test_collect_bridge_results_mismatched_ipc_ns_gives_ng():
    ipc_inode, pid = 100, 200
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(br, 'CONTROL_SOCKET_BASE_DIR', tmpdir):
            sock_path = br._sock_path_for_pid(ipc_inode, pid)
            verb = _make_verb(
                {sock_path: RecvMsgOk(type='standard', ipc_ns=999, pid=pid)}
            )
            results = verb._collect_bridge_results([pid], ipc_inode, timeout_sec=1.0)
    assert results[0].is_ok is False
    assert 'ipc_ns=999' in results[0].ng_message
