"""Tests for the bridge daemon status verb.

An in-process abstract socket server is used to simulate the bridge daemon.
"""

import os
import socket
import threading
from types import SimpleNamespace

from ros2agnocast.verb import agnocast_bridge_daemon_status as br
from ros2agnocast.verb.agnocast_bridge_daemon_status import (
    BridgeControlSocket,
    BridgeDaemonStatusVerb,
    RecvMsgNotRunning,
    RecvMsgOk,
    RecvMsgParseError,
    RecvMsgUnresponsive,
)

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


def test_recv_msg_returns_ok_for_valid_json():
    addr = _unique_abstract_addr('_bridge')
    t = _start_server(addr, b'{"pid":456}')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgOk)
    assert result.pid == 456


def test_recv_msg_returns_not_running_when_no_server():
    addr = _unique_abstract_addr('_noserver')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=0.1)
    assert isinstance(result, RecvMsgNotRunning)


def test_recv_msg_returns_unresponsive_when_server_sends_nothing():
    addr = _unique_abstract_addr('_empty')
    t = _start_server(addr, payload=None)
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgUnresponsive)


def test_recv_msg_returns_parse_error_for_invalid_json():
    addr = _unique_abstract_addr('_badjson')
    t = _start_server(addr, payload=b'not-json')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


def test_recv_msg_returns_parse_error_for_missing_pid():
    addr = _unique_abstract_addr('_missingfields')
    t = _start_server(addr, payload=b'{"type":"bridge"}')
    result = BridgeControlSocket(addr).recv_msg(timeout_sec=2.0)
    t.join(timeout=2.0)
    assert isinstance(result, RecvMsgParseError)


# ---------------------------------------------------------------------------
# BridgeDaemonStatusVerb.main
# ---------------------------------------------------------------------------

def _make_verb(result) -> BridgeDaemonStatusVerb:
    """Build a verb whose _socket_class returns a pre-set recv result."""

    class _MockSocketClass:
        def __init__(self, sock_addr: str) -> None:
            pass

        def recv_msg(self, timeout_sec: float):
            return result

    verb = BridgeDaemonStatusVerb()
    verb._socket_class = _MockSocketClass
    return verb


def test_main_ok_verdict_and_exit_code(capsys):
    verb = _make_verb(RecvMsgOk(pid=100))
    ret = verb.main(args=SimpleNamespace(timeout=1.0, verbose=False))
    out = capsys.readouterr().out
    assert ret == 0
    assert 'OK' in out
    assert 'PID: 100' in out


def test_main_not_running_verdict_and_exit_code(capsys):
    verb = _make_verb(RecvMsgNotRunning())
    ret = verb.main(args=SimpleNamespace(timeout=1.0, verbose=False))
    out = capsys.readouterr().out
    assert ret == 1
    assert 'NG' in out


def test_main_unresponsive_verdict_and_exit_code(capsys):
    verb = _make_verb(RecvMsgUnresponsive())
    ret = verb.main(args=SimpleNamespace(timeout=1.0, verbose=False))
    out = capsys.readouterr().out
    assert ret == 1
    assert 'NG' in out


def test_main_parse_error_verdict_and_exit_code(capsys):
    verb = _make_verb(RecvMsgParseError())
    ret = verb.main(args=SimpleNamespace(timeout=1.0, verbose=False))
    out = capsys.readouterr().out
    assert ret == 1
    assert 'NG' in out


def test_main_verbose_prints_context(capsys, monkeypatch):
    monkeypatch.setattr(br, '_self_ipc_ns_inode', lambda: 12345)
    monkeypatch.setattr(br, '_read_ros_domain_id', lambda: 0)

    verb = _make_verb(RecvMsgOk(pid=100))
    ret = verb.main(args=SimpleNamespace(timeout=1.0, verbose=True))
    out = capsys.readouterr().out
    assert ret == 0
    assert 'IPC namespace inode: 12345' in out
    assert 'ROS_DOMAIN_ID: 0' in out
