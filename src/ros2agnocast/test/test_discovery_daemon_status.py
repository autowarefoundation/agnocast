"""Pure-logic tests for the discovery_daemon_status verb.

The verb itself talks to /proc and DDS, but the small helpers are exercised
here with a tmp directory in place of `/dev/shm`.
"""

import fcntl
import os
import tempfile
from unittest.mock import patch

from ros2agnocast.verb import discovery_daemon_status as ds


# --- type_registry: informational description (no OK/NG) --------------------

def test_describe_type_registry_missing_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(ds, '_type_registry_base', return_value=tmpdir):
            detail = ds._describe_type_registry(999999)
        assert 'no Agnocast process has registered yet' in detail


def test_describe_type_registry_empty_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        os.makedirs(os.path.join(tmpdir, str(ns_inode)))
        with patch.object(ds, '_type_registry_base', return_value=tmpdir):
            detail = ds._describe_type_registry(ns_inode)
        assert 'no Agnocast process has registered yet' in detail


def test_describe_type_registry_with_live_pid_reports_count():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        ns_dir = os.path.join(tmpdir, str(ns_inode))
        os.makedirs(ns_dir)
        # Name the file after our own PID so /proc/<pid> is guaranteed live.
        with open(os.path.join(ns_dir, f'{os.getpid()}.txt'), 'w') as fp:
            fp.write('/topic\ttype\tpub\t/node\n')
        with patch.object(ds, '_type_registry_base', return_value=tmpdir):
            detail = ds._describe_type_registry(ns_inode)
        assert '1 live registration' in detail


def test_describe_type_registry_stale_pid_noted():
    """A <pid>.txt whose process is gone is noted as stale, not live."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        ns_dir = os.path.join(tmpdir, str(ns_inode))
        os.makedirs(ns_dir)
        # PID 999999999 is unlikely to be alive.
        with open(os.path.join(ns_dir, '999999999.txt'), 'w') as fp:
            fp.write('/topic\ttype\tpub\t/node\n')
        with patch.object(ds, '_type_registry_base', return_value=tmpdir):
            detail = ds._describe_type_registry(ns_inode)
        assert 'stale' in detail
        assert 'no Agnocast process has registered yet' in detail


# --- daemon process: probe the agent's singleton flock ----------------------

def test_check_daemon_process_missing_lock_is_ng():
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.dict(os.environ, {'AGNOCAST_TMPFS_DIR': tmpdir}):
            ok, detail = ds._check_daemon_process(424242)
        assert ok is False
        assert 'no agent lock' in detail


def test_check_daemon_process_free_lock_is_ng():
    """Lock file exists but nobody holds it -> no live agent."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 424242
        open(os.path.join(tmpdir, f'agnocast_discovery_agent_{ns_inode}.lock'), 'w').close()
        with patch.dict(os.environ, {'AGNOCAST_TMPFS_DIR': tmpdir}):
            ok, detail = ds._check_daemon_process(ns_inode)
        assert ok is False
        assert 'free' in detail


def test_check_daemon_process_held_lock_is_ok():
    """A held flock (as the real agent holds it) -> OK."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 424242
        lock_path = os.path.join(tmpdir, f'agnocast_discovery_agent_{ns_inode}.lock')
        holder = open(lock_path, 'w')
        try:
            fcntl.flock(holder.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.dict(os.environ, {'AGNOCAST_TMPFS_DIR': tmpdir}):
                ok, detail = ds._check_daemon_process(ns_inode)
            assert ok is True
            assert 'holds the singleton lock' in detail
        finally:
            holder.close()


def test_singleton_lock_path_honors_agnocast_tmpfs_dir(monkeypatch):
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', '/run/custom')
    assert ds._singleton_lock_path(7) == '/run/custom/agnocast_discovery_agent_7.lock'

    monkeypatch.delenv('AGNOCAST_TMPFS_DIR', raising=False)
    assert ds._singleton_lock_path(7) == '/dev/shm/agnocast_discovery_agent_7.lock'


def test_type_registry_base_honors_agnocast_tmpfs_dir(monkeypatch):
    """`AGNOCAST_TMPFS_DIR` overrides the `/dev/shm` default consistently with the writer."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', '/run/custom')
    assert ds._type_registry_base() == '/run/custom/agnocast_type_registry'

    monkeypatch.delenv('AGNOCAST_TMPFS_DIR', raising=False)
    assert ds._type_registry_base() == '/dev/shm/agnocast_type_registry'
