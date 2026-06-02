"""Pure-logic tests for the discovery_daemon_status verb.

The verb itself talks to /proc and DDS, but the small helpers and the verdict
logic are exercised here with tmp dirs and patched checks.
"""

import fcntl
import os
import tempfile
from argparse import Namespace
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
            ok, reason = ds._check_daemon_process(424242)
        assert ok is False
        assert 'no singleton lock file' in reason


def test_check_daemon_process_free_lock_is_ng():
    """Lock file exists but nobody holds it -> no live agent."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 424242
        open(os.path.join(tmpdir, f'agnocast_discovery_agent_{ns_inode}.lock'), 'w').close()
        with patch.dict(os.environ, {'AGNOCAST_TMPFS_DIR': tmpdir}):
            ok, reason = ds._check_daemon_process(ns_inode)
        assert ok is False
        assert 'free' in reason


def test_check_daemon_process_held_lock_is_ok():
    """A held flock (as the real agent holds it) -> OK."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 424242
        lock_path = os.path.join(tmpdir, f'agnocast_discovery_agent_{ns_inode}.lock')
        holder = open(lock_path, 'w')
        try:
            fcntl.flock(holder.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.dict(os.environ, {'AGNOCAST_TMPFS_DIR': tmpdir}):
                ok, reason = ds._check_daemon_process(ns_inode)
            assert ok is True
            assert 'holds the singleton lock' in reason
        finally:
            holder.close()


# --- path helpers -----------------------------------------------------------

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


# --- verdict (main) ---------------------------------------------------------

def _run_main(proc, gossip, verbose=False):
    """Run the verb's main() with the checks patched. ``gossip=None`` asserts
    the gossip check is never invoked (process down)."""
    verb = ds.DiscoveryDaemonStatusVerb()
    args = Namespace(gossip_timeout=3.0, verbose=verbose)
    with patch.object(ds, 'warn_if_gossip_timeout_overridden', lambda a: None), \
            patch.object(ds, '_self_ipc_ns_inode', return_value=4026531839), \
            patch.object(ds, '_check_daemon_process', return_value=proc), \
            patch.object(ds, '_describe_type_registry', return_value='2 live registration(s) in /x'):
        if gossip is None:
            with patch.object(ds, '_check_gossip') as gossip_mock:
                rc = verb.main(args=args)
                gossip_mock.assert_not_called()
        else:
            with patch.object(ds, '_check_gossip', return_value=gossip):
                rc = verb.main(args=args)
    return rc


def test_verdict_running(capsys):
    rc = _run_main(proc=(True, 'holds the singleton lock (/x.lock)'),
                   gossip=(True, 'snapshot received on /_agnocast_discovery'))
    out = capsys.readouterr().out
    assert rc == 0
    assert 'OK. The discovery agent is running.' in out


def test_verdict_not_running_skips_gossip(capsys):
    rc = _run_main(proc=(False, 'singleton lock is free (/x.lock)'), gossip=None)
    out = capsys.readouterr().out
    assert rc == 1
    assert 'NG. The discovery agent is not running.' in out
    # The per-check reason is diagnostic detail, not shown by default.
    assert 'singleton lock is free' not in out


def test_verdict_running_but_not_publishing(capsys):
    rc = _run_main(proc=(True, 'holds the singleton lock (/x.lock)'),
                   gossip=(False, 'no snapshot on /_agnocast_discovery within 3.0s'))
    out = capsys.readouterr().out
    assert rc == 1
    assert 'running but not publishing' in out
    assert 'no snapshot' not in out  # reason is --verbose only


def test_default_output_is_verdict_only(capsys):
    """Default output carries no inode header, type_registry, or per-check reason."""
    _run_main(proc=(True, 'holds the singleton lock (/x.lock)'),
              gossip=(True, 'snapshot received on /_agnocast_discovery'))
    out = capsys.readouterr().out
    assert out.strip() == 'OK. The discovery agent is running.'


def test_verbose_shows_inode_checks_and_type_registry(capsys):
    rc = _run_main(proc=(True, 'holds the singleton lock (/x.lock)'),
                   gossip=(True, 'snapshot received on /_agnocast_discovery'), verbose=True)
    out = capsys.readouterr().out
    assert rc == 0
    assert 'IPC namespace inode: 4026531839' in out
    assert 'type_registry: 2 live registration(s)' in out
    assert 'holds the singleton lock' in out  # per-check reason shown in --verbose
    assert 'OK. The discovery agent is running.' in out
