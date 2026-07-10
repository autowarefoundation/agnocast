"""Pure-logic tests for the discovery_daemon_status verb.

The verb itself talks to /proc and DDS, but the small helpers and the verdict
logic are exercised here with tmp dirs and patched checks.
"""

import os
import tempfile
from argparse import Namespace
from unittest.mock import MagicMock, patch

from ros2agnocast.verb import agnocast_discovery_daemon_status as ds


# --- daemon process: query the kmod agent registry -------------------------

def _patch_exists(ret):
    """Patch the ioctl wrapper so agnocast_discovery_agent_exists returns ``ret``."""
    lib = MagicMock()
    lib.agnocast_discovery_agent_exists.return_value = ret
    return patch('ctypes.CDLL', return_value=lib)


def test_check_daemon_process_registered_is_ok():
    with _patch_exists(1):
        ok, reason = ds._check_daemon_process(0)
    assert ok is True
    assert 'registered in the kmod' in reason


def test_check_daemon_process_not_registered_is_ng():
    with _patch_exists(0):
        ok, reason = ds._check_daemon_process(3)
    assert ok is False
    assert 'no agent registered' in reason


def test_check_daemon_process_kmod_error_is_ng():
    """A negative ioctl return (module unloaded / ioctl failure) reads as not-running."""
    with _patch_exists(-1):
        ok, reason = ds._check_daemon_process(0)
    assert ok is False
    assert 'kmod query failed' in reason


def test_check_daemon_process_missing_symbol_is_ng():
    """An older/mismatched wrapper lacking the symbol reads as NG, not a CLI traceback."""
    lib = MagicMock(spec=[])  # any attribute access raises AttributeError, like an absent export
    with patch('ctypes.CDLL', return_value=lib):
        ok, reason = ds._check_daemon_process(0)
    assert ok is False
    assert 'version skew' in reason


def test_read_ros_domain_id(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    assert ds._read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', '')
    assert ds._read_ros_domain_id() == 0
    monkeypatch.setenv('ROS_DOMAIN_ID', '5')
    assert ds._read_ros_domain_id() == 5
    monkeypatch.setenv('ROS_DOMAIN_ID', 'notanint')
    assert ds._read_ros_domain_id() == 0


# --- verdict (main) ---------------------------------------------------------

def _run_main(proc, gossip, verbose=False):
    """Run the verb's main() with the checks patched. ``gossip=None`` asserts
    the gossip check is never invoked (process down)."""
    verb = ds.DiscoveryDaemonStatusVerb()
    args = Namespace(gossip_timeout=3.0, verbose=verbose)
    with patch.object(ds, 'warn_if_gossip_timeout_overridden', lambda a: None), \
            patch.object(ds, '_self_ipc_ns_inode', return_value=4026531839), \
            patch.object(ds, '_check_daemon_process', return_value=proc):
        if gossip is None:
            with patch.object(ds, '_check_gossip') as gossip_mock:
                rc = verb.main(args=args)
                gossip_mock.assert_not_called()
        else:
            with patch.object(ds, '_check_gossip', return_value=gossip):
                rc = verb.main(args=args)
    return rc


def test_verdict_running(capsys):
    rc = _run_main(proc=(True, 'registered in the kmod (domain 0)'),
                   gossip=(True, 'snapshot received on /_agnocast_discovery'))
    out = capsys.readouterr().out
    assert rc == 0
    assert 'OK. The discovery agent is running.' in out


def test_verdict_not_running_skips_gossip(capsys):
    rc = _run_main(proc=(False, 'no agent registered in the kmod (domain 0)'), gossip=None)
    out = capsys.readouterr().out
    assert rc == 1
    assert 'NG. The discovery agent is not running.' in out
    # The per-check reason is diagnostic detail, not shown by default.
    assert 'no agent registered' not in out


def test_verdict_running_but_not_publishing(capsys):
    rc = _run_main(proc=(True, 'registered in the kmod (domain 0)'),
                   gossip=(False, 'no snapshot on /_agnocast_discovery within 3.0s'))
    out = capsys.readouterr().out
    assert rc == 1
    assert 'running but not publishing' in out
    assert 'no snapshot' not in out  # reason is --verbose only


def test_default_output_is_verdict_only(capsys):
    """Default output carries no inode header or per-check reason."""
    _run_main(proc=(True, 'registered in the kmod (domain 0)'),
              gossip=(True, 'snapshot received on /_agnocast_discovery'))
    out = capsys.readouterr().out
    assert out.strip() == 'OK. The discovery agent is running.'


def test_verbose_shows_inode_and_checks(capsys):
    rc = _run_main(proc=(True, 'registered in the kmod (domain 0)'),
                   gossip=(True, 'snapshot received on /_agnocast_discovery'), verbose=True)
    out = capsys.readouterr().out
    assert rc == 0
    assert 'IPC namespace inode: 4026531839' in out
    assert 'registered in the kmod' in out  # per-check reason shown in --verbose
    assert 'OK. The discovery agent is running.' in out
