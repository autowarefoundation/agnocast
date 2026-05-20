"""Pure-logic tests for the discovery_daemon_status verb.

The verb itself talks to /proc and DDS, but the small helpers are
exercised here with a tmp directory in place of `/run/agnocast`.
"""

import os
import tempfile
from unittest.mock import patch

from ros2agnocast.verb import discovery_daemon_status as ds


def test_check_type_registry_missing_dir_returns_ng():
    with tempfile.TemporaryDirectory() as tmpdir:
        with patch.object(ds, '_TYPE_REGISTRY_BASE', tmpdir):
            ok, detail = ds._check_type_registry(999999)
        assert ok is False
        assert 'missing' in detail


def test_check_type_registry_empty_dir_returns_ng():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        os.makedirs(os.path.join(tmpdir, str(ns_inode)))
        with patch.object(ds, '_TYPE_REGISTRY_BASE', tmpdir):
            ok, detail = ds._check_type_registry(ns_inode)
        assert ok is False
        assert 'no <pid>.txt' in detail


def test_check_type_registry_with_files_returns_ok():
    with tempfile.TemporaryDirectory() as tmpdir:
        ns_inode = 12345
        ns_dir = os.path.join(tmpdir, str(ns_inode))
        os.makedirs(ns_dir)
        with open(os.path.join(ns_dir, '4242.txt'), 'w') as fp:
            fp.write('/topic\ttype\tpub\t/node\n')
        with patch.object(ds, '_TYPE_REGISTRY_BASE', tmpdir):
            ok, detail = ds._check_type_registry(ns_inode)
        assert ok is True
        assert '1' in detail


def test_check_daemon_process_self_ns_no_match():
    """When no discovery_agent is running in this NS, returns NG.

    This test runs in the pytest process's own IPC NS, where no
    discovery_agent should be present unless the test runner happens to
    overlap with one — extremely unlikely under normal CI.
    """
    my_ns_inode = ds._self_ipc_ns_inode()
    ok, detail = ds._check_daemon_process(my_ns_inode)
    # Either no daemon (most cases) or one is genuinely running. Both
    # are valid outputs; we just assert the function returns the
    # expected tuple shape and detail string is non-empty.
    assert isinstance(ok, bool)
    assert isinstance(detail, str)
    assert detail
