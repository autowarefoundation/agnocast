"""Unit tests for the discovery agent's TypeRegistryReader.

The reader scans a per-namespace tmpfs directory of `<pid>.txt` files
written by `agnocastlib::internal::TypeRegistryWriter`. These tests use
a temporary directory as the base — no kmod, no rclpy.
"""

import os
import tempfile
import textwrap

from ros2agnocast_discovery_agent.type_registry import (
    RegistryEntry,
    TypeRegistryReader,
)


def _write(path: str, content: str) -> None:
    with open(path, 'w', encoding='utf-8') as fp:
        fp.write(content)


def _make_reader(tmpdir: str, ns_inode: int = 1234) -> TypeRegistryReader:
    os.makedirs(os.path.join(tmpdir, str(ns_inode)), exist_ok=True)
    return TypeRegistryReader(ipc_ns_inode=ns_inode, base_dir=tmpdir)


def test_rebuild_parses_well_formed_lines():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(
            os.path.join(reader.ns_dir, '1111.txt'),
            '/chatter\tstd_msgs/msg/Int32\tpub\t/talker_node\n'
            '/echo\tstd_msgs/msg/String\tsub\t/listener_node\n')
        reader.rebuild()

        assert reader.lookup('/chatter', 'pub', '/talker_node') == RegistryEntry(
            pid=1111, type_name='std_msgs/msg/Int32')
        assert reader.lookup('/echo', 'sub', '/listener_node') == RegistryEntry(
            pid=1111, type_name='std_msgs/msg/String')


def test_rebuild_replaces_previous_table():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        path = os.path.join(reader.ns_dir, '1111.txt')
        _write(path, '/old\tstd_msgs/msg/Int32\tpub\t/n\n')
        reader.rebuild()
        assert reader.lookup('/old', 'pub', '/n') is not None

        os.unlink(path)
        reader.rebuild()
        assert reader.lookup('/old', 'pub', '/n') is None


def test_rebuild_handles_missing_ns_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = TypeRegistryReader(ipc_ns_inode=99999, base_dir=tmpdir)
        reader.rebuild()  # should not raise
        assert reader.table == {}


def test_rebuild_skips_unterminated_tail():
    """A writer dying mid-write leaves a tail without `\\n`. Skip it."""
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(
            os.path.join(reader.ns_dir, '1.txt'),
            '/good\tstd_msgs/msg/Int32\tpub\t/node_a\n'
            '/partial\tstd_msgs/msg/Float64\tpub')
        reader.rebuild()

        assert reader.lookup('/good', 'pub', '/node_a') is not None
        # The partial line must be dropped.
        assert reader.lookup('/partial', 'pub', '') is None


def test_rebuild_skips_lines_with_too_few_fields(caplog):
    """Lines with fewer than 4 tab-separated fields are dropped + warned."""
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(
            os.path.join(reader.ns_dir, '1.txt'),
            '/good\tstd_msgs/msg/Int32\tpub\t/node_a\n'
            '/bad\tonly_two_fields\n')
        reader.rebuild()
        assert reader.lookup('/good', 'pub', '/node_a') is not None
        # No assertion on warning text since logger is optional.


def test_rebuild_accepts_extra_fields_forward_compat():
    """Lines with 5+ fields keep their first 4 fields and ignore extras."""
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(
            os.path.join(reader.ns_dir, '7.txt'),
            '/topic\tstd_msgs/msg/Int32\tpub\t/node_a\tfuture_field_v2\tand_one_more\n')
        reader.rebuild()
        entry = reader.lookup('/topic', 'pub', '/node_a')
        assert entry == RegistryEntry(pid=7, type_name='std_msgs/msg/Int32')


def test_rebuild_skips_lines_with_unknown_role():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(
            os.path.join(reader.ns_dir, '1.txt'),
            '/topic\tstd_msgs/msg/Int32\tunknown_role\t/node_a\n')
        reader.rebuild()
        assert reader.lookup('/topic', 'unknown_role', '/node_a') is None
        assert reader.lookup('/topic', 'pub', '/node_a') is None


def test_cleanup_dead_pids_removes_files_for_dead_pids():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        # PID 1 is always alive (init). PID 999999999 is unlikely to be alive.
        alive_path = os.path.join(reader.ns_dir, '1.txt')
        dead_path = os.path.join(reader.ns_dir, '999999999.txt')
        _write(alive_path, '/t\tT\tpub\t/n\n')
        _write(dead_path, '/t\tT\tpub\t/n\n')

        reader.cleanup_dead_pids()

        assert os.path.exists(alive_path)
        assert not os.path.exists(dead_path)


def test_cleanup_dead_pids_handles_missing_ns_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = TypeRegistryReader(ipc_ns_inode=99999, base_dir=tmpdir)
        reader.cleanup_dead_pids()  # should not raise


def test_rebuild_skips_non_numeric_filenames():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = _make_reader(tmpdir)
        _write(os.path.join(reader.ns_dir, 'README.txt'),
               '/topic\tstd_msgs/msg/Int32\tpub\t/node_a\n')
        reader.rebuild()
        # File ignored because its name doesn't parse as int.
        assert reader.table == {}
