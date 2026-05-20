"""Read agnocastlib's per-process tmpfs type announcements.

Each Agnocast process appends `<topic>\\t<type>\\t<role>\\t<node>\\n` lines to
``/run/agnocast/<ipc_ns_inode>/<pid>.txt`` on each ``Publisher<T>`` /
``Subscription<T>`` construction (see
``agnocastlib::internal::TypeRegistryWriter``). The daemon walks the per-NS
directory once per tick and joins the discovered ``(topic, role,
node_name)`` triples with the kmod's endpoint list (which has node_name
but lacks the rosidl type name and the owning pid) so the gossip
publication can carry both pieces of information.

Forward-compatibility rules (parser contract):

* Lines must terminate in ``\\n``. A trailing unterminated tail (writer
  died mid-write) is silently dropped.
* Each line is tab-split. The first four fields are required —
  ``topic``, ``type``, ``role``, ``node_name`` — and any extra fields
  are ignored, so the writer can extend the format additively without
  the daemon needing a lockstep update.
* Lines with fewer than four fields are skipped and a single ``WARN``
  is logged per rebuild (corrupt file or future-incompatible writer).
"""

import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

BASE_DIR = '/run/agnocast'


@dataclass(frozen=True)
class RegistryEntry:
    pid: int
    type_name: str


# Match key (topic_name, role, node_name) -> (pid, type_name).
# role is "pub" or "sub".
RegistryTable = Dict[Tuple[str, str, str], RegistryEntry]


class TypeRegistryReader:
    """Scan ``/run/agnocast/<ns_inode>/`` and build an in-memory lookup table.

    The reader is stateless across ticks; each ``rebuild()`` call replaces
    the table from scratch. This intentionally lets the table forget
    entries whose source process died and whose file was removed by either
    the writer's ``atexit`` cleanup or our own ``cleanup_dead_pids()``.
    """

    def __init__(self, ipc_ns_inode: int, base_dir: str = BASE_DIR, logger=None):
        self._ns_dir = os.path.join(base_dir, str(ipc_ns_inode))
        self._table: RegistryTable = {}
        self._logger = logger
        self._warned_malformed_files: set = set()

    @property
    def ns_dir(self) -> str:
        return self._ns_dir

    @property
    def table(self) -> RegistryTable:
        return self._table

    def rebuild(self) -> None:
        """Re-scan ``ns_dir`` and replace the internal table."""
        new_table: RegistryTable = {}
        try:
            entries = list(os.scandir(self._ns_dir))
        except FileNotFoundError:
            self._table = new_table
            return
        except PermissionError as exc:
            if self._logger is not None:
                self._logger.warn(f'type_registry: cannot read {self._ns_dir}: {exc}')
            self._table = new_table
            return

        for entry in entries:
            if not entry.is_file() or not entry.name.endswith('.txt'):
                continue
            try:
                pid = int(entry.name[:-len('.txt')])
            except ValueError:
                # Unexpected filename; skip silently.
                continue
            self._ingest_file(entry.path, pid, new_table)

        self._table = new_table

    def _ingest_file(self, path: str, pid: int, table: RegistryTable) -> None:
        try:
            with open(path, 'rb') as fp:
                data = fp.read()
        except (FileNotFoundError, PermissionError):
            return

        text = data.decode('utf-8', errors='replace')
        # Only fully terminated lines are accepted. `splitlines(keepends=True)`
        # preserves the trailing `\n` so we can distinguish complete lines
        # from a torn tail.
        for raw_line in text.splitlines(keepends=True):
            if not raw_line.endswith('\n'):
                # Writer was interrupted mid-line; skip the tail.
                continue
            line = raw_line[:-1]
            fields = line.split('\t')
            if len(fields) < 4:
                if path not in self._warned_malformed_files and self._logger is not None:
                    self._logger.warn(
                        f'type_registry: malformed line in {path}: '
                        f'expected at least 4 tab-separated fields, got {len(fields)}')
                    self._warned_malformed_files.add(path)
                continue
            topic, type_name, role, node_name = fields[0], fields[1], fields[2], fields[3]
            # Extra fields beyond the required four are ignored on purpose
            # to keep this reader forward-compatible with future writers.
            if role not in ('pub', 'sub'):
                continue
            table[(topic, role, node_name)] = RegistryEntry(pid=pid, type_name=type_name)

    def cleanup_dead_pids(self) -> None:
        """Remove tmpfs files whose owning PID is no longer alive.

        Called once per tick after ``rebuild()``. The writer process is
        responsible for unlinking its own file on graceful exit; this
        sweep covers SIGKILL / crash cases.
        """
        try:
            entries = list(os.scandir(self._ns_dir))
        except FileNotFoundError:
            return

        for entry in entries:
            if not entry.is_file() or not entry.name.endswith('.txt'):
                continue
            try:
                pid = int(entry.name[:-len('.txt')])
            except ValueError:
                continue
            if os.path.exists(f'/proc/{pid}'):
                continue
            try:
                os.unlink(entry.path)
            except FileNotFoundError:
                pass
            except PermissionError as exc:
                if self._logger is not None:
                    self._logger.warn(
                        f'type_registry: failed to unlink stale {entry.path}: {exc}')

    def lookup(
        self, topic_name: str, role: str, node_name: str
    ) -> Optional[RegistryEntry]:
        """Return ``(pid, type_name)`` for the matching endpoint or ``None``."""
        return self._table.get((topic_name, role, node_name))
