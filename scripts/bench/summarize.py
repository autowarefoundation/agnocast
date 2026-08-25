#!/usr/bin/env python3
"""Collapse one iteration's per-process CSVs into a single summary.csv.

Percentiles here are per-process; the pooled percentiles used for plotting are
recomputed from the raw CSVs by plot_results.py.

Besides the four headline metrics (publish, publish_ipc, e2e, receive_ipc) this
also emits one row per segment of the IPC round trip when the raw CSVs carry the
breakdown columns, named "<metric>_<segment>" (e.g. receive_ipc_up). Segments
whose column is absent or all zero are skipped, so a kernel-module run keeps the
four headline rows only.
"""

import argparse
import csv
import re
from pathlib import Path

PUB_RE = re.compile(r"^publisher_(\d+)_latencies\.csv$")
SUB_RE = re.compile(r"^subscriber_(\d+)_topic_(\d+)_latencies\.csv$")

HEADER = [
    "backend",
    "iteration",
    "metric",
    "topic_idx",
    "sub_idx",
    "count",
    "p50_us",
    "p90_us",
    "p99_us",
    "p999_us",
    "throughput_ops_sec",
    "delivery_rate",
]

# Segment name -> raw CSV column, in the order they occur during a round trip.
# send_syscall is nested inside up, so it is reported but never summed.
IPC_SEGMENTS = [
    ("total", "ipc_total_ns"),
    ("req", "ipc_req_ns"),
    ("send_syscall", "ipc_send_syscall_ns"),
    ("up", "ipc_up_ns"),
    ("lock", "ipc_lock_ns"),
    ("work", "ipc_work_ns"),
    ("down", "ipc_down_ns"),
    ("post", "ipc_post_ns"),
]


def percentile_ns(sorted_values, pct):
    """Nearest-rank percentile over a pre-sorted list."""
    if not sorted_values:
        return 0.0
    idx = int(pct / 100.0 * len(sorted_values) + 0.5) - 1
    idx = min(max(idx, 0), len(sorted_values) - 1)
    return sorted_values[idx]


def read_table(path):
    """Return {column name: [int, ...]} for every integer column present."""
    columns = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames or []:
            columns[name] = []
        for row in reader:
            for name, cell in row.items():
                if name not in columns:
                    continue
                try:
                    columns[name].append(int(cell))
                except (TypeError, ValueError):
                    pass
    return columns


def read_meta(path):
    if not path.exists():
        return None
    with open(path, newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        row = next(reader, None)
    if not row or len(row) < 3:
        return None
    try:
        return int(row[1]), int(row[2])
    except ValueError:
        return None


def make_row(backend, iteration, metric, topic_idx, sub_idx, values, duration, delivery_rate=""):
    values = sorted(values)
    count = len(values)
    return [
        backend,
        iteration,
        metric,
        topic_idx,
        sub_idx,
        count,
        f"{percentile_ns(values, 50) / 1000.0:.3f}",
        f"{percentile_ns(values, 90) / 1000.0:.3f}",
        f"{percentile_ns(values, 99) / 1000.0:.3f}",
        f"{percentile_ns(values, 99.9) / 1000.0:.3f}",
        f"{count / duration:.1f}" if duration > 0 else "",
        delivery_rate,
    ]


def segment_rows(backend, iteration, metric, topic_idx, sub_idx, columns, duration):
    """One row per IPC segment that the raw CSV actually carries."""
    rows = []
    for segment, column in IPC_SEGMENTS:
        values = columns.get(column)
        if not values or not any(values):
            continue
        rows.append(
            make_row(
                backend,
                iteration,
                f"{metric}_{segment}",
                topic_idx,
                sub_idx,
                values,
                duration,
            )
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iter-dir", required=True, type=Path)
    parser.add_argument("--iteration", required=True, type=int)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--backend", required=True)
    args = parser.parse_args()

    rows = []

    for path in sorted(args.iter_dir.glob("publisher_*_latencies.csv")):
        match = PUB_RE.match(path.name)
        if not match:
            continue
        topic_idx = int(match.group(1))
        columns = read_table(path)
        latency = columns.get("publish_latency_ns", [])
        ipc = columns.get("publish_ipc_ns", [])
        if not latency:
            continue
        rows.append(
            make_row(args.backend, args.iteration, "publish", topic_idx, -1, latency, args.duration)
        )
        rows.append(
            make_row(
                args.backend, args.iteration, "publish_ipc", topic_idx, -1, ipc, args.duration
            )
        )
        rows.extend(
            segment_rows(
                args.backend, args.iteration, "publish_ipc", topic_idx, -1, columns, args.duration
            )
        )

    for path in sorted(args.iter_dir.glob("subscriber_*_topic_*_latencies.csv")):
        match = SUB_RE.match(path.name)
        if not match:
            continue
        sub_idx, topic_idx = int(match.group(1)), int(match.group(2))
        columns = read_table(path)
        e2e = columns.get("e2e_latency_ns", [])
        ipc = columns.get("receive_ipc_ns", [])
        if not e2e:
            continue

        delivery_rate = ""
        pub_meta = read_meta(args.iter_dir / f"publisher_{topic_idx}_meta.csv")
        sub_meta = read_meta(args.iter_dir / f"subscriber_{sub_idx}_topic_{topic_idx}_meta.csv")
        if pub_meta and sub_meta and pub_meta[1] > 0:
            delivery_rate = f"{sub_meta[1] / pub_meta[1]:.4f}"

        rows.append(
            make_row(
                args.backend,
                args.iteration,
                "e2e",
                topic_idx,
                sub_idx,
                e2e,
                args.duration,
                delivery_rate,
            )
        )
        rows.append(
            make_row(
                args.backend, args.iteration, "receive_ipc", topic_idx, sub_idx, ipc, args.duration
            )
        )
        rows.extend(
            segment_rows(
                args.backend,
                args.iteration,
                "receive_ipc",
                topic_idx,
                sub_idx,
                columns,
                args.duration,
            )
        )

    with open(args.iter_dir / "summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)


if __name__ == "__main__":
    main()
