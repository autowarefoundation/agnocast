#!/usr/bin/env python3
"""Print where the metadata IPC round trip spends its time, per backend.

  scripts/bench/breakdown.py results/kmod results/daemon

Pools every per-message sample under <dir>/iter_*/ and reports each segment of
the round trip. The segments are non-overlapping and sum to the total, so the
"share" column (computed from means, which are additive, unlike percentiles)
says directly which part dominates.

send_syscall is nested inside up rather than being a segment of its own; it is
shown for reference and excluded from the share column.

Pure standard library on purpose: this runs on a measurement machine without
having to install the plotting requirements.
"""

import argparse
import csv
import sys
from pathlib import Path

# (segment, column, explanation)
SEGMENTS = [
    ("req", "ipc_req_ns", "client: marshal request, zero response struct"),
    ("up", "ipc_up_ns", "sendmsg -> daemon's recv returns (transport + wakeup)"),
    ("lock", "ipc_lock_ns", "daemon: dispatch, topic lookup, lock acquisition"),
    ("work", "ipc_work_ns", "daemon: handler body and response fill"),
    ("down", "ipc_down_ns", "daemon's reply -> client's recvmsg returns"),
    ("post", "ipc_post_ns", "client: unmarshal response into caller args"),
]

NESTED = [("send_syscall", "ipc_send_syscall_ns", "sendmsg() itself, nested inside up")]

METRICS = [
    ("publish_ipc", "publisher_*_latencies.csv", "publish_ipc_ns"),
    ("receive_ipc", "subscriber_*_topic_*_latencies.csv", "receive_ipc_ns"),
]


def percentile(sorted_values, pct):
    if not sorted_values:
        return 0.0
    idx = int(pct / 100.0 * len(sorted_values) + 0.5) - 1
    idx = min(max(idx, 0), len(sorted_values) - 1)
    return sorted_values[idx]


def pool(root: Path, pattern: str):
    """Merge the named integer columns of every matching CSV under root/iter_*."""
    columns = {}
    iter_dirs = sorted(root.glob("iter_*")) or [root]
    for iter_dir in iter_dirs:
        for path in sorted(iter_dir.glob(pattern)):
            with open(path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    for name, cell in row.items():
                        if name is None:
                            continue
                        try:
                            columns.setdefault(name, []).append(int(cell))
                        except (TypeError, ValueError):
                            pass
    return columns


def us(ns):
    return ns / 1000.0


def report(label: str, root: Path):
    for metric, pattern, total_column in METRICS:
        columns = pool(root, pattern)
        totals = columns.get(total_column)
        if not totals:
            print(f"\n=== {label}: {metric} — no samples under {root} ===")
            continue

        ordered_total = sorted(totals)
        n = len(ordered_total)
        mean_total = sum(totals) / n

        stamped = columns.get("ipc_daemon_stamped", [])
        has_breakdown = any(columns.get(col) and any(columns[col]) for _, col, _ in SEGMENTS)

        print(f"\n=== {label}: {metric} (n={n}) ===")
        if not has_breakdown:
            print(
                "  no breakdown columns with data: this backend services metadata in the\n"
                "  caller's own context, so there is nothing to attribute from userspace.\n"
                f"  round trip  mean {us(mean_total):8.2f}  p50 {us(percentile(ordered_total, 50)):8.2f}"
                f"  p99 {us(percentile(ordered_total, 99)):8.2f}"
                f"  p99.9 {us(percentile(ordered_total, 99.9)):8.2f}  (us)"
            )
            continue

        if stamped and not all(stamped):
            missing = len(stamped) - sum(1 for v in stamped if v)
            print(f"  note: {missing}/{len(stamped)} samples carried no daemon stamps")

        header = f"  {'segment':<14}{'mean':>9}{'p50':>9}{'p90':>9}{'p99':>9}{'p99.9':>10}{'share':>8}"
        print(header)
        print("  " + "-" * (len(header) - 2))

        segment_mean_sum = 0.0
        for name, column, _ in SEGMENTS:
            values = columns.get(column)
            if not values:
                continue
            ordered = sorted(values)
            mean = sum(values) / len(values)
            segment_mean_sum += mean
            share = (mean / mean_total * 100.0) if mean_total else 0.0
            print(
                f"  {name:<14}{us(mean):9.2f}{us(percentile(ordered, 50)):9.2f}"
                f"{us(percentile(ordered, 90)):9.2f}{us(percentile(ordered, 99)):9.2f}"
                f"{us(percentile(ordered, 99.9)):10.2f}{share:7.1f}%"
            )

        print("  " + "-" * (len(header) - 2))
        print(
            f"  {'round trip':<14}{us(mean_total):9.2f}{us(percentile(ordered_total, 50)):9.2f}"
            f"{us(percentile(ordered_total, 90)):9.2f}{us(percentile(ordered_total, 99)):9.2f}"
            f"{us(percentile(ordered_total, 99.9)):10.2f}{100.0:7.1f}%"
        )
        drift = segment_mean_sum - mean_total
        print(f"  {'(segment sum)':<14}{us(segment_mean_sum):9.2f}   drift {us(drift):+.3f} us")

        for name, column, _ in NESTED:
            values = columns.get(column)
            if not values or not any(values):
                continue
            ordered = sorted(values)
            mean = sum(values) / len(values)
            print(
                f"  {name:<14}{us(mean):9.2f}{us(percentile(ordered, 50)):9.2f}"
                f"{us(percentile(ordered, 90)):9.2f}{us(percentile(ordered, 99)):9.2f}"
                f"{us(percentile(ordered, 99.9)):10.2f}{'nested':>8}"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "dirs", nargs="+", type=Path, help="result directories, each containing iter_*/"
    )
    parser.add_argument("--legend", action="store_true", help="print what each segment covers")
    args = parser.parse_args()

    for root in args.dirs:
        if not root.is_dir():
            print(f"skipping {root}: not a directory", file=sys.stderr)
            continue
        report(root.name, root)

    if args.legend:
        print("\nSegments (in the order they occur):")
        for name, _, description in SEGMENTS + NESTED:
            print(f"  {name:<14}{description}")

    print("")


if __name__ == "__main__":
    main()
