#!/usr/bin/env python3
"""Plots the shared heap usage of processes started with AGNOCAST_HEAPHOOK_MEMORY_STATS set.

Shows the usage over time of the top N processes and, unless --no-total, the total over all processes.
Each process has one color: the solid line is the current usage, the dashed line the peak (max) recorded
by the heaphook, and the dotted line the high water mark (hwm): the peak span from the lowest to the
highest allocated address, i.e. allocated memory plus external fragmentation. The totals are drawn the same way in black; the total max
is the sum of the per-process peaks, which need not have happened at the same time.

Plots a file recorded with agnocast_heaphook_memory_stats.sh and ranks processes by their peak usage
(the recorded max) over the whole log. Log timestamps are ISO 8601 with UTC offset and are shown in
local time.

Usage:
  agnocast_heaphook_memory_plot.py LOG [--top N] [--no-total] [--output FILE]
"""

import argparse
import collections
import datetime
import os
import re

import matplotlib.pyplot as plt

# A line printed by agnocast_heaphook_memory_stats.sh. The timestamp is ISO 8601 with UTC offset.
LOG_LINE = re.compile(
    r"^(?P<time>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?[+-]\d{2}:\d{2})"
    r"\s+pid=(?P<pid>\d+)\s+current=(?P<current>\d+)\s+max=(?P<max>\d+)\s+hwm=(?P<hwm>\d+)\s+name=(?P<name>\S+)"
)
MIB = 1024 * 1024


def label(pid, name):
    return f"{pid} {name}" if name else str(pid)


def describe(values, hwms, max_mib=None):
    """max_mib: the peak recorded by the heaphook; sampling alone misses spikes between samples."""
    return f"{values[-1]:.2f} MiB, peak {max(values) if max_mib is None else max_mib:.2f}, hwm {max(hwms):.2f}"


def draw(ax, series, names, totals, title):
    """series: {pid: (ts, current_mib, max_mib, hwm_mib)} already restricted to the pids to plot, in legend order.

    totals: (times, current_mib, max_mib, hwm_mib) summed over all processes, or None to not plot them.
    """
    ax.clear()
    plotted = []
    for pid, (ts, values, maxes, hwms) in series.items():
        (line,) = ax.plot(ts, values, label=f"{label(pid, names.get(pid))} ({describe(values, hwms, max(maxes))})")
        ax.plot(ts, maxes, "--", color=line.get_color(), linewidth=1)
        ax.plot(ts, hwms, ":", color=line.get_color(), linewidth=1.5)
        plotted += [values, maxes, hwms]
    if totals and totals[0]:
        times, values, maxes, hwms = totals
        # The total's own peak is only known at the samples; the sum of the per-process peaks bounds it.
        ax.plot(
            times, values, "k-", linewidth=2,
            label=f"total ({describe(values, hwms)}, sum of peaks {max(maxes):.2f})",
        )
        ax.plot(times, maxes, "k--", linewidth=1.5)
        ax.plot(times, hwms, "k:", linewidth=2)
        plotted += [values, maxes, hwms]
    # Style key; the colored entries above identify the processes.
    for style, name in (("-", "current"), ("--", "max"), (":", "hwm")):
        ax.plot([], [], style, color="gray", label=name)
    ax.set_ylabel("shared heap [MiB]")
    ax.set_title(title)
    # Headroom above the highest line so the legend does not cover it.
    top = max([max(values) for values in plotted if values] + [0.1])
    ax.set_ylim(0, top * 1.4)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize="small", ncol=2)


def plot_log(args):
    """Plots a log written by agnocast_heaphook_memory_stats.sh."""
    history = collections.defaultdict(lambda: ([], [], [], []))
    total_by_time = collections.OrderedDict()
    max_by_time = {}
    hwm_by_time = {}
    names = {}
    with open(args.log) as f:
        for line in f:
            m = LOG_LINE.match(line)
            if not m:
                continue
            pid, current, max_ = int(m["pid"]), int(m["current"]), int(m["max"])
            hwm = int(m["hwm"])
            names[pid] = m["name"]
            # Naive local time, which is what matplotlib shows without further configuration.
            t = datetime.datetime.fromisoformat(m["time"]).astimezone().replace(tzinfo=None)
            ts, values, maxes, hwms = history[pid]
            ts.append(t)
            values.append(current / MIB)
            maxes.append(max_ / MIB)
            hwms.append(hwm / MIB)
            total_by_time[t] = total_by_time.get(t, 0.0) + current / MIB
            max_by_time[t] = max_by_time.get(t, 0.0) + max_ / MIB
            hwm_by_time[t] = hwm_by_time.get(t, 0.0) + hwm / MIB

    if not history:
        raise SystemExit(f"no samples found in {args.log}")

    peaks = {pid: max(maxes) for pid, (_, _, maxes, _) in history.items()}
    top = sorted(peaks, key=peaks.get, reverse=True)[: args.top]

    fig, ax = plt.subplots(figsize=(11, 6))
    times = list(total_by_time)
    totals = (
        times, list(total_by_time.values()), [max_by_time[t] for t in times], [hwm_by_time[t] for t in times]
    ) if args.total else None
    draw(
        ax, {pid: history[pid] for pid in top}, names, totals,
        f"{os.path.basename(args.log)}: top {args.top} of {len(history)} processes by peak"
        + (", and total" if args.total else ""),
    )
    dates = plt.matplotlib.dates
    # Shows times for short logs and dates once the log spans days.
    locator = dates.AutoDateLocator()
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(dates.ConciseDateFormatter(locator))
    ax.set_xlabel("local time")

    if args.output:
        fig.savefig(args.output, dpi=120, bbox_inches="tight")
        print(f"saved {args.output}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="log written by agnocast_heaphook_memory_stats.sh")
    parser.add_argument("--top", type=int, default=5, help="number of processes to plot (default: 5)")
    parser.add_argument(
        "--total", action=argparse.BooleanOptionalAction, default=True,
        help="plot the total over all processes (default: on)",
    )
    parser.add_argument("--output", help="save the plot to this file instead of showing it")
    args = parser.parse_args()

    plot_log(args)


if __name__ == "__main__":
    main()
