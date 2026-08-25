#!/usr/bin/env python3
"""Aggregate sweep results and render comparison figures.

  python3 plot_results.py \
    --data results/kmod   --label "kernel module" --color "#1f77b4" \
    --data results/daemon --label "user daemon"   --color "#d62728" \
    --output-dir figures/

Percentiles are pooled: every per-message sample from every iteration, topic,
publisher and subscriber of a configuration goes into one array, and the
percentile is taken from that array. Averaging per-process percentiles instead
would understate the tail, because a rare stall in one process gets diluted by
the well-behaved ones.
"""

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.colors as mcolors  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

matplotlib.rcParams.update(
    {
        "font.family": "serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 9,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "figure.figsize": (3.5, 2.5),
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "lines.linewidth": 1.0,
        "lines.markersize": 4,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linewidth": 0.5,
    }
)

DEFAULT_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]

# metric -> (csv glob, column holding nanosecond samples)
METRIC_COLUMNS = {
    "publish": ("publisher_*_latencies.csv", "publish_latency_ns"),
    "publish_ipc": ("publisher_*_latencies.csv", "publish_ipc_ns"),
    "e2e": ("subscriber_*_topic_*_latencies.csv", "e2e_latency_ns"),
    "receive_ipc": ("subscriber_*_topic_*_latencies.csv", "receive_ipc_ns"),
}

METRIC_TITLES = {
    "publish": "Publish path",
    "publish_ipc": "Publish IPC round trip",
    "e2e": "End-to-end",
    "receive_ipc": "Receive IPC round trip",
}

SWEEPS = {
    "a": ("sweep_a", r"num_topics_(\d+)", "Number of topics", "topics"),
    "b": ("sweep_b", r"num_subscribers_(\d+)", "Number of subscribers", "subscribers"),
    "c": ("sweep_c", r"rate_hz_(\d+)", "Publish rate (Hz)", "rate"),
}


def pool_metric_us(config_dir: Path, metric: str) -> np.ndarray:
    """Concatenate every per-message sample of one metric under a config dir."""
    pattern, column = METRIC_COLUMNS[metric]
    chunks = []
    for iter_dir in sorted(config_dir.glob("iter_*")):
        for path in iter_dir.glob(pattern):
            try:
                df = pd.read_csv(path, usecols=[column])
            except (ValueError, KeyError):
                continue
            if not df.empty:
                chunks.append(df[column].to_numpy())
    if not chunks:
        return np.empty(0, dtype=np.float64)
    return np.concatenate(chunks) / 1000.0


def summarize(values_us: np.ndarray) -> dict:
    return {
        "count": int(values_us.size),
        "p50": float(np.percentile(values_us, 50)),
        "p99": float(np.percentile(values_us, 99)),
        "p999": float(np.percentile(values_us, 99.9)),
        "p9999": float(np.percentile(values_us, 99.99)),
        "max": float(values_us.max()),
    }


def pooled_stats(sweep_dir: Path, param_regex: str) -> pd.DataFrame:
    rows = []
    if not sweep_dir.is_dir():
        return pd.DataFrame()
    for config_dir in sorted(p for p in sweep_dir.iterdir() if p.is_dir()):
        match = re.search(param_regex, config_dir.name)
        if not match:
            continue
        for metric in METRIC_COLUMNS:
            values = pool_metric_us(config_dir, metric)
            if values.size == 0:
                continue
            rows.append(
                {"sweep_value": float(match.group(1)), "metric": metric, **summarize(values)}
            )
    return pd.DataFrame(rows)


def pooled_stats_2d(sweep_dir: Path) -> pd.DataFrame:
    rows = []
    if not sweep_dir.is_dir():
        return pd.DataFrame()
    for config_dir in sorted(p for p in sweep_dir.iterdir() if p.is_dir()):
        match = re.match(r"nt_(\d+)_ns_(\d+)$", config_dir.name)
        if not match:
            continue
        for metric in METRIC_COLUMNS:
            values = pool_metric_us(config_dir, metric)
            if values.size == 0:
                continue
            rows.append(
                {
                    "num_topics": int(match.group(1)),
                    "num_subscribers": int(match.group(2)),
                    "metric": metric,
                    **summarize(values),
                }
            )
    return pd.DataFrame(rows)


def save_figure(fig, name, output_dir):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{name}.pdf")
    fig.savefig(out / f"{name}.png")
    plt.close(fig)
    print(f"  saved {name}.pdf / .png")


def plot_vs_sweep(datasets, metric, xlabel, filename, output_dir):
    fig, ax = plt.subplots()
    plotted = False

    for label, color, stats in datasets:
        data = stats[stats["metric"] == metric].sort_values("sweep_value")
        if data.empty:
            continue
        # to_numpy(): older matplotlib cannot index a pandas Series directly.
        prefix = f"{label} " if len(datasets) > 1 else ""
        x = data["sweep_value"].to_numpy()
        ax.plot(x, data["p50"].to_numpy(), label=f"{prefix}p50", color=color, marker="o")
        ax.plot(
            x,
            data["p999"].to_numpy(),
            label=f"{prefix}p99.9",
            color=color,
            linestyle="--",
            marker="v",
            alpha=0.85,
        )
        plotted = True

    if not plotted:
        plt.close(fig)
        return

    ax.set_xlabel(xlabel)
    ax.set_ylabel("Latency (us)")
    ax.set_title(METRIC_TITLES[metric])
    ax.legend(loc="best", ncol=2, fontsize=6)
    save_figure(fig, filename, output_dir)


def plot_cdf(datasets, output_dir, suffix=""):
    for metric in METRIC_COLUMNS:
        fig, ax = plt.subplots()
        plotted = False

        for label, color, samples in datasets:
            values = samples.get(metric)
            if values is None or values.size == 0:
                continue
            ordered = np.sort(values)
            cdf = np.arange(1, ordered.size + 1) / ordered.size
            ax.plot(ordered, cdf, label=label, color=color, linewidth=0.8)
            plotted = True

        if not plotted:
            plt.close(fig)
            continue

        ax.set_xlabel(f"{METRIC_TITLES[metric]} latency (us)")
        ax.set_ylabel("CDF")
        ax.set_xscale("log")
        ax.legend(loc="lower right", fontsize=6)
        save_figure(fig, f"cdf_{metric}{suffix}", output_dir)


def plot_heatmaps(datasets_2d, metric, output_dir):
    pivots = []
    for label, stats in datasets_2d:
        if stats.empty:
            continue
        subset = stats[stats["metric"] == metric]
        if subset.empty:
            continue
        pivots.append(
            (
                label,
                subset.pivot(
                    index="num_topics", columns="num_subscribers", values="p999"
                ).sort_index(),
            )
        )

    if not pivots:
        print(f"  no 2D data for {metric}, skipping heatmap")
        return

    vmin = min(h.min().min() for _, h in pivots)
    vmax = max(h.max().max() for _, h in pivots)
    norm = mcolors.LogNorm(vmin=max(vmin, 0.01), vmax=vmax)

    fig, axes = plt.subplots(1, len(pivots), figsize=(2.6 * len(pivots), 2.8))
    if len(pivots) == 1:
        axes = [axes]

    image = None
    for ax, (label, heatmap) in zip(axes, pivots):
        ax.grid(False)
        image = ax.imshow(heatmap.values, aspect="auto", cmap="YlOrRd", origin="lower", norm=norm)
        ax.set_xticks(range(len(heatmap.columns)))
        ax.set_xticklabels(heatmap.columns.astype(int))
        ax.set_yticks(range(len(heatmap.index)))
        ax.set_yticklabels(heatmap.index.astype(int))
        ax.set_title(label, fontsize=8)

    axes[0].set_ylabel("Number of topics")
    for ax in axes[1:]:
        ax.set_yticklabels([])

    fig.colorbar(image, ax=axes, shrink=0.8, label="p99.9 latency (us)")
    fig.text(0.42, 0.01, "Number of subscribers", ha="center", fontsize=8)
    save_figure(fig, f"heatmap_{metric}_p999", output_dir)


def write_stats_table(all_stats, output_dir):
    """Dump every pooled percentile to CSV so the numbers are inspectable."""
    if not all_stats:
        return
    combined = pd.concat(all_stats, ignore_index=True)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / "pooled_stats.csv"
    combined.to_csv(path, index=False)
    print(f"  saved {path.name} ({len(combined)} rows)")

    # A pivot on the IPC metrics is the most direct read of the backend cost.
    for metric in ("publish_ipc", "receive_ipc", "e2e"):
        subset = combined[combined["metric"] == metric]
        if subset.empty or subset["label"].nunique() < 2:
            continue
        print(f"\n  {METRIC_TITLES[metric]} (us), pooled:")
        pivot = subset.pivot_table(
            index=["sweep", "sweep_value"], columns="label", values=["p50", "p999"]
        )
        print(pivot.to_string(float_format=lambda v: f"{v:8.3f}"))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--data", action="append", required=True, help="results root per backend")
    parser.add_argument("--label", action="append", help="label per --data")
    parser.add_argument("--color", action="append", help="color per --data")
    parser.add_argument("--output-dir", default="figures")
    parser.add_argument("--sweep", choices=["a", "b", "c", "d", "cdf", "all"], default="all")
    return parser.parse_args()


def main():
    args = parse_args()

    count = len(args.data)
    labels = list(args.label or [])
    labels += [f"dataset {i}" for i in range(len(labels), count)]
    colors = list(args.color or [])
    colors += DEFAULT_COLORS[len(colors) : count]

    output_dir = Path(args.output_dir)
    collected = []

    for key, (dirname, regex, xlabel, short) in SWEEPS.items():
        if args.sweep not in (key, "all"):
            continue
        print(f"Sweep {key.upper()}: {xlabel}")

        datasets = []
        for data_dir, label, color in zip(args.data, labels, colors):
            sweep_dir = Path(data_dir) / dirname
            if not sweep_dir.is_dir():
                sweep_dir = Path(data_dir)
            stats = pooled_stats(sweep_dir, regex)
            if stats.empty:
                print(f"  no data under {sweep_dir}")
                continue
            datasets.append((label, color, stats))
            collected.append(stats.assign(label=label, sweep=key))

        for metric in METRIC_COLUMNS:
            plot_vs_sweep(datasets, metric, xlabel, f"{metric}_vs_{short}", output_dir)

    if args.sweep in ("d", "all"):
        print("Sweep D: topics x subscribers")
        datasets_2d = []
        for data_dir, label in zip(args.data, labels):
            sweep_dir = Path(data_dir) / "sweep_d"
            if not sweep_dir.is_dir():
                sweep_dir = Path(data_dir)
            datasets_2d.append((label, pooled_stats_2d(sweep_dir)))
        for metric in ("e2e", "publish_ipc", "receive_ipc"):
            plot_heatmaps(datasets_2d, metric, output_dir)

    if args.sweep in ("cdf", "all"):
        print("Latency CDFs")
        # For "all", the CDF pools every configuration of every sweep, which is
        # a distribution over the whole experiment rather than one operating
        # point. Point --data at a single configuration directory for a CDF of
        # that configuration alone.
        datasets = []
        for data_dir, label, color in zip(args.data, labels, colors):
            root = Path(data_dir)
            config_dirs = (
                [root]
                if list(root.glob("iter_*"))
                else [p.parent for p in root.glob("*/*/iter_0")]
            )
            samples = {}
            for metric in METRIC_COLUMNS:
                chunks = [pool_metric_us(c, metric) for c in config_dirs]
                chunks = [c for c in chunks if c.size]
                samples[metric] = np.concatenate(chunks) if chunks else np.empty(0)
            if any(v.size for v in samples.values()):
                datasets.append((label, color, samples))
        plot_cdf(datasets, output_dir)

    write_stats_table(collected, output_dir)
    print("\nDone.")


if __name__ == "__main__":
    main()
