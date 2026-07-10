#!/usr/bin/env python3
"""Aggregate agnocast_bench results into a MQ-vs-eventfd comparison table.

Usage: python3 analyze.py <results_dir>

<results_dir> layout (produced by run_eventfd_bench.sh):
  <results_dir>/{mq_base,eventfd}/N<k>/pub.csv        # publish_latency_ns
  <results_dir>/{mq_base,eventfd}/N<k>/sub_*.csv       # e2e_latency_ns (merged)
"""
import glob
import os
import sys


def _pct(values, p):
    if not values:
        return None
    s = sorted(values)
    idx = min(len(s) - 1, int(round(p / 100.0 * len(s))))
    return s[idx]


def _read(path):
    out = []
    try:
        with open(path) as f:
            next(f, None)  # header
            for line in f:
                line = line.strip()
                if line:
                    out.append(int(line))
    except FileNotFoundError:
        pass
    return out


def _collect(tag_dir):
    """Return {N: {'publish': [...], 'e2e': [...]}} for one branch dir."""
    res = {}
    for nd in sorted(glob.glob(os.path.join(tag_dir, "N*"))):
        n = int(os.path.basename(nd)[1:])
        publish = _read(os.path.join(nd, "pub.csv"))
        e2e = []
        for sub in glob.glob(os.path.join(nd, "sub_*.csv")):
            e2e += _read(sub)
        res[n] = {"publish": publish, "e2e": e2e}
    return res


def _us(ns):
    return "-" if ns is None else f"{ns / 1000.0:.1f}"


def _delta(base, ev):
    if not base or not ev or base[0] is None or ev[0] is None:
        return "-"
    return f"{(ev[1] - base[1]) / base[1] * 100.0:+.0f}%"  # base/ev are (label, p50)


def main():
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <results_dir>")
    root = sys.argv[1]
    mq = _collect(os.path.join(root, "mq_base"))
    ev = _collect(os.path.join(root, "eventfd"))
    ns = sorted(set(mq) & set(ev))

    for metric in ("publish", "e2e"):
        title = "publish latency (publisher wall-clock)" if metric == "publish" \
            else "e2e latency (publish -> callback)"
        print(f"\n=== {title} — us ===")
        print(f"{'N':>4} | {'MQ p50':>8} {'MQ p99':>8} | {'evfd p50':>8} {'evfd p99':>8} | "
              f"{'p50 Δ':>6} {'p99 Δ':>6}")
        print("-" * 66)
        for n in ns:
            mv, ev_v = mq[n][metric], ev[n][metric]
            m50, m99 = _pct(mv, 50), _pct(mv, 99)
            e50, e99 = _pct(ev_v, 50), _pct(ev_v, 99)
            d50 = _delta(("mq", m50), ("ev", e50)) if (m50 and e50) else "-"
            d99 = _delta(("mq", m99), ("ev", e99)) if (m99 and e99) else "-"
            print(f"{n:>4} | {_us(m50):>8} {_us(m99):>8} | {_us(e50):>8} {_us(e99):>8} | "
                  f"{d50:>6} {d99:>6}   (n_mq={len(mv)}, n_ev={len(ev_v)})")
    print("\nΔ = eventfd vs MQ; negative = eventfd faster.")


if __name__ == "__main__":
    main()
