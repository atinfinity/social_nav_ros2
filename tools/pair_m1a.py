#!/usr/bin/env python3
"""oracle-vel の切り分け: crowd の M1a(CSV の M1_moving)のシード別対差と、事前固定の分岐 A-D。

R = oracle-vel - social(主指標)、T = use_tracker - social、V = use_tracker - oracle-vel。
SE = 標本標準偏差 / sqrt(n)、比 = 平均 / SE、区間 = 平均 +- 2*SE。
social を渡さなければ V だけ出す。標準ライブラリのみ。
"""
import argparse
import csv
import math
import sys

N_EXPECTED = 200
COL = "M1_moving"


def load(path):
    rows = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r["scenario"] != "crowd":
                continue
            seed = int(r["seed"])
            if seed in rows:
                sys.exit(f"error: {path}: duplicate seed {seed}")
            rows[seed] = int(r[COL])
    if len(rows) != N_EXPECTED:
        sys.exit(f"error: {path}: crowd rows = {len(rows)}, expected {N_EXPECTED}")
    return rows


def stats(a, b):
    """a - b のシード別対差。"""
    d = [a[s] - b[s] for s in sorted(a)]
    n = len(d)
    mean = sum(d) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in d) / (n - 1))
    se = sd / math.sqrt(n)
    ratio = mean / se if se > 0 else float("nan")
    return mean, se, ratio, mean - 2 * se, mean + 2 * se


def branch(mean, se, lo, hi, delta):
    if mean > 2 * se:
        return "A", "残りあり"
    if mean < -2 * se:
        return "D", "逆向き(social の方が接触が多い)"
    if lo >= -delta and hi <= delta:
        return "B", f"同等(残りは {delta:.2f} 件/シード未満)"
    return "C", f"判定不能(残りは検出できず、同等も示せず。残りの上限 {hi:+.3f})"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("use")
    p.add_argument("oracle")
    p.add_argument("social", nargs="?")
    p.add_argument("--delta", type=float, default=0.70)
    a = p.parse_args()

    use, ora = load(a.use), load(a.oracle)
    soc = load(a.social) if a.social else None
    for name, d in (("oracle", ora), ("social", soc)):
        if d is not None and set(d) != set(use):
            sys.exit(f"error: seed set of {name} differs from use")

    seeds = sorted(use)
    print(f"seeds {seeds[0]}-{seeds[-1]}, n={len(seeds)}")
    print(f"sum M1a: use={sum(use.values())} oracle={sum(ora.values())}"
          + (f" social={sum(soc.values())}" if soc else ""))
    pairs = [("V = use - oracle", use, ora)]
    if soc:
        pairs = [("R = oracle - social", ora, soc),
                 ("T = use - social", use, soc)] + pairs
    res = {}
    for name, x, y in pairs:
        m, se, r, lo, hi = stats(x, y)
        res[name[0]] = (m, se, lo, hi)
        print(f"{name}: mean {m:+.3f} SE {se:.3f} ratio {r:.2f} "
              f"interval [{lo:+.3f}, {hi:+.3f}]")
    if soc:
        m, se, lo, hi = res["R"]
        b, text = branch(m, se, lo, hi, a.delta)
        print(f"branch (R only, delta={a.delta:.2f}): {b} {text}")


if __name__ == "__main__":
    main()
