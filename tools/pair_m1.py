#!/usr/bin/env python3
"""安全停止の整合: corridor_oncoming の M1 合計(CSV の M1 = M1a + M1b)のシード別対差と、事前固定の分岐 A/B/C。
規則の全文は docs/sim_results.md の「安全停止の整合: 規則の固定」(測定の前に固定したもの)。標準ライブラリのみ。

使い方:
  pair_m1.py --base base.csv --d1 d1.csv --d2 d2.csv [--legacy legacy.csv] [--scenario corridor_oncoming] [--seeds 521-620]
  pair_m1.py --self-test
入力は social_nav_sim の --csv(または run_sim_eval.sh の social_<範囲>.csv)。対差 = 介入 - 既定(base)。
SE = 標本標準偏差 / sqrt(n)。分岐: A = D1 の平均 <= -2*SE、A でなく D2 の平均 <= -2*SE なら B、どちらも届かなければ C。
legacy は併記だけ(分岐には使わない)。終了コード: 検査エラー 2、それ以外 0。
"""
import argparse
import csv
import math
import sys


def fail(msg):
    sys.exit(f"error: {msg}")


def load(path, scenario, seeds):
    """{seed: (M1, M1a, M1b)}。シード集合は seeds に一致しなければエラー。"""
    rows = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r["scenario"] != scenario:
                continue
            s = int(r["seed"])
            if s in rows:
                fail(f"{path}: duplicate seed {s}")
            rows[s] = (int(r["M1"]), int(r["M1_moving"]), int(r["M1_stopped"]))
    if set(rows) != set(seeds):
        fail(f"{path}: seed set differs (got {len(rows)} rows, expected {len(seeds)})")
    return rows


def stats(x, y, k=0):
    """x - y のシード別対差の (平均, SE)。k は列(0 = M1 合計、1 = M1a、2 = M1b)。"""
    d = [x[s][k] - y[s][k] for s in sorted(x)]
    n = len(d)
    mean = sum(d) / n
    if n < 2:
        fail("n < 2")
    sd = math.sqrt(sum((v - mean) ** 2 for v in d) / (n - 1))
    return mean, sd / math.sqrt(n)


def branch(d1, d2):
    """d1, d2 = (平均, SE)。"""
    # 全シードで対差 0(SE = 0)のときは「届いた」に数えない(平均 < 0 を要する)
    if d1[0] < 0 and d1[0] <= -2 * d1[1]:
        return "A", "安全停止が起点"
    if d2[0] < 0 and d2[0] <= -2 * d2[1]:
        return "B", "停止後の人側の反応が原因"
    return "C", "sfm2 の性質として記録し、対処しない"


def fmt(name, st):
    m, se = st
    ratio = m / se if se > 0 else float("nan")
    return f"{name}: mean {m:+.3f} SE {se:.3f} ratio {ratio:.2f}"


def report(base, d1, d2, legacy):
    seeds = sorted(base)
    print(f"seeds {seeds[0]}-{seeds[-1]}, n={len(seeds)}")
    def sums(r):
        return f"M1 {sum(v[0] for v in r.values())} (M1a {sum(v[1] for v in r.values())} + M1b {sum(v[2] for v in r.values())})"
    print(f"base: {sums(base)}")
    out = {}
    for name, r in (("D1", d1), ("D2", d2), ("legacy", legacy)):
        if r is None:
            continue
        print(f"{name}: {sums(r)}")
        for label, k in (("M1", 0), ("M1a", 1), ("M1b", 2)):
            st = stats(r, base, k)
            print("  " + fmt(f"{name} - base {label}", st))
            if k == 0:
                out[name] = st
    b, text = branch(out["D1"], out["D2"])
    print(f"branch: {b} {text}")
    return b


def parse_seeds(s):
    a, b = s.split("-")
    return range(int(a), int(b) + 1)


def self_test():
    seeds = range(1, 11)
    def mk(vals):
        return {s: (v, v, 0) for s, v in zip(seeds, vals)}
    base = mk([3, 4, 5, 6, 3, 4, 5, 6, 3, 4])
    # D1: 全シードで -2(SE 0)-> A
    d1 = mk([v - 2 for v in [3, 4, 5, 6, 3, 4, 5, 6, 3, 4]])
    d2 = mk([3, 4, 5, 6, 3, 4, 5, 6, 3, 4])
    assert branch(stats(d1, base), stats(d2, base))[0] == "A"
    # D1 が届かず D2 が届く -> B
    d1n = mk([4, 3, 5, 6, 4, 3, 5, 6, 4, 3])
    d2b = mk([v - 1 for v in [3, 4, 5, 6, 3, 4, 5, 6, 3, 4]])
    d2b[1] = (3, 3, 0)   # 1 件だけ対差 0 にして SE を非ゼロにする
    assert branch(stats(d1n, base), stats(d2b, base))[0] == "B"
    # 両方に当たれば A
    assert branch(stats(d1, base), stats(d2b, base))[0] == "A"
    # どちらも届かない -> C
    assert branch(stats(d1n, base), stats(d2, base))[0] == "C"
    # 境界: 平均 = -2 SE ちょうどは届く
    assert branch((-2.0, 1.0), (0.0, 1.0))[0] == "A"
    assert branch((-1.99, 1.0), (-2.0, 1.0))[0] == "B"
    assert branch((-1.99, 1.0), (-1.99, 1.0))[0] == "C"
    assert branch((0.0, 0.0), (0.0, 0.0))[0] == "C"   # 差が全て 0 は届かない
    # 平均と SE の値
    m, se = stats(mk([2, 4]) if False else {1: (2, 2, 0), 2: (4, 4, 0), 3: (6, 6, 0)}, {1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)})
    assert abs(m - 4.0) < 1e-12 and abs(se - 2.0 / math.sqrt(3)) < 1e-12
    print("self-test OK")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--base")
    p.add_argument("--d1")
    p.add_argument("--d2")
    p.add_argument("--legacy")
    p.add_argument("--scenario", default="corridor_oncoming")
    p.add_argument("--seeds", default="521-620")
    p.add_argument("--self-test", action="store_true")
    a = p.parse_args()
    if a.self_test:
        self_test()
        return
    if not (a.base and a.d1 and a.d2):
        fail("--base, --d1, --d2 are required")
    seeds = parse_seeds(a.seeds)
    base = load(a.base, a.scenario, seeds)
    d1 = load(a.d1, a.scenario, seeds)
    d2 = load(a.d2, a.scenario, seeds)
    legacy = load(a.legacy, a.scenario, seeds) if a.legacy else None
    report(base, d1, d2, legacy)


if __name__ == "__main__":
    main()
