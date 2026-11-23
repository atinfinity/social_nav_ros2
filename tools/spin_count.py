#!/usr/bin/env python3
"""譲るときの 1 周: sim の --trace から「1 周」を数える。定義は docs/sim_results.md の `## 譲るときの 1 周` 節(これが正)。

使い方:
  spin_count.py row TRACE [TRACE ...]     各 trace の 1 行(CSV、ヘッダなし)を標準出力へ
  spin_count.py header                    CSV のヘッダを出す
  spin_count.py table CSV [CSV ...]       CSV から発生率の表(全体とシナリオ別)を出す
1 周 = 累積 theta の (最大 - 最小) >= 270 度。向きは最大と最小の時刻の前後(最小が先なら +)。
"""
import math
import sys

COLS = ["scenario", "seed", "range_deg", "dir_deg", "t_start", "t_end", "spin270", "spin180",
        "proceed_frac", "slow_frac", "frames",
        "goal_frac", "early_frac", "yw_frac", "t_onset30"]   # 後ろ 4 列は参考(合否に使わない)
SLOW = 0.05  # [m/s]


def analyze(path):
    scen, seed = "", ""
    goal = None
    fr = []  # (t, x, y, theta, behavior)
    with open(path) as f:
        for line in f:
            if line.startswith("H "):
                p = line.split()
                scen, seed = p[1], p[2]
            elif line.startswith("G "):
                p = line.split()
                goal = (float(p[1]), float(p[2]))
            elif line.startswith("F "):
                p = line.split(None, 6)
                fr.append((float(p[1]), float(p[2]), float(p[3]), float(p[4]), p[5]))
    if len(fr) < 2:
        return [scen, seed, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0, len(fr), 0.0, 0.0, 0.0, 0.0]
    th = [r[3] for r in fr]
    imax = max(range(len(th)), key=lambda i: th[i])
    imin = min(range(len(th)), key=lambda i: th[i])
    rng = math.degrees(th[imax] - th[imin])
    a, b = sorted((imin, imax))
    signed = math.degrees(th[b] - th[a])  # 先 -> 後の変化(最小が先なら +)
    seg = fr[a:b + 1]
    proc = sum(1 for r in seg if r[4] == "PROCEED") / len(seg)
    slow_n = 0
    n = 0
    for i in range(1, len(seg)):
        dt = seg[i][0] - seg[i - 1][0]
        if dt <= 0:
            continue
        v = math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2]) / dt
        n += 1
        slow_n += v < SLOW
    slow = slow_n / n if n else 0.0
    # 参考: 回転量(|dtheta| の和)のうち、目標から 1.0 m 以内 / t < 2 s で起きた割合、区間の YIELD+WAIT の割合、
    # 区間の最初の向きから 30 度ずれた最初の時刻(回転の開始の目安)
    tot = g_in = e_in = 0.0
    onset = fr[b][0]
    found = False
    for i in range(a + 1, b + 1):
        d = abs(fr[i][3] - fr[i - 1][3])
        tot += d
        if goal and math.hypot(fr[i][1] - goal[0], fr[i][2] - goal[1]) <= 1.0:
            g_in += d
        if fr[i][0] < 2.0:
            e_in += d
        if not found and abs(fr[i][3] - fr[a][3]) >= math.radians(30):
            onset, found = fr[i][0], True
    yw = sum(1 for r in seg if r[4] in ("YIELD", "WAIT")) / len(seg)
    return [scen, seed, round(rng, 1), round(signed, 1), fr[a][0], fr[b][0],
            int(rng >= 270), int(rng >= 180), round(proc, 3), round(slow, 3), len(fr),
            round(g_in / tot, 3) if tot else 0.0, round(e_in / tot, 3) if tot else 0.0, round(yw, 3), onset]


def table(files):
    import csv
    rows = []
    for fn in files:
        with open(fn) as f:
            rows += list(csv.DictReader(f))
    by = {}
    for r in rows:
        by.setdefault(r["scenario"], []).append(r)
    print("| scenario | runs | spin>=270 | spin>=180 | rate270 |")
    print("|---|---|---|---|---|")
    order = list(by)
    tot = [0, 0, 0]
    for s in order:
        rs = by[s]
        n, a, b = len(rs), sum(int(r["spin270"]) for r in rs), sum(int(r["spin180"]) for r in rs)
        tot = [tot[0] + n, tot[1] + a, tot[2] + b]
        print(f"| {s} | {n} | {a} | {b} | {a / n:.3f} |")
    if tot[0]:
        print(f"| total | {tot[0]} | {tot[1]} | {tot[2]} | {tot[1] / tot[0]:.3f} |")
    print()
    for r in rows:
        if int(r["spin270"]):
            print(f"spin: {r['scenario']} seed={r['seed']} range={r['range_deg']} dir={r['dir_deg']} "
                  f"t={r['t_start']}..{r['t_end']} proceed={r['proceed_frac']} slow={r['slow_frac']}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cmd = sys.argv[1]
    if cmd == "header":
        print(",".join(COLS))
    elif cmd == "row":
        for p in sys.argv[2:]:
            print(",".join(str(x) for x in analyze(p)))
    elif cmd == "table":
        table(sys.argv[2:])
    else:
        sys.exit(__doc__)
