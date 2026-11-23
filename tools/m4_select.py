#!/usr/bin/env python3
"""M4 の改善: 段の基準・M4 の余裕つき目標・点の順位・採用の条件を all.csv(tools/run_sim_eval.sh の出力)から判定する。標準ライブラリのみ。
規則の全文は docs/sim_results.md の「M4 の改善」節(測定の前に固定したもの)。

使い方:
  m4_select.py --report C/all.csv                      # 421-520 の区分・r・順位だけを表示(直前なし。現行の測定用)
  m4_select.py --cand-sel C/all.csv --base-sel B/all.csv   # 1-20  (候補, 直前の採用点)
               --cand-tgt C/all.csv --base-tgt B/all.csv   # 421-520 (採用の条件は 421-520 が必要)

段の基準(5 構成 x 6 シナリオの各マス。直前の採用点と比べる):
  1-20      : M1a+M1b の合計と M1a の合計が増えない。
  421-520   : シード別対差(候補 - 直前)の平均 <= 2*SE(SE = 標本標準偏差/sqrt(n))。M1a+M1b、M1a、M6 の対象。
  両範囲    : 候補の M6(= 未到達の割合)<= 5%。social は M5(baseline 比 +30%、crowd・static_group は +50%)、
              M9(平均 <= 20)、M3(static_group・crowd で social <= no_tactical)。baseline の行は直前と一致(入力の検査)。
M4 の余裕つき目標(421-520、区分はその all.csv の baseline 平均): 1.0 s 以上は social が baseline 比 35% 以上減
  (目標値 = 0.65 x baseline)、未満は social <= 0.8 s。r = social / 目標値(r <= 1 で到達)。6 シナリオ全部。
点の順位: 段の基準を満たす点のうち、(1) r > 1 のシナリオ数が最少、(2) 同数なら r の最大値が最小。
採用の条件: 直前より順位が良い、直前で r > 1 のシナリオの少なくとも 1 つで M4 のシード別対差(直前 - 候補)の平均 > 2*SE、
  直前で r <= 1 のシナリオを r > 1 にしない。加えて crowd の M1a の余裕: social の対差(候補 - 直前)の平均 <= -SE
  (両範囲。差が全て 0 なら満たす)。
入力の検査: 構成・シナリオ・シード集合(範囲どおり)・本数。違えばエラーで止まる。
終了コード: 検査エラー 2、段の基準が不合格 1、段は合格で採用 0、段は合格だが不採用 3。
"""
import argparse
import csv
import math
import sys

CONFIGS = ["social", "baseline", "no_tactical", "use_tracker", "ignore_robot"]
SCENARIOS = ["corridor_oncoming", "crossing", "overtake", "follow", "static_group", "crowd"]
RANGES = {"sel": ("select", 1, 20), "tgt": ("select", 421, 100)}
M6_MAX = 0.05
M4_REL = 0.35        # 相対: baseline 比 35% 以上減
M4_ABS = 0.8         # 絶対: social <= 0.8 s
M4_SPLIT = 1.0       # baseline 平均がこれ以上なら相対
M5_LIMIT = {"static_group": 1.5, "crowd": 1.5}
M5_DEFAULT = 1.3
M9_MAX = 20.0
INVARIANT_COLS = ["M1_moving", "M1_stopped", "M4", "M5", "M9", "reached"]


def fail(msg):
    sys.exit(f"error: {msg}")


def load(path, rname, start, n):
    """{(config, scenario): {seed: row}} を返し、構成・シナリオ・シード集合・本数を検査する。"""
    cells = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            key = (r["config"], r["scenario"])
            seed = int(r["seed"])
            if r["range"] != rname:
                fail(f"{path}: range={r['range']!r}, expected {rname!r}")
            if seed in cells.setdefault(key, {}):
                fail(f"{path}: duplicate {key} seed {seed}")
            cells[key][seed] = r
    want = set(range(start, start + n))
    for c in CONFIGS:
        for s in SCENARIOS:
            d = cells.get((c, s))
            if d is None:
                fail(f"{path}: missing cell {c}/{s}")
            if set(d) != want:
                fail(f"{path}: {c}/{s} seeds differ from {start}..{start + n - 1} (rows={len(d)}, expected {n})")
    extra = set(cells) - {(c, s) for c in CONFIGS for s in SCENARIOS}
    if extra:
        fail(f"{path}: unexpected cells {sorted(extra)}")
    return cells


def vals(cell, col):
    return [float(cell[s][col]) for s in sorted(cell)]


def mean(x):
    return sum(x) / len(x)


def m1a(cell):
    return vals(cell, "M1_moving")


def m1ab(cell):
    return [a + b for a, b in zip(vals(cell, "M1_moving"), vals(cell, "M1_stopped"))]


def m6(cell):
    return 1 - mean(vals(cell, "reached"))


def m6s(cell):
    return [1 - v for v in vals(cell, "reached")]


def paired(x, y):
    """x - y のシード別対差の (平均, SE)。"""
    d = [a - b for a, b in zip(x, y)]
    n = len(d)
    mu = mean(d)
    sd = math.sqrt(sum((v - mu) ** 2 for v in d) / (n - 1)) if n > 1 else 0.0
    return mu, sd / math.sqrt(n)


def baseline_check(label, cand, base):
    """baseline 構成の行が直前と一致するか(パラメータを変えても baseline は変わらない)。"""
    for s in SCENARIOS:
        for seed in sorted(base[("baseline", s)]):
            for col in INVARIANT_COLS:
                if cand[("baseline", s)][seed][col] != base[("baseline", s)][seed][col]:
                    fail(f"{label}: baseline/{s} seed {seed} の {col} が直前と違う")


def stage(label, cand, base, mode):
    """段の基準。全マスの合否を表示して、不合格のマス数を返す。"""
    print(f"== 段の基準 {label}({'各マスで増えない' if mode == 'sel' else '対差の平均 <= 2*SE'}、M6 <= {M6_MAX:.0%}、"
          "social は M5・M9・M3)")
    print(f"{'config':<13}{'scenario':<18}{'M1a+M1b':>9}{'(base)':>8}{'M1a':>6}{'(base)':>8}{'M6':>7}  result")
    nfail = 0
    for c in CONFIGS:
        for s in SCENARIOS:
            cc, bb = cand[(c, s)], base[(c, s)]
            ok = True
            notes = []
            for name, f in (("M1a+M1b", m1ab), ("M1a", m1a)):
                x, y = f(cc), f(bb)
                if mode == "sel":
                    bad = sum(x) > sum(y)
                    detail = f"{sum(x):g}>{sum(y):g}"
                else:
                    mu, se = paired(x, y)
                    bad = mu > 2 * se
                    detail = f"mean {mu:+.3f} > 2SE {2 * se:.3f}"
                if bad:
                    ok = False
                    notes.append(f"{name} {detail}")
            r6 = m6(cc)
            if mode == "tgt":
                mu, se = paired(m6s(cc), m6s(bb))
                if mu > 2 * se:
                    ok = False
                    notes.append(f"M6 mean {mu:+.3f} > 2SE {2 * se:.3f}")
            if r6 > M6_MAX + 1e-12:
                ok = False
                notes.append(f"M6 {r6:.1%}")
            if c == "social":
                m5s, m5b = mean(vals(cc, "M5")), mean(vals(cand[("baseline", s)], "M5"))
                lim = M5_LIMIT.get(s, M5_DEFAULT)
                if m5s > lim * m5b:
                    ok = False
                    notes.append(f"M5 {m5s:.1f} > {lim:.1f}x{m5b:.1f}")
                m9 = mean(vals(cc, "M9"))
                if m9 > M9_MAX:
                    ok = False
                    notes.append(f"M9 {m9:.1f}")
                if s in ("static_group", "crowd"):
                    m3s, m3n = mean(vals(cc, "M3")), mean(vals(cand[("no_tactical", s)], "M3"))
                    if m3s > m3n + 1e-12:
                        ok = False
                        notes.append(f"M3 {m3s:.3f} > no_tactical {m3n:.3f}")
            nfail += not ok
            print(f"{c:<13}{s:<18}{sum(m1ab(cc)):>9g}{sum(m1ab(bb)):>8g}{sum(m1a(cc)):>6g}{sum(m1a(bb)):>8g}"
                  f"{r6:>7.1%}  {'合格' if ok else '不合格: ' + '; '.join(notes)}")
    print(f"-> {label}: 不合格 {nfail} / {len(CONFIGS) * len(SCENARIOS)} マス")
    return nfail


def targets(cand):
    """シナリオごとの {区分, social, baseline, 目標値, r}(421-520)。"""
    out = {}
    for s in SCENARIOS:
        soc = mean(vals(cand[("social", s)], "M4"))
        bas = mean(vals(cand[("baseline", s)], "M4"))
        if bas >= M4_SPLIT:
            kind, goal = "相対", (1 - M4_REL) * bas
        else:
            kind, goal = "絶対", M4_ABS
        out[s] = {"kind": kind, "soc": soc, "bas": bas, "goal": goal, "r": soc / goal}
    return out


def rank(t):
    over = [v["r"] for v in t.values() if v["r"] > 1 + 1e-12]
    return (len(over), max(v["r"] for v in t.values()))


def print_targets(label, t):
    print(f"== M4 の余裕つき目標 {label}(相対 = baseline 比 {M4_REL:.0%} 以上減、絶対 = social <= {M4_ABS} s。r = social/目標値)")
    print(f"{'scenario':<18}{'区分':<5}{'baseline':>9}{'social':>8}{'目標値':>8}{'r':>7}  到達")
    for s in SCENARIOS:
        v = t[s]
        print(f"{s:<18}{v['kind']:<5}{v['bas']:>9.3f}{v['soc']:>8.3f}{v['goal']:>8.3f}{v['r']:>7.3f}  "
              f"{'到達' if v['r'] <= 1 + 1e-12 else '未到達'}")
    n, mx = rank(t)
    print(f"-> 順位キー: r > 1 のシナリオ数 {n}、r の最大値 {mx:.3f}")


def margin(label, cand, base):
    """crowd の M1a の余裕(social の対差の平均 <= -SE)。"""
    mu, se = paired(m1a(cand[("social", "crowd")]), m1a(base[("social", "crowd")]))
    ok = mu <= -se + 1e-12
    print(f"   crowd M1a の余裕 {label}: 対差 {mu:+.3f}/シード(SE {se:.3f})  {'余裕あり' if ok else '余裕なし'}")
    return ok


def adopt(cand, base):
    tc, tb = targets(cand), targets(base)
    print_targets("候補", tc)
    print_targets("直前", tb)
    better = rank(tc) < rank(tb)
    print(f"-> 順位: 候補 {rank(tc)} / 直前 {rank(tb)}  {'良い' if better else '良くない'}")
    improved, broke = [], []
    for s in SCENARIOS:
        if tb[s]["r"] > 1 + 1e-12:
            mu, se = paired(vals(base[("social", s)], "M4"), vals(cand[("social", s)], "M4"))
            hit = mu > 2 * se
            print(f"   {s}: M4 の改善(直前 - 候補) {mu:+.3f} s、2SE {2 * se:.3f}  {'超える' if hit else '超えない'}")
            if hit:
                improved.append(s)
        elif tc[s]["r"] > 1 + 1e-12:
            broke.append(s)
    if broke:
        print(f"   直前で到達だったのに未到達になった: {', '.join(broke)}")
    return better and bool(improved) and not broke


def self_test():
    import os
    import subprocess
    import tempfile

    def write(path, rname, start, n, bad_m6=0):
        cols = ["config", "scenario", "range", "seed"] + INVARIANT_COLS + ["M3"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for c in CONFIGS:
                for s in SCENARIOS:
                    for i in range(n):
                        reached = 0 if (c == "social" and s == "crowd" and i < bad_m6) else 1
                        m4 = 2.0 if c == "baseline" else 1.0
                        w.writerow([c, s, rname, start + i, 0, 0, m4, 1.0, 1.0, reached, 0.0])

    me = os.path.abspath(__file__)
    with tempfile.TemporaryDirectory() as d:
        a, b = os.path.join(d, "a.csv"), os.path.join(d, "b.csv")
        write(a, "select", 421, 100)
        r = subprocess.run([sys.executable, me, "--cand-tgt", a, "--base-tgt", a], capture_output=True, text=True)
        assert r.returncode == 3 and "採用しない" in r.stdout, r.stdout + r.stderr
        write(b, "select", 421, 100, bad_m6=4)   # M6 が 0/100 -> 4/100(絶対値は 5% 以下だが対差が 2SE 超)
        r = subprocess.run([sys.executable, me, "--cand-tgt", b, "--base-tgt", a], capture_output=True, text=True)
        assert r.returncode == 1 and "M6 mean" in r.stdout, r.stdout + r.stderr
    print("m4_select self-test: OK")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k in ("cand-sel", "base-sel", "cand-tgt", "base-tgt", "report"):
        p.add_argument("--" + k)
    p.add_argument("--self-test", action="store_true")
    a = p.parse_args()
    if a.self_test:
        self_test()
        return
    if a.report:
        rname, start, n = RANGES["tgt"]
        print_targets("421-520", targets(load(a.report, rname, start, n)))
        return
    ok = True
    ran = False
    loaded = {}
    margin_ok = True
    for k, label in (("sel", "1-20"), ("tgt", "421-520")):
        cp, bp = getattr(a, "cand_" + k), getattr(a, "base_" + k)
        if cp is None and bp is None:
            continue
        if cp is None or bp is None:
            fail(f"--cand-{k} と --base-{k} は両方必要")
        rname, start, n = RANGES[k]
        cand, base = load(cp, rname, start, n), load(bp, rname, start, n)
        baseline_check(label, cand, base)
        loaded[k] = (cand, base)
        ran = True
        ok &= stage(label, cand, base, k) == 0
        margin_ok &= margin(label, cand, base)
    if not ran:
        fail("範囲が指定されていない")
    if not ok:
        print("総合: 段の基準 不合格")
        sys.exit(1)
    if "tgt" not in loaded:
        print("総合: 段の基準 合格(421-520 がないので採用は判定しない)")
        sys.exit(0)
    cand, base = loaded["tgt"]
    cond = adopt(cand, base)
    if not margin_ok:
        print("   crowd の M1a に余裕なし")
    res = cond and margin_ok
    print("総合: 段の基準 合格、" + ("採用" if res else "採用しない"))
    sys.exit(0 if res else 3)


if __name__ == "__main__":
    main()
