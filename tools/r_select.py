#!/usr/bin/env python3
"""R2(stop_aware)の選定と判定を all.csv(tools/run_sim_eval.sh の出力)から行う。標準ライブラリのみ。
規則の全文は docs/sim_results.md の「接触直前の止まり方: 規則の固定」(測定の前に固定したもの)。段の基準は m4_select.py を流用する。

使い方:
  r_select.py --cand-sel C/all.csv --base-sel B/all.csv     # 選定 1-20   (候補 = on、基準 = 現行の既定)
              --cand-tgt C/all.csv --base-tgt B/all.csv     # 選定 621-720
  r_select.py --cand-judge C/all.csv --base-judge B/all.csv # 判定 21-120(採用点だけ 1 回)

選定(点は on の 1 つ):
  段の基準 = m4_select.py の段(5 構成 x 6 シナリオ。1-20 は M1 合計と M1a が増えない。621-720 は M1 合計、M1a、M6 の
  対差の平均 <= 2*SE。両範囲で M6 <= 5%、social は M5・M9・M3)。加えて crowd の social の M1a の対差(候補 - 現行)の平均
  <= -SE(両範囲。差が全て 0 なら満たす)。通らなければ判定は回さない。
  併記: 621-720 の social の corridor_oncoming + crowd の M1 合計(現行との差)、M1a -> M1b の移動件数
  min(-dM1a, max(0, dM1b))(改善に数えない)。
判定(21-120):
  (1) 全 30 マスで M1 合計(M1a+M1b)が現行を超えない。
  (2) social の corridor_oncoming と crowd の M1a のシード別対差(候補 - 現行)の平均 <= -SE。
  (3) social の M1a の合計の上限 31 / 0 / 159(corridor_oncoming / crossing / crowd)、ほか 0。
  (4) 全マスで M6 <= 5%。social は M5(baseline 比 +30%、crowd・static_group は +50%)、M9(平均 <= 20)、
      M3(static_group・crowd で no_tactical 以下)、M1b(social の合計 <= 同じシナリオの baseline 構成)。
入力の検査: 構成・シナリオ・シード集合(範囲どおり)・本数。baseline 構成の行は現行と一致。違えばエラー(終了コード 2)。
終了コード: 検査エラー 2、段の基準または判定が不合格 1、選定で採用 0(判定が合格のときも 0)、段は合格だが不採用 3。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import m4_select as m4  # noqa: E402

RANGES = {"sel": ("select", 1, 20), "tgt": ("select", 621, 100), "judge": ("judge", 21, 100)}
CAPS = {"corridor_oncoming": 31, "crossing": 0, "crowd": 159}   # social の M1a の上限(ほかは 0)
M1_SCENARIOS = ("corridor_oncoming", "crowd")


def fail(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(2)


m4.fail = fail   # 入力の検査エラーは終了コード 2


def total(cell, f):
    return sum(f(cell))


def movement(cand, base, s):
    """M1a -> M1b の移動件数 min(-dM1a, max(0, dM1b))(social。改善に数えない)。"""
    c, b = cand[("social", s)], base[("social", s)]
    da = total(c, m4.m1a) - total(b, m4.m1a)
    db = sum(m4.vals(c, "M1_stopped")) - sum(m4.vals(b, "M1_stopped"))
    return da, db, max(0.0, min(-da, max(0.0, db)))


def select_report(label, cand, base):
    print(f"== 併記 {label}(social。M1 合計 = M1a + M1b。移動は改善に数えない)")
    print(f"{'scenario':<18}{'M1':>6}{'(現行)':>8}{'dM1a':>7}{'dM1b':>7}{'移動件数':>9}")
    for s in m4.SCENARIOS:
        da, db, mv = movement(cand, base, s)
        print(f"{s:<18}{total(cand[('social', s)], m4.m1ab):>6g}{total(base[('social', s)], m4.m1ab):>8g}"
              f"{da + 0.0:>+7g}{db + 0.0:>+7g}{mv:>9g}")
    c = sum(total(cand[("social", s)], m4.m1ab) for s in M1_SCENARIOS)
    b = sum(total(base[("social", s)], m4.m1ab) for s in M1_SCENARIOS)
    print(f"-> {label}: corridor_oncoming + crowd の M1 合計 {c:g}(現行 {b:g}、差 {c - b:+g})")


def run_select(a):
    ok = True
    margin_ok = True
    ran = False
    for k, label in (("sel", "1-20"), ("tgt", "621-720")):
        cp, bp = getattr(a, "cand_" + k), getattr(a, "base_" + k)
        if cp is None and bp is None:
            continue
        if cp is None or bp is None:
            fail(f"--cand-{k} と --base-{k} は両方必要")
        rname, start, n = RANGES[k]
        cand, base = m4.load(cp, rname, start, n), m4.load(bp, rname, start, n)
        m4.baseline_check(label, cand, base)
        ran = True
        ok &= m4.stage(label, cand, base, k) == 0
        margin_ok &= m4.margin(label, cand, base)
        select_report(label, cand, base)
    if not ran:
        fail("範囲が指定されていない")
    if not ok:
        print("総合: 段の基準 不合格(判定は回さない)")
        return 1
    if not margin_ok:
        print("総合: 段の基準 合格、crowd の M1a に余裕なし。採用しない(判定は回さない)")
        return 3
    print("総合: 段の基準 合格、crowd の M1a の余裕あり。採用(判定へ)")
    return 0


def run_judge(cand, base):
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"   {'合格' if cond else '不合格'}: {msg}")
        ok &= cond

    print("== 判定 21-120(採用点だけ 1 回)")
    for c in m4.CONFIGS:
        for s in m4.SCENARIOS:
            x, y = total(cand[(c, s)], m4.m1ab), total(base[(c, s)], m4.m1ab)
            check(x <= y, f"(1) {c}/{s} M1 合計 {x:g} <= 現行 {y:g}")
    for s in M1_SCENARIOS:
        mu, se = m4.paired(m4.m1a(cand[("social", s)]), m4.m1a(base[("social", s)]))
        check(mu <= -se + 1e-12, f"(2) social/{s} M1a の対差 {mu:+.3f}/シード <= -SE {-se:.3f}")
    for s in m4.SCENARIOS:
        x, cap = total(cand[("social", s)], m4.m1a), CAPS.get(s, 0)
        check(x <= cap, f"(3) social/{s} M1a {x:g} <= 上限 {cap}")
    for c in m4.CONFIGS:
        for s in m4.SCENARIOS:
            r6 = m4.m6(cand[(c, s)])
            check(r6 <= m4.M6_MAX + 1e-12, f"(4) {c}/{s} M6 {r6:.1%} <= {m4.M6_MAX:.0%}")
    for s in m4.SCENARIOS:
        cc = cand[("social", s)]
        m5s, m5b = m4.mean(m4.vals(cc, "M5")), m4.mean(m4.vals(cand[("baseline", s)], "M5"))
        lim = m4.M5_LIMIT.get(s, m4.M5_DEFAULT)
        check(m5s <= lim * m5b, f"(4) social/{s} M5 {m5s:.1f} <= {lim:.1f}x{m5b:.1f}")
        m9 = m4.mean(m4.vals(cc, "M9"))
        check(m9 <= m4.M9_MAX, f"(4) social/{s} M9 {m9:.1f} <= {m4.M9_MAX:.0f}")
        if s in ("static_group", "crowd"):
            m3s, m3n = m4.mean(m4.vals(cc, "M3")), m4.mean(m4.vals(cand[("no_tactical", s)], "M3"))
            check(m3s <= m3n + 1e-12, f"(4) social/{s} M3 {m3s:.3f} <= no_tactical {m3n:.3f}")
        b1s = sum(m4.vals(cc, "M1_stopped"))
        b1b = sum(m4.vals(cand[("baseline", s)], "M1_stopped"))
        check(b1s <= b1b, f"(4) social/{s} M1b {b1s:g} <= baseline {b1b:g}")
    print("総合: 判定 " + ("合格" if ok else "不合格(既定を戻し、値を記録する。回し直さない)"))
    return 0 if ok else 1


def self_test():
    import csv
    import subprocess
    import tempfile

    def write(path, rname, start, n, mod=None):
        cols = ["config", "scenario", "range", "seed", "M1_moving", "M1_stopped", "M3", "M4", "M5", "M9", "reached"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for c in m4.CONFIGS:
                for s in m4.SCENARIOS:
                    for i in range(n):
                        row = {"M1_moving": 0, "M1_stopped": 0, "M3": 0.0, "M4": 1.0, "M5": 1.0, "M9": 1.0, "reached": 1}
                        if mod:
                            mod(row, c, s, i)
                        w.writerow([c, s, rname, start + i] + [row[k] for k in cols[4:]])
        return path

    def crowd_base(row, c, s, i):   # 現行: social の crowd は 1 件/シード
        if c == "social" and s == "crowd":
            row["M1_moving"] = 1

    def crowd_better(row, c, s, i):   # 候補: M1a 1 -> 0、最初の 3 シードだけ M1b が 1(移動 3 件)
        if c == "social" and s == "crowd":
            row["M1_stopped"] = 1 if i < 3 else 0

    def crowd_m6_bad(row, c, s, i):
        crowd_better(row, c, s, i)
        if c == "social" and s == "crowd" and i < 4:
            row["reached"] = 0

    me = os.path.abspath(__file__)

    def run(*args):
        return subprocess.run([sys.executable, me, *args], capture_output=True, text=True)

    with tempfile.TemporaryDirectory() as d:
        p = lambda name: os.path.join(d, name)  # noqa: E731
        sb, tb = write(p("sb"), "select", 1, 20, crowd_base), write(p("tb"), "select", 621, 100, crowd_base)
        sc, tc = write(p("sc"), "select", 1, 20, crowd_better), write(p("tc"), "select", 621, 100, crowd_better)
        # 現行と同一: 差が全て 0 -> 余裕あり、採用
        r = run("--cand-sel", sb, "--base-sel", sb, "--cand-tgt", tb, "--base-tgt", tb)
        assert r.returncode == 0 and "採用(判定へ)" in r.stdout, r.stdout + r.stderr
        # crowd の M1a が 1 -> 0(M1 合計は 20 -> 3、100 -> 3)。移動 = min(20, 3) = 3、min(100, 3) = 3
        r = run("--cand-sel", sc, "--base-sel", sb, "--cand-tgt", tc, "--base-tgt", tb)
        assert r.returncode == 0 and "crowd           " in r.stdout, r.stdout + r.stderr
        assert "crowd             " in r.stdout and "dM1a" in r.stdout, r.stdout
        rows = [ln.split() for ln in r.stdout.splitlines() if ln.startswith("crowd ")]
        mv = [ln for ln in rows if len(ln) == 6]   # 併記の行: scenario M1 現行 dM1a dM1b 移動
        assert [x[5] for x in mv] == ["3", "3"] and mv[1][3] == "-100", rows
        assert "M1 合計 3(現行 100、差 -97)" in r.stdout, r.stdout
        # 621-720 の crowd の M6 が 0 -> 4/100(絶対値は 5% 以下だが対差が 2SE 超): 段の基準で不合格
        tm = write(p("tm"), "select", 621, 100, crowd_m6_bad)
        r = run("--cand-sel", sc, "--base-sel", sb, "--cand-tgt", tm, "--base-tgt", tb)
        assert r.returncode == 1 and "M6 mean" in r.stdout, r.stdout + r.stderr
        # 余裕なし: crowd の M1a の対差が -1 と +1 が 1 シードずつ(平均 0 > -SE)
        def crowd_weak(row, c, s, i):
            if c == "social" and s == "crowd":
                row["M1_moving"] = 0 if i == 0 else (2 if i == 1 else 1)
        sw, tw = write(p("sw"), "select", 1, 20, crowd_weak), write(p("tw"), "select", 621, 100, crowd_weak)
        r = run("--cand-sel", sw, "--base-sel", sb, "--cand-tgt", tw, "--base-tgt", tb)
        assert r.returncode == 3 and "余裕なし" in r.stdout, r.stdout + r.stderr
        # 範囲違いは検査エラー
        r = run("--cand-sel", tb, "--base-sel", tb)
        assert r.returncode == 2, r.stdout + r.stderr
        # 判定: 現行 crowd M1a 100(<= 159)。候補 0 -> 合格
        jb = write(p("jb"), "judge", 21, 100, crowd_base)
        jc = write(p("jc"), "judge", 21, 100)
        r = run("--cand-judge", jc, "--base-judge", jb)
        assert r.returncode == 0 and "判定 合格" in r.stdout, r.stdout + r.stderr
        # 候補 2/シード(200 > 159、対差 +1、M1 合計が現行超え)-> 不合格
        def crowd_worse(row, c, s, i):
            if c == "social" and s == "crowd":
                row["M1_moving"] = 2
        jw = write(p("jw"), "judge", 21, 100, crowd_worse)
        r = run("--cand-judge", jw, "--base-judge", jb)
        assert r.returncode == 1 and "不合格: (3) social/crowd M1a 200 <= 上限 159" in r.stdout, r.stdout + r.stderr
        assert "不合格: (1) social/crowd M1 合計 200 <= 現行 100" in r.stdout, r.stdout
    print("r_select self-test: OK")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k in ("cand-sel", "base-sel", "cand-tgt", "base-tgt", "cand-judge", "base-judge"):
        p.add_argument("--" + k)
    p.add_argument("--self-test", action="store_true")
    a = p.parse_args()
    if a.self_test:
        self_test()
        return
    if a.cand_judge or a.base_judge:
        if not (a.cand_judge and a.base_judge):
            fail("--cand-judge と --base-judge は両方必要")
        if any((a.cand_sel, a.base_sel, a.cand_tgt, a.base_tgt)):
            fail("判定は選定と同時に指定しない")
        rname, start, n = RANGES["judge"]
        cand, base = m4.load(a.cand_judge, rname, start, n), m4.load(a.base_judge, rname, start, n)
        m4.baseline_check("21-120", cand, base)
        sys.exit(run_judge(cand, base))
    sys.exit(run_select(a))


if __name__ == "__main__":
    main()
