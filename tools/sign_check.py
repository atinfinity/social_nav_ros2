#!/usr/bin/env python3
"""譲るときの 1 周の案 H の段 1: crowd の 200 本の M1 と M1a の符号検定(規則は docs/sim_results.md の「譲るときの 1 周の案 H の測定の規則」)。標準ライブラリのみ。

使い方:
  sign_check.py --before before.csv --after after.csv [--scenario crowd] [--seeds 3901-4100] [--alpha 0.10]
  sign_check.py --self-test
入力は social_nav_sim の --csv(または run_sim_eval.sh の social_<範囲>.csv)。d = 後 - 前(シードごと)。n+ = d > 0、n- = d < 0、d = 0 は検定から除く(本数は記録)。
(S1) M1 と M1a の両方で、片側の正確な符号検定(対立仮説「増える」)P(X >= n+ | n+ + n-, 0.5) > alpha(既定 0.10)。n+ + n- = 0 は p = 1。
(S2) 合計の前後と差は記録のみ(合否に使わない)。
(S3) 未到達(1 - reached)の本数が増えない。
終了コード: 0 = 合格、1 = 不合格(S1 か S3)、2 = 検査エラー(シード不一致、シードが 21-120 に重なる、壊れた値、重複)。
"""
import argparse
import csv
import math
import sys


class CheckError(Exception):
    pass


def parse_seeds(text):
    try:
        a, b = text.split("-")
        a, b = int(a), int(b)
    except ValueError:
        raise CheckError(f"--seeds は A-B の形で指定する: {text}")
    if a > b:
        raise CheckError(f"--seeds の範囲が逆: {text}")
    if a <= 120 and b >= 21:
        raise CheckError(f"シード範囲 {text} が 21-120(判定用)と重なる")
    return list(range(a, b + 1))


def load(path, scenario, seeds):
    """{seed: (M1, M1a, unreached)}。シード集合が seeds に一致しなければ検査エラー。"""
    rows = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r["scenario"] != scenario:
                continue
            s = int(r["seed"])
            if s in rows:
                raise CheckError(f"{path}: シード {s} が重複")
            try:
                m1, m1a, reached = float(r["M1"]), float(r["M1_moving"]), float(r["reached"])
            except ValueError:
                raise CheckError(f"{path}: シード {s} の値が数でない")
            for v in (m1, m1a):
                if not math.isfinite(v) or v < 0 or v != int(v):
                    raise CheckError(f"{path}: シード {s} の M1 / M1_moving が非負の整数でない({v})")
            if reached not in (0.0, 1.0):
                raise CheckError(f"{path}: シード {s} の reached が 0 か 1 でない({reached})")
            rows[s] = (int(m1), int(m1a), 1 - int(reached))
    if set(rows) != set(seeds):
        raise CheckError(f"{path}: シード集合が違う(行 {len(rows)}、期待 {len(seeds)})")
    return rows


def sign_p(n_plus, n_minus):
    """P(X >= n_plus | n = n_plus + n_minus, 0.5)。n = 0 は 1。"""
    n = n_plus + n_minus
    if n == 0:
        return 1.0
    return sum(math.comb(n, k) for k in range(n_plus, n + 1)) / 2 ** n


def column(before, after, k, alpha):
    d = [after[s][k] - before[s][k] for s in sorted(before)]
    n_plus = sum(1 for v in d if v > 0)
    n_minus = sum(1 for v in d if v < 0)
    p = sign_p(n_plus, n_minus)
    return {"n_plus": n_plus, "n_minus": n_minus, "n_zero": len(d) - n_plus - n_minus, "p": p,
            "before": sum(before[s][k] for s in before), "after": sum(after[s][k] for s in after),
            "s1": p > alpha}


def judge(before, after, alpha=0.10):
    """(合格か, {M1, M1a, unreached} の記録)。S2 は合否に使わない。"""
    m1 = column(before, after, 0, alpha)
    m1a = column(before, after, 1, alpha)
    ub, ua = sum(before[s][2] for s in before), sum(after[s][2] for s in after)
    s3 = ua <= ub
    ok = m1["s1"] and m1a["s1"] and s3
    return ok, {"M1": m1, "M1a": m1a, "unreached": {"before": ub, "after": ua, "s3": s3}}


def report(ok, r, alpha):
    for name in ("M1", "M1a"):
        c = r[name]
        print(f"{name}: n+ = {c['n_plus']}, n- = {c['n_minus']}, d=0 = {c['n_zero']}, p = {c['p']:.6f} "
              f"(S1 p > {alpha}: {'OK' if c['s1'] else 'NG'})")
        print(f"{name} S2(記録のみ): 前 {c['before']} 後 {c['after']} 差 {c['after'] - c['before']:+d}")
    u = r["unreached"]
    print(f"未到達: 前 {u['before']} 後 {u['after']} (S3: {'OK' if u['s3'] else 'NG'})")
    print("結果: " + ("合格" if ok else "不合格"))


def self_test():
    import os
    import tempfile
    assert sign_p(0, 0) == 1.0
    assert abs(sign_p(3, 0) - 0.125) < 1e-12
    assert abs(sign_p(5, 5) - 638 / 1024) < 1e-12
    assert sign_p(0, 7) == 1.0
    assert abs(sign_p(10, 0) - 1 / 1024) < 1e-12

    seeds = list(range(3901, 4101))

    def mk(m1, m1a=None, unr=None):
        return {s: (m1(s), (m1a or m1)(s), (unr or (lambda s: 0))(s)) for s in seeds}

    base = mk(lambda s: 2)
    # 同点(d = 0)は除く: 全部同じなら n+ = n- = 0、p = 1、合格
    ok, r = judge(base, base)
    assert ok and r["M1"]["p"] == 1.0 and r["M1"]["n_zero"] == 200
    # 増えるシードが 3 本だけ(n+ = 3、n- = 0、p = 0.125)なら合格、5 本(p = 0.03125)なら S1 不合格
    ok, r = judge(base, mk(lambda s: 3 if s < 3904 else 2))
    assert ok and abs(r["M1"]["p"] - 0.125) < 1e-12 and r["M1"]["n_zero"] == 197
    ok, r = judge(base, mk(lambda s: 3 if s < 3906 else 2))
    assert not ok and not r["M1"]["s1"] and r["M1a"]["s1"] is False
    # M1a だけが S1 不合格でも不合格
    ok, r = judge(base, mk(lambda s: 2, lambda s: 3 if s < 3906 else 2))
    assert not ok and r["M1"]["s1"] and not r["M1a"]["s1"]
    # S3: 未到達が増えると不合格(減る / 同じは合格)
    ok, r = judge(base, mk(lambda s: 2, unr=lambda s: 1 if s == 3901 else 0))
    assert not ok and r["M1"]["s1"] and not r["unreached"]["s3"]
    ok, r = judge(mk(lambda s: 2, unr=lambda s: 1 if s == 3901 else 0), base)
    assert ok
    # S2 は合否に影響しない: 合計が大きく増えても、n+ が少なければ合格(1 本 +50)
    ok, r = judge(base, mk(lambda s: 52 if s == 3901 else 2))
    assert ok and r["M1"]["after"] - r["M1"]["before"] == 50
    # 合計が減っても n+ が多ければ不合格(S2 で救わない)
    ok, r = judge(mk(lambda s: 10 if s == 3901 else 2), mk(lambda s: 2 if s == 3901 else 3 if s < 3908 else 2))
    assert not ok and r["M1"]["after"] < r["M1"]["before"]
    # CLI(ファイル経由): 合格 0、不合格 1、シード不一致 2、21-120 の拒否 2
    def write(path, rows, scenario="crowd"):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["scenario", "seed", "M1", "M1_moving", "reached"])
            for s, (a, b, u) in rows.items():
                w.writerow([scenario, s, a, b, 1 - u])
    with tempfile.TemporaryDirectory() as d:
        b, a = os.path.join(d, "b.csv"), os.path.join(d, "a.csv")
        write(b, base)
        write(a, base)
        assert main(["--before", b, "--after", a]) == 0
        write(a, mk(lambda s: 3 if s < 3906 else 2))
        assert main(["--before", b, "--after", a]) == 1
        short = {s: v for s, v in base.items() if s != 3950}
        write(a, short)
        assert main(["--before", b, "--after", a]) == 2
        write(a, base)
        assert main(["--before", b, "--after", a, "--seeds", "21-120"]) == 2
        assert main(["--before", b, "--after", a, "--seeds", "100-300"]) == 2
        shifted = {s + 1: v for s, v in base.items()}
        write(a, shifted)
        assert main(["--before", b, "--after", a]) == 2
        # 壊れた CSV は 2(不合格の 1 にしない): seed が数でない、小数、列の欠け
        for bad in ("crowd,x,2,2,1\n", "crowd,4100.0,2,2,1\n", "crowd,3950\n"):
            write(a, {s: v for s, v in base.items() if s != 3950})
            with open(a, "a") as f:
                f.write(bad)
            assert main(["--before", b, "--after", a]) == 2
        write(a, base, scenario="corridor_oncoming")
        assert main(["--before", b, "--after", a]) == 2
    print("self-test OK")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--before")
    ap.add_argument("--after")
    ap.add_argument("--scenario", default="crowd")
    ap.add_argument("--seeds", default="3901-4100")
    ap.add_argument("--alpha", type=float, default=0.10)
    ap.add_argument("--self-test", action="store_true")
    try:
        args = ap.parse_args(argv)
    except SystemExit as e:
        return 2 if e.code else 0
    if args.self_test:
        return self_test()
    try:
        if not args.before or not args.after:
            raise CheckError("--before と --after が必要")
        seeds = parse_seeds(args.seeds)
        before = load(args.before, args.scenario, seeds)
        after = load(args.after, args.scenario, seeds)
    except (CheckError, OSError, KeyError, ValueError, TypeError, csv.Error) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    ok, r = judge(before, after, args.alpha)
    report(ok, r, args.alpha)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
