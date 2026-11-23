#!/usr/bin/env python3
"""TTC の副次確認: TTC の人のモデル(run_sim_eval.sh の TTC=1 が出す $OUT_DIR/ttc/)で、変更前後の social を (シナリオ, シード) の対差で比べる。
規則の全文は docs/sim_results.md の「TTC の副次確認: 規則の固定」(測定の前に固定したもの)。標準ライブラリのみ。

使い方:
  ttc_check.py --before DIR --after DIR [--range select] [--config social]
  ttc_check.py --self-test
各 DIR は .../ttc(<config>_<range>.csv と conditions.txt がある)。対差 d = 変更後 - 変更前。SE = 標本標準偏差 / sqrt(n)。
要確認: F1 = M1 合計(CSV の M1)の d の平均 > 0 かつ >= 2 SE、F2 = 未到達(1 - reached)の d の平均 > 0 かつ >= 2 SE。
注意(止めない): crowd / corridor_oncoming の M2 が 2 SE 以上下がる、M5 が 2 SE 以上増える(変更前の M1 合計 <= 5 で M1 が増えたときも)。
1 シナリオ 100 本未満の結果は「参考」(判定を出さない)。
終了コード: 悪化なし(と参考)0、要確認 1、検査エラー 2。
"""
import argparse
import csv
import math
import os
import sys

MIN_SEEDS = 100
JUDGE_LO, JUDGE_HI = 21, 120   # 判定用の範囲。TTC の確認には使わない
RULE_SCENARIOS = ("corridor_oncoming", "crossing", "overtake", "follow", "static_group", "crowd")
METRICS = ("M1_moving", "M1_stopped", "M1", "unreached", "M2", "M4", "M5")
ATTN_SCENARIOS = ("crowd", "corridor_oncoming")


class CheckError(Exception):
    pass


def fail(msg):
    raise CheckError(msg)


def load_csv(path):
    """{(scenario, seed): {metric: float}}"""
    rows = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            key = (r["scenario"], int(r["seed"]))
            if key in rows:
                fail(f"{path}: duplicate {key}")
            rows[key] = {
                "M1_moving": float(r["M1_moving"]), "M1_stopped": float(r["M1_stopped"]), "M1": float(r["M1"]),
                "unreached": 1.0 - float(r["reached"]),
                "M2": float(r["M2"]), "M4": float(r["M4"]), "M5": float(r["M5"]),
            }
    return rows


def parse_conditions(text):
    d = {}
    for line in text.splitlines():
        if ": " in line:
            k, v = line.split(": ", 1)
            d[k.strip()] = v.strip()
    return d


def check_meta(before_text, after_text):
    b, a = parse_conditions(before_text), parse_conditions(after_text)
    for name, c in (("before", b), ("after", a)):
        if c.get("ped_model") != "ttc":
            fail(f"{name}: conditions.txt に 'ped_model: ttc' が無い(DIR は .../ttc を指す)")
    for name, c in (("before", b), ("after", a)):
        for k in ("git_sha", "docker_image_id", "arch"):
            if not c.get(k) or c.get(k) == "unknown":
                fail(f"{name}: conditions.txt の {k} が無い、空、または unknown")
    for k in ("docker_image_id", "arch"):
        if b.get(k) != a.get(k):
            fail(f"{k} が違う: before={b.get(k)} after={a.get(k)}")
    if b["git_sha"].removesuffix("-dirty") == a["git_sha"].removesuffix("-dirty"):
        fail(f"git_sha が同じ({b.get('git_sha')}): 変更前後が同じコミット")


def validate_rows(rows, name):
    """壊れた値(NaN、inf、範囲外、非整数)は検査エラー。M1 系は非負の整数、未到達(1 - reached)は 0 か 1。"""
    for key, v in rows.items():
        for m in METRICS:
            x = v[m]
            if not math.isfinite(x):
                fail(f"{name}: {key} の {m} が有限でない({x})")
        for m in ("M1_moving", "M1_stopped", "M1"):
            if v[m] < 0 or v[m] != int(v[m]):
                fail(f"{name}: {key} の {m} が非負の整数でない({v[m]})")
        if v["unreached"] not in (0.0, 1.0):
            fail(f"{name}: {key} の reached が 0 か 1 でない(未到達 {v['unreached']})")


def check_keys(before, after):
    for name, rows in (("before", before), ("after", after)):
        validate_rows(rows, name)
        scen = {s for s, _ in rows}
        if scen != set(RULE_SCENARIOS):
            fail(f"{name}: シナリオが規則の 6 シナリオと違う(あるもの {sorted(scen)}。rear などは対象外)")
        bad = sorted({seed for _, seed in rows if JUDGE_LO <= seed <= JUDGE_HI})
        if bad:
            fail(f"{name}: 判定範囲 {JUDGE_LO}–{JUDGE_HI} と重なるシードがある({bad[:3]}...)。この issue では使わない")
    if set(before) != set(after):
        fail("(シナリオ, シード) の集合が変更前後で違う")
    scen = {}
    for s, seed in before:
        scen.setdefault(s, set()).add(seed)
    if len({frozenset(v) for v in scen.values()}) != 1:
        fail("シナリオごとのシード集合が違う")
    return sorted(scen), sorted(next(iter(scen.values())))


def mean_se(d):
    n = len(d)
    if n < 2:
        fail("n < 2")
    m = sum(d) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in d) / (n - 1))
    return m, sd / math.sqrt(n)


def diffs(before, after, scenarios, metric):
    return [after[k][metric] - before[k][metric] for k in sorted(before) if k[0] in scenarios]


def worse_up(st):
    return st[0] > 0 and st[0] >= 2 * st[1]


def evaluate(before, after):
    """(結論, 詳細 dict)。結論 = 'ok' | 'review' | 'ref'。"""
    scenarios, seeds = check_keys(before, after)
    n_seeds = len(seeds)
    f1 = mean_se(diffs(before, after, scenarios, "M1"))
    f2 = mean_se(diffs(before, after, scenarios, "unreached"))
    flags = []
    if worse_up(f1):
        flags.append("F1")
    if worse_up(f2):
        flags.append("F2")
    notes = []
    for s in ATTN_SCENARIOS:
        if s not in scenarios:
            continue
        m2 = mean_se(diffs(before, after, [s], "M2"))
        m5 = mean_se(diffs(before, after, [s], "M5"))
        if m2[0] < 0 and -m2[0] >= 2 * m2[1]:
            notes.append(f"{s}: M2 が 2 SE 以上下がった({m2[0]:+.4f}, SE {m2[1]:.4f})")
        if worse_up(m5):
            notes.append(f"{s}: M5 が 2 SE 以上増えた({m5[0]:+.4f}, SE {m5[1]:.4f})")
    m1_before = sum(v["M1"] for v in before.values())
    m1_after = sum(v["M1"] for v in after.values())
    low_power = m1_before <= 5
    if low_power and m1_after > m1_before:
        notes.append(f"M1 合計が増えた({m1_before:g} → {m1_after:g})。件数が少ないので注意に格上げ")
    if n_seeds < MIN_SEEDS:
        verdict = "ref"
    elif flags:
        verdict = "review"
    else:
        verdict = "ok"
    return verdict, dict(scenarios=scenarios, seeds=seeds, f1=f1, f2=f2, flags=flags, notes=notes,
                         m1_before=m1_before, m1_after=m1_after, low_power=low_power)


def report(before, after, info, verdict):
    scenarios = info["scenarios"]
    print(f"seeds {info['seeds'][0]}-{info['seeds'][-1]}, n={len(info['seeds'])} per scenario, scenarios={len(scenarios)}")
    print(f"{'scenario':<18}{'metric':<11}{'before':>9}{'after':>9}{'d mean':>10}{'SE':>8}")
    for s in scenarios + ["ALL"]:
        sc = scenarios if s == "ALL" else [s]
        for m in METRICS:
            keys = [k for k in sorted(before) if k[0] in sc]
            bm = sum(before[k][m] for k in keys) / len(keys)
            am = sum(after[k][m] for k in keys) / len(keys)
            dm, se = mean_se(diffs(before, after, sc, m))
            print(f"{s:<18}{m:<11}{bm:>9.4f}{am:>9.4f}{dm:>+10.4f}{se:>8.4f}")
    for name, st in (("F1 M1 合計", info["f1"]), ("F2 未到達", info["f2"])):
        print(f"{name}: d mean {st[0]:+.5f} SE {st[1]:.5f} -> {'当たり' if worse_up(st) else 'なし'}")
    print(f"M1 合計: 変更前 {info['m1_before']:g} 変更後 {info['m1_after']:g}")
    if info["low_power"]:
        print("検出力の注記: F1 の検出力が低い(増加がおよそ +4 件以上でないと当たらない)")
    for n in info["notes"]:
        print(f"注意: {n}")
    label = {"ok": "悪化なし", "review": "要確認(" + ",".join(info["flags"]) + ")。自己マージと判定 21–120 を止めてユーザーに聞く",
             "ref": f"参考(1 シナリオ {MIN_SEEDS} 本未満のため判定しない。要確認の当たり: {','.join(info['flags']) or 'なし'})"}
    print(f"結論: {label[verdict]}")


def run(before_dir, after_dir, rng, config):
    texts = []
    for d in (before_dir, after_dir):
        with open(os.path.join(d, "conditions.txt")) as f:
            texts.append(f.read())
    check_meta(*texts)
    before = load_csv(os.path.join(before_dir, f"{config}_{rng}.csv"))
    after = load_csv(os.path.join(after_dir, f"{config}_{rng}.csv"))
    verdict, info = evaluate(before, after)
    report(before, after, info, verdict)
    return verdict


def self_test():
    scens = list(RULE_SCENARIOS)
    SEED0 = 1420   # 1421 以降(21–120 と重ならない)

    def mk(n, m1=lambda s, k: 0, unreached=lambda s, k: 0, m2=lambda s, k: 1.0, m5=lambda s, k: 0.5):
        d = {}
        for s in scens:
            for k in range(1, n + 1):
                a = m1(s, k)
                d[(s, SEED0 + k)] = {"M1_moving": float(a), "M1_stopped": 0.0, "M1": float(a), "unreached": float(unreached(s, k)),
                             "M2": m2(s, k), "M4": 0.1, "M5": m5(s, k)}
        return d

    base = mk(100, m1=lambda s, k: 1 if (s == "crowd" and k <= 2) else 0)
    assert evaluate(base, mk(100, m1=lambda s, k: 1 if (s == "crowd" and k <= 2) else 0))[0] == "ok"   # 全 d = 0

    def plus(n_seeds):
        return mk(100, m1=lambda s, k: (1 if (s == "crowd" and k <= 2) else 0) + (1 if (s == "crossing" and k <= n_seeds) else 0))
    assert evaluate(base, plus(1))[0] == "ok"        # 1 シードだけ +1
    assert evaluate(base, plus(3))[0] == "ok"        # 3 本は 2 SE に届かない
    v, info = evaluate(base, plus(4))
    assert v == "review" and info["flags"] == ["F1"], (v, info["flags"])   # 4 シードで +1
    assert info["low_power"] and info["notes"]                                # 件数が少ない注記と注意の格上げ
    # M1a -1 かつ M1b +1 は合計が同じ -> 悪化なし
    mv = mk(100); mv2 = mk(100)
    for k in range(1, 101):
        mv[("crossing", SEED0 + k)].update(M1_moving=1.0, M1=1.0)
        mv2[("crossing", SEED0 + k)].update(M1_stopped=1.0, M1=1.0)
    assert evaluate(mv, mv2)[0] == "ok"
    # 未到達の増加 -> 要確認(F2)
    v, info = evaluate(base, mk(100, m1=lambda s, k: 1 if (s == "crowd" and k <= 2) else 0,
                                unreached=lambda s, k: 1 if (s == "static_group" and k <= 10) else 0))
    assert v == "review" and info["flags"] == ["F2"]
    # M2 低下 / M5 増加は注意だけ(止めない)
    v, info = evaluate(base, mk(100, m1=lambda s, k: 1 if (s == "crowd" and k <= 2) else 0,
                                m2=lambda s, k: 0.5 if s == "crowd" and k % 2 else 1.0,
                                m5=lambda s, k: 0.5 + (0.2 if s == "corridor_oncoming" and k % 2 else 0)))
    assert v == "ok" and len(info["notes"]) == 2, info["notes"]
    # シード不一致 -> エラー
    try:
        evaluate(base, mk(99)); raise AssertionError("seed mismatch")
    except CheckError:
        pass
    # 100 本未満 -> 参考
    small = mk(50)
    assert evaluate(small, mk(50, m1=lambda s, k: 1))[0] == "ref"
    # 規則の 6 シナリオがそろわない入力は検査エラー(1 シナリオだけ、rear、余分なシナリオ)
    def expect_error(b, a):
        try:
            evaluate(b, a); raise AssertionError("expected CheckError")
        except CheckError:
            pass
    one = {k: v for k, v in base.items() if k[0] == "crowd"}
    expect_error(one, dict(one))
    rear = {(("rear_" + str(i)), k): dict(v) for i in range(3) for (s, k), v in base.items() if s == "crowd"}
    expect_error(rear, dict(rear))
    extra = dict(base); extra[("rear_0", 1)] = dict(base[("crowd", SEED0 + 1)])
    expect_error(extra, dict(extra))
    # 21–120 と重なるシードは検査エラー(判定範囲を使わない)
    def shift(d, off):
        return {(s, k + off): dict(v) for (s, k), v in d.items()}
    expect_error(shift(base, 20 - SEED0), shift(base, 20 - SEED0))        # 21–120
    expect_error(shift(base, 100 - SEED0), shift(base, 100 - SEED0))      # 101–200(重なる)
    v, _ = evaluate(base, dict(base)); assert v == "ok"   # 1421–1520 は使える
    # 壊れた値(NaN、inf、範囲外、非整数)は検査エラー(「悪化なし」にしない)
    def corrupt(metric, val):
        a = mk(100)
        a[("crowd", SEED0 + 1)][metric] = val
        return a
    for metric, val in (("M1", float("nan")), ("M1", float("inf")), ("M1", -1.0), ("M1", 0.5), ("M1_moving", 0.5),
                        ("unreached", float("nan")), ("unreached", -4.0), ("unreached", 0.5),
                        ("M2", float("nan")), ("M5", float("inf")), ("M4", float("nan"))):
        expect_error(mk(100), corrupt(metric, val))
        expect_error(corrupt(metric, val), mk(100))
    # 全 d が同じ正の値(SE = 0)は要確認。M2 の上昇と M5 の低下は注意にしない
    assert evaluate(mk(100), mk(100, m1=lambda s, k: 1))[0] == "review"
    v, info = evaluate(mk(100), mk(100, m2=lambda s, k: 1.5, m5=lambda s, k: 0.1))
    assert v == "ok" and not info["notes"]
    # -dirty だけが違う git_sha は同じコミット
    try:
        check_meta("git_sha: abc\ndocker_image_id: X\narch: a\nped_model: ttc\n",
                   "git_sha: abc-dirty\ndocker_image_id: X\narch: a\nped_model: ttc\n"); raise AssertionError("dirty")
    except CheckError:
        pass
    # conditions の欠損・空・unknown は検査エラー(git_sha、docker_image_id、arch)
    for k in ("git_sha", "docker_image_id", "arch"):
        for bad in ("", "unknown", None):
            def txt(sha):
                kv = {"git_sha": sha, "docker_image_id": "X", "arch": "a"}
                if bad is None:
                    kv.pop(k)
                else:
                    kv[k] = bad
                return "".join(f"{kk}: {vv}\n" for kk, vv in kv.items()) + "ped_model: ttc\n"
            for pair in ((txt("a"), txt("b")), (txt("b"), txt("a"))):
                try:
                    check_meta(*pair); raise AssertionError(f"meta {k} {bad!r}")
                except CheckError:
                    pass
    # conditions の検査
    good_b = "git_sha: a\ndocker_image_id: X\narch: arm64\nped_model: ttc\n"
    good_a = "git_sha: b\ndocker_image_id: X\narch: arm64\nped_model: ttc\n"
    check_meta(good_b, good_a)
    for bad_b, bad_a in ((good_b.replace("ped_model: ttc\n", ""), good_a),
                         (good_b, good_a.replace("X", "Y")),
                         (good_b, good_a.replace("arm64", "x86_64")),
                         (good_b, good_b)):
        try:
            check_meta(bad_b, bad_a); raise AssertionError("meta")
        except CheckError:
            pass
    print("self-test OK")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--before")
    p.add_argument("--after")
    p.add_argument("--range", default="select")
    p.add_argument("--config", default="social")
    p.add_argument("--self-test", action="store_true")
    a = p.parse_args()
    try:
        if a.self_test:
            self_test()
            return 0
        if not (a.before and a.after):
            fail("--before と --after が必要")
        return 1 if run(a.before, a.after, a.range, a.config) == "review" else 0
    except (CheckError, OSError, KeyError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
