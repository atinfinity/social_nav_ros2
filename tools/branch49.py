#!/usr/bin/env python3
"""接触直前の止まり方の確認 0: R1 / R2 の分岐 p を contacts CSV から数える(規則は docs/sim_results.md「接触直前の止まり方: 規則の固定」)。

  branch49.py A_contacts.csv B_contacts.csv ...   # social の contacts CSV(321-420 と 521-620)
  branch49.py --self-test

分母 N = corridor_oncoming + crowd、cls in {0,1}、d1_valid == 1 の接触。
分子 = そのうち d1_nfree == 0。p >= 1/2 -> R1、p < 1/2 -> R2。
d1_valid == 0 は除外して件数を書く。除外が M1a(cls in {0,1} の接触)の 10% を超えたら分岐を止める。
併記(分岐に使わない): シナリオ別・範囲別の p、d1_zero_free、d1_chosen_contact。
"""
import argparse
import csv
import os
import sys
import tempfile

SCENARIOS = ("corridor_oncoming", "crowd")
RANGES = (("321-420", 321, 420), ("521-620", 521, 620))


def range_of(seed):
    for name, a, b in RANGES:
        if a <= seed <= b:
            return name
    return "other"


def load(paths):
    rows = []
    for path in paths:
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    return rows


def count(rows):
    """M1a の接触を数える。戻り値: dict(m1a, excluded, n, k, zf, cc, by_scn, by_rng)。"""
    def new():
        return {"n": 0, "k": 0, "zf": 0, "cc": 0}
    tot = new()
    tot["m1a"] = 0
    tot["excluded"] = 0
    by_scn = {s: new() for s in SCENARIOS}
    by_rng = {r[0]: new() for r in RANGES}
    by_rng["other"] = new()
    for r in rows:
        if r["scenario"] not in SCENARIOS or int(r["cls"]) not in (0, 1):
            continue
        tot["m1a"] += 1
        if int(r["d1_valid"]) != 1:
            tot["excluded"] += 1
            continue
        k = int(int(r["d1_nfree"]) == 0)
        zf = int(r["d1_zero_free"])
        cc = int(r["d1_chosen_contact"])
        for d in (tot, by_scn[r["scenario"]], by_rng[range_of(int(r["seed"]))]):
            d["n"] += 1
            d["k"] += k
            d["zf"] += zf
            d["cc"] += cc
    return {"tot": tot, "by_scn": by_scn, "by_rng": by_rng}


def frac(k, n):
    return k / n if n else float("nan")


def branch(res):
    t = res["tot"]
    if t["m1a"] and t["excluded"] > 0.1 * t["m1a"]:
        return "STOP"
    if t["n"] == 0:
        return "STOP"
    return "R1" if 2 * t["k"] >= t["n"] else "R2"


def fmt(name, d):
    return (f"  {name}: N={d['n']} nfree0={d['k']} p={frac(d['k'], d['n']):.4f} "
            f"zero_free={d['zf']} ({frac(d['zf'], d['n']):.4f}) "
            f"chosen_contact={d['cc']} ({frac(d['cc'], d['n']):.4f})")


def report(res):
    t = res["tot"]
    print(f"M1a(corridor_oncoming+crowd, cls 0/1)={t['m1a']}  d1_valid==0 で除外={t['excluded']} "
          f"({frac(t['excluded'], t['m1a']):.4f})")
    print(fmt("合計", t))
    for s, d in res["by_scn"].items():
        print(fmt(s, d))
    for r, d in res["by_rng"].items():
        if d["n"] or r != "other":
            print(fmt(r, d))
    b = branch(res)
    if b == "STOP":
        print("分岐: 停止(除外が M1a の 10% 超、または N=0)。相談役に聞く")
    else:
        print(f"分岐: p={frac(t['k'], t['n']):.4f} -> {b}")
    return b


def self_test():
    hdr = "scenario,seed,cls,d1_valid,d1_nfree,d1_zero_free,d1_chosen_contact"
    lines = [
        "corridor_oncoming,321,0,1,0,1,1",   # 分子
        "corridor_oncoming,322,1,1,0,0,1",   # 分子
        "corridor_oncoming,323,0,1,3,1,0",
        "crowd,521,1,1,0,1,1",               # 分子
        "crowd,522,0,1,2,1,0",
        "crowd,523,0,0,0,1,1",               # 除外(valid 0)
        "crowd,524,2,1,0,1,1",               # cls 2: 数えない
        "overtake,325,0,1,0,1,1",            # 対象外シナリオ
    ]
    with tempfile.TemporaryDirectory() as d:
        pa, pb = os.path.join(d, "a.csv"), os.path.join(d, "b.csv")
        with open(pa, "w") as f:
            f.write("\n".join([hdr] + lines[:4]) + "\n")
        with open(pb, "w") as f:
            f.write("\n".join([hdr] + lines[4:]) + "\n")
        res = count(load([pa, pb]))
    t = res["tot"]
    # M1a = 6(cls 0/1、対象シナリオ)、除外 1、N = 5、分子 = 3
    assert (t["m1a"], t["excluded"], t["n"], t["k"]) == (6, 1, 5, 3), t
    assert (t["zf"], t["cc"]) == (4, 3), t
    assert (res["by_scn"]["corridor_oncoming"]["n"], res["by_scn"]["corridor_oncoming"]["k"]) == (3, 2)
    assert (res["by_scn"]["crowd"]["n"], res["by_scn"]["crowd"]["k"]) == (2, 1)
    assert (res["by_rng"]["321-420"]["n"], res["by_rng"]["321-420"]["k"]) == (3, 2)
    assert (res["by_rng"]["521-620"]["n"], res["by_rng"]["521-620"]["k"]) == (2, 1)
    # 除外 1/6 > 10% -> 停止
    assert branch(res) == "STOP"
    # 除外の割合を下げる(除外 1 / M1a 20 = 5%)と p = 3/5 >= 1/2 -> R1
    r2 = {"tot": dict(t, m1a=20), "by_scn": res["by_scn"], "by_rng": res["by_rng"]}
    assert branch(r2) == "R1"
    # 境界: p = 1/2 ちょうどは R1、それ未満は R2
    assert branch({"tot": {"m1a": 4, "excluded": 0, "n": 4, "k": 2}}) == "R1"
    assert branch({"tot": {"m1a": 5, "excluded": 0, "n": 5, "k": 2}}) == "R2"
    print("branch49 self-test: OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        self_test()
        return 0
    if not a.csv:
        ap.error("contacts CSV のパスが要る")
    report(count(load(a.csv)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
