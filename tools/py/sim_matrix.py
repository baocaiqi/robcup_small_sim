# -*- coding: utf-8 -*-
"""sim_matrix.py — 对手矩阵评估（docs/19 §4.1）

问题：我们所有 A/B 都只用 `--opp scripted --strength 2.0` 一个档位 → 过拟合风险。
本工具把"改动评估"变成**跨对手矩阵**：任何我方改动必须在多风格对手上都不劣化。

矩阵（每格跑 N 场 × 每种子）：
    scripted×1.0（松散）  scripted×2.0（现基线）  scripted×3.0（高压）
    wall（摆大巴/门线堆人墙）  self（自我博弈/镜像）

用法：
    # 基线（改动前的 exe，例如从 HEAD 编的那个）
    python tools/py/sim_matrix.py run  --exe build/verify/sim_base.exe --tag base
    # 候选（当前工作区编的）
    python tools/py/sim_matrix.py run  --exe build/verify/sim_bench.exe --tag cand
    # 对比（按 docs/19 判据：每格 Δ≥-0.5；矩阵均值不劣化；单格 <-1.0 必须解释）
    python tools/py/sim_matrix.py cmp  --base logs/matrix_base.json --cand logs/matrix_cand.json

产物：logs/matrix_<tag>.json（每格明细，可复现）
判据（cmp 会直接打印 PASS/FAIL，不替人下结论，但把超限格标出来）。
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
LOGS = os.path.join(ROOT, "logs")

# (档位名, --opp 值, --strength 值)
CELLS = [
    ("scripted1.0", "scripted", 1.0),
    ("scripted2.0", "scripted", 2.0),
    ("scripted3.0", "scripted", 3.0),
    ("wall",        "wall",     2.0),   # wall 不看 strength
    ("self",        "self",     2.0),   # self 不看 strength
]

SUM_RE = re.compile(
    r"汇总:\s*我们\s*(\d+)\s*:\s*(\d+)\s*对手\s*\(均\s*([\d.]+)\s*:\s*([\d.]+)\)\s*"
    r"平均控球\s*([\d.]+)%\s*平均射门\s*([\d.]+)")
DISC_RE = re.compile(
    r"门区2\+人\s*均\s*([\d.]+)\s*帧/场.*?单人停留>20帧\s*均\s*([\d.]+)\s*帧/场.*?"
    r"争球重置\s*均\s*([\d.]+)\s*次/场.*?角区救球触发\s*([\d.]+)\s*次/场", re.S)


def run_cell(exe, opp, strength, seeds, games, frames):
    """跑一格的每个种子，返回该格汇总（均值）"""
    per = []
    for sd in seeds:
        cmd = [exe, "--games", str(games), "--frames", str(frames),
               "--seed", str(sd), "--strength", str(strength)]
        if opp != "scripted":
            cmd += ["--opp", opp]
        p = subprocess.run(cmd, capture_output=True, cwd=ROOT)
        out = (p.stdout or b"").decode("utf-8", "replace")
        m, d = SUM_RE.search(out), DISC_RE.search(out)
        if not m:
            print(f"  [!] {opp} s={sd}: 没解析到汇总，前 200 字:\n{out[:200]}")
            continue
        cell = {
            "seed": sd,
            "us": float(m.group(3)), "them": float(m.group(4)),
            "net": float(m.group(3)) - float(m.group(4)),
            "poss": float(m.group(5)), "shots": float(m.group(6)),
            "ga_frames": float(d.group(1)) if d else None,
            "ga_solo": float(d.group(2)) if d else None,
            "fb": float(d.group(3)) if d else None,
            "rescue": float(d.group(4)) if d else None,
        }
        per.append(cell)
        print(f"  {opp:<9} s={sd}: 净胜 {cell['net']:+.1f}  控球 {cell['poss']:.1f}%  "
              f"射门 {cell['shots']:.0f}  门区2+人 {cell['ga_frames']} 帧  争球 {cell['fb']}")
    if not per:
        return None
    keys = ["net", "poss", "shots", "ga_frames", "ga_solo", "fb", "rescue"]
    agg = {k: sum(c[k] for c in per if c[k] is not None) / max(1, sum(1 for c in per if c[k] is not None))
           for k in keys}
    agg["n"] = len(per)
    return agg


def cmd_run(a):
    os.makedirs(LOGS, exist_ok=True)
    seeds = [int(x) for x in a.seeds.split(",")]
    res = {"exe": a.exe, "games": a.games, "frames": a.frames, "seeds": seeds, "cells": {}}
    print(f"=== 对手矩阵: {a.exe}（每格 {a.games} 场 × 种子 {seeds}）===")
    for name, opp, st in CELLS:
        print(f"[{name}]")
        agg = run_cell(a.exe, opp, st, seeds, a.games, a.frames)
        if agg:
            res["cells"][name] = agg
    out = os.path.join(LOGS, f"matrix_{a.tag}.json")
    with open(out, "w", encoding="utf-8") as fp:
        json.dump(res, fp, ensure_ascii=False, indent=2)
    print(f"\n已保存 {os.path.relpath(out, ROOT)}")
    if res["cells"]:
        mean = sum(c["net"] for c in res["cells"].values()) / len(res["cells"])
        print(f"矩阵平均净胜: {mean:+.2f}")
    return 0


def cmd_cmp(a):
    B = json.load(open(a.base, encoding="utf-8"))
    C = json.load(open(a.cand, encoding="utf-8"))
    print(f"=== 矩阵对比  base={os.path.basename(a.base)}  cand={os.path.basename(a.cand)} ===")
    print(f"{'档位':<12}{'base净胜':>9}{'cand净胜':>9}{'Δ':>7}   {'控球Δ':>7}{'射门Δ':>7}{'争球Δ':>7}")
    bad, warn, dsum = [], [], []
    for name, _, _ in CELLS:
        b, c = B["cells"].get(name), C["cells"].get(name)
        if not b or not c:
            print(f"{name:<12}  （缺数据，跳过）")
            continue
        d = c["net"] - b["net"]
        dsum.append(d)
        dp, ds, df = c["poss"] - b["poss"], c["shots"] - b["shots"], c["fb"] - b["fb"]
        flag = ""
        if d < -1.0:
            flag = "  ← 超限(>1.0)，必须解释"
            bad.append(name)
        elif d < -0.5:
            flag = "  ← 超出噪声"
            warn.append(name)
        print(f"{name:<12}{b['net']:>+9.2f}{c['net']:>+9.2f}{d:>+7.2f}   "
              f"{dp:>+7.2f}{ds:>+7.0f}{df:>+7.2f}{flag}")
    if dsum:
        mean = sum(dsum) / len(dsum)
        print(f"\n矩阵平均 Δ: {mean:+.2f}（判据：各档 ≥ -0.5 且均值不劣化）")
        verdict = "PASS" if (not bad and mean > -0.3) else ("WARN" if not bad else "FAIL")
        print(f"结论: {verdict}"
              + (f"   超限档: {', '.join(bad)}" if bad else "")
              + (f"   超噪声档: {', '.join(warn)}" if warn else ""))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="跑一遍矩阵并存 JSON")
    r.add_argument("--exe", required=True)
    r.add_argument("--tag", required=True, help="输出 logs/matrix_<tag>.json")
    r.add_argument("--games", type=int, default=50)
    r.add_argument("--frames", type=int, default=24000)
    r.add_argument("--seeds", default="1,2")
    r.set_defaults(fn=cmd_run)
    c = sub.add_parser("cmp", help="对比两份矩阵 JSON")
    c.add_argument("--base", required=True)
    c.add_argument("--cand", required=True)
    c.set_defaults(fn=cmd_cmp)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
