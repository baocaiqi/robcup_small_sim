# -*- coding: utf-8 -*-
"""param_sensitivity.py — 灵敏度筛查：75 个旋钮里，哪些是"死"的？

为什么需要：轮次 74 的归因发现 3 个参数改了之后仿真指标**一模一样**
（kCoverLineTta / kInterceptLineDist / kReboundRushDist）——说明它们不在
当前会走到的代码路径上（或变化量跨不过任何阈值）。搜索一个死旋钮 = 白烧算力。

做法：每个参数单独设成"很响"的值（默认值 ×1.6 或 +40%，夹到安全范围），
跑同一批种子，与基线比；四项指标(net/shots/gaf/fb)全部逐位相同 ⇒ 判为"死"。

用法：python tools\\py\\param_sensitivity.py --workers 3 --out docs/work/param_sensitivity.json
"""
import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tune_es as T   # 复用参数读取、任务池、评估函数


def loud_value(name, v):
    """给一个"很响"的扰动量：±60%，夹进 RANGES（若有），并保证不越界。"""
    if name in T.RANGES:
        lo, hi = T.RANGES[name]
        # 取离默认值较远的那一端
        return hi if (hi - v) > (v - lo) else lo
    return v * 1.6 if v > 0 else v * 1.6 if v < 0 else 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--games", type=int, default=20)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--opp", nargs="*", default=["scripted1.0"])
    ap.add_argument("--baseline", default=os.path.join("docs", "work", "tune_base_scripted20.json"))
    ap.add_argument("--out", default=os.path.join("docs", "work", "param_sensitivity.json"))
    ap.add_argument("--only-group", default=None, help="只查某一组（attack/defense/roles/motion）")
    a = ap.parse_args()

    base_vals, ints = T.load_baseline_params()
    names = sorted(base_vals)
    if a.only_group:
        names = [n for n in names if n.startswith(a.only_group + ".")]
    with open(a.baseline, encoding="utf-8") as f:
        base = json.load(f)
    tasks = T.make_tasks(a.games, a.seeds, a.opp)
    if not tasks:
        print("✗ 任务池为空"); return 2
    print(f"=== 灵敏度筛查：{len(names)} 个参数，{len(tasks)} 个任务 × {a.games} 局，{a.workers} 并发 ===")
    print(f"    基线: 净胜球 {base['net']:.3f} 被射门 {base['shots']:.1f} "
          f"门区 {base['gaf']:.2f} 争球 {base['fb']:.1f}")

    os.makedirs(os.path.join("build", "sens"), exist_ok=True)
    live, dead = [], []
    t0 = time.time()
    for k, name in enumerate(names, 1):
        x = [base_vals[n] for n in names]
        v = loud_value(name, base_vals[name])
        x[k - 1] = v
        agg = T.eval_vector(x, names, tasks, ints, a.workers, os.path.join("build", "sens"), name.replace(".", "_"))
        if agg is None:
            print(f"  [{k}/{len(names)}] {name}: 评估失败（跳过）"); continue
        same = all(abs(agg[m] - base[m]) < 1e-9 for m in ("net", "shots", "gaf", "fb"))
        d = {m: agg[m] - base[m] for m in ("net", "shots", "gaf", "fb")}
        rec = {"name": name, "default": base_vals[name], "loud": v, "delta": d, "dead": same}
        (dead if same else live).append(rec)
        flag = "死旋钮" if same else "有效"
        print(f"  [{k:2d}/{len(names)}] {name:34s} → {v:<10.4g} Δnet={d['net']:+.2f} "
              f"Δshots={d['shots']:+.0f} Δgaf={d['gaf']:+.2f} Δfb={d['fb']:+.1f}  {flag}"
              f"  [{time.time()-t0:.0f}s]")
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump({"baseline": base, "live": live, "dead": dead}, f, ensure_ascii=False, indent=1)

    print(f"\n=== 结论：有效 {len(live)} 个，死旋钮 {len(dead)} 个 ===")
    if dead:
        print("死旋钮（下轮搜索请排除）:")
        for r in dead:
            print("   ", r["name"], "默认", r["default"])
    print(f"→ {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
