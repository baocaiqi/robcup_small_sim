# -*- coding: utf-8 -*-
"""accept_params.py — 按规格 §5 的验收标准，机器判定一组参数能不能合并。

为什么要有这个脚本：搜索出的参数在**训练集**上一定好看（那是它被选中的原因），
真正的问题是"换个种子还灵不灵"。人工看几个数字很容易自欺欺人，
所以把验收标准写成代码，判定结果只有 PASS / FAIL。

验收标准（docs/work/RL参数搜索规格.md §5）：
  1. **两套留出集**（A=1001-1003, B=2001-2003）净胜球都 ≥ 基线 + 0.3/场
  2. 被射门数 ≤ 基线 × 1.10（防守不得明显变差）
  3. 门区纪律（我方门区挤 2 人以上帧数）≤ 基线（这是被判点球的根因，不许变差）
  4. 争球重置 ≤ 基线 × 1.05（不许变得更爱卡球）
每套种子集都用**同一批任务**（4 种对手 × 3 种子 × 20 局）做配对比较。

用法：
    python tools\\py\\accept_params.py docs/work/best_attack.txt docs/work/best_combined.txt
    python tools\\py\\accept_params.py best.txt --group both --games 20
"""
import argparse
import json
import os
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tune_es as T

SETS = [
    ("训练集", [1, 2, 3], os.path.join("docs", "work", "tune_base_train20.json")),
    ("留出A", [1001, 1002, 1003], os.path.join("docs", "work", "tune_base_A20.json")),
    ("留出B", [2001, 2002, 2003], os.path.join("docs", "work", "tune_base_B20.json")),
]
MIN_DNET = 0.30      # 留出集净胜球至少涨这么多
MAX_SHOTS = 1.10     # 被射门最多涨 10%
MAX_FB = 1.05        # 争球重置最多涨 5%


def eval_on(param_file, names, seeds, baseline_json, games, ints, workers, tmpdir):
    with open(baseline_json, encoding="utf-8") as f:
        base = json.load(f)
    tasks = T.make_tasks(games, seeds)
    x = [T.load_baseline_params()[0][n] for n in names]
    if param_file:
        with open(param_file, encoding="utf-8") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line:
                    continue
                p = line.split()
                if len(p) == 2 and p[0] in names:
                    x[names.index(p[0])] = float(p[1])
    tag = os.path.basename(param_file or "base").replace(".txt", "")
    agg = T.eval_vector(x, names, tasks, ints, workers, tmpdir, tag)
    return base, agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--group", default="both", choices=sorted(T.GROUPS))
    ap.add_argument("--games", type=int, default=20)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tmpdir", default=os.path.join("build", "accept"))
    a = ap.parse_args()
    os.makedirs(a.tmpdir, exist_ok=True)
    names = T.GROUPS[a.group]
    base_vals, ints = T.load_baseline_params()

    print(f"=== 验收：group={a.group}（{len(names)} 个参数），每组 {a.games} 局/任务 × 12 任务 ===")
    all_pass = True
    for pf in a.files:
        print(f"\n### {pf}")
        print(f"{'种子集':>6} {'净胜球':>8} {'基线':>7} {'Δ':>7} {'被射门':>8} {'相对':>7} "
              f"{'门区纪律':>8} {'争球':>7} {'相对':>7}  判定")
        file_ok = True
        holdout_ok = True
        for label, seeds, bj in SETS:
            base, agg = eval_on(pf, names, seeds, bj, a.games, ints, a.workers, a.tmpdir)
            d = agg["net"] - base["net"]
            sr = agg["shots"] / max(base["shots"], 1e-9)
            fr = agg["fb"] / max(base["fb"], 1e-9)
            ok = (sr <= MAX_SHOTS) and (agg["gaf"] <= base["gaf"] + 1e-9) and (fr <= MAX_FB)
            if label.startswith("留出"):
                ok = ok and (d >= MIN_DNET)
                holdout_ok = holdout_ok and (d >= MIN_DNET)
            file_ok = file_ok and ok
            print(f"{label:>6} {agg['net']:>8.3f} {base['net']:>7.3f} {d:>+7.3f} "
                  f"{agg['shots']:>8.1f} {sr*100-100:>+6.1f}% {agg['gaf']:>8.2f} "
                  f"{agg['fb']:>7.1f} {fr*100-100:>+6.1f}%  {'✅' if ok else '❌'}")
        print(f"  → {'✅ PASS（可合并）' if file_ok else '❌ FAIL'}"
              f"{'' if holdout_ok else '（留出集未达标：很可能是过拟合训练种子）'}")
        all_pass = all_pass and file_ok
    print(f"\n=== 总结：{'全部 PASS' if all_pass else '有 FAIL，不合并'} ===")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
