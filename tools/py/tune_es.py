# -*- coding: utf-8 -*-
"""tune_es.py — 离线参数搜索（差分进化 DE），用来"用机器调进攻与防守的参数"。

见 docs/work/RL参数搜索规格.md。核心设计：

1. **配对比较降噪**：同一批种子下，候选参数与基线各跑一遍，比的是"差值"。
   仿真单局有随机性（开局微扰），只看绝对分会把噪声当成提升。
2. **多对手池**：脚本对手(1.0/1.4)、门线人墙、半场对称 —— 防止"只打得过一种脚本"。
3. **训练/留出种子分离**：搜索只用 --seeds，验收用 --holdout-seeds（搜索期间不可见）。
4. **适应度 = 净胜球提升 − 退化罚**：
        fitness = Δ净胜球 − 0.02×被射门恶化% − 0.02×争球重置恶化% − 0.1×门区纪律恶化(帧/场)
   （纯堆进攻不算赢：被射门变多、卡球变多、门区违规都会被扣回来）
5. 零第三方依赖（纯标准库；numpy 在这个环境里没装也不影响）。

用法：
    python tools\\py\\tune_es.py baseline --out docs/work/tune_baseline.json
    python tools\\py\\tune_es.py search  --group attack --gens 6 --pop 12 --workers 8
    python tools\\py\\tune_es.py eval    --params docs/work/best_params.txt --holdout
"""
import argparse
import json
import os
import random
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

sys.stdout.reconfigure(encoding="utf-8")

BENCH = os.path.join("build", "Release", "sim_bench.exe")
PARAM_DUMP = os.path.join("docs", "work", "params_基线.txt")
FIT_RE = re.compile(
    r"^FIT games=(\d+) net=(-?[\d.]+) gf=(-?[\d.]+) ga=(-?[\d.]+) "
    r"poss=(-?[\d.]+) shots=(-?[\d.]+) gaf=(-?[\d.]+) fb=(-?[\d.]+) sec=([\d.]+)")

# 对手池：(名字, 额外命令行, 权重)
OPPONENTS = [
    ("scripted1.0", "--opp scripted --strength 1.0", 2.0),
    ("scripted1.4", "--opp scripted --strength 1.4", 1.0),
    ("wall",        "--opp wall",                    1.0),
    ("keephalf",    "--opp yellow",                  1.0),
]

# 参数分组（--group）
GROUPS = {
    "attack": ["shoot.kMinOpen", "shoot.kAngleFull", "shoot.kWOpen", "shoot.kBankWOpen",
               "shoot.kBankMinQ", "shoot.kBankMargin", "shoot.kBankMaxDist",
               "shoot.kBankDirectWeak", "shoot.kBankPrepDist", "shoot.kBankPrepMargin",
               "roles.kPrepDist", "roles.kPrepPosTol", "roles.kPrepAngTol",
               "roles.kShootAlignTimeout", "roles.kDribAngTol", "roles.kMaxShootPushes",
               "pass.PASS_MAX_DIST", "pass.BLOCK_THRESHOLD", "pass.OFFSET_BASE",
               "pass.THREAT_RADIUS"],
    "defense": ["defense.kInterceptLineDist", "defense.kMinBallSpeed", "defense.kMySpeed",
                "defense.kReachMargin", "defense.kDoubleTeamDangerDist",
                "defense.kDoubleTeamLateral", "defense.kDoubleTeamCarryDist",
                "defense.kDoubleTeamCoverDist", "roles.kCoverLineDanger",
                "roles.kCoverLineTta", "roles.kCoverLineDist", "roles.kCoverLineGiveUp",
                "roles.kReboundRushSpeed", "roles.kReboundRushDist",
                "roles.kGkNoPushDist", "roles.kGkSideClear", "roles.kGkBackOff",
                "roles.kGkBehindMargin"],
    "motion": ["motion.kAlignedGain", "motion.kMaxRotW", "motion.kMinRotW",
               "motion.kNearDist", "motion.kCreepMax"],
    "pilot": ["shoot.kMinOpen", "shoot.kBankMinQ", "roles.kPrepDist",
              "roles.kCoverLineTta", "defense.kInterceptLineDist",
              "pass.PASS_MAX_DIST", "motion.kNearDist", "roles.kReboundRushDist"],
}
GROUPS["both"] = GROUPS["attack"] + GROUPS["defense"]   # 攻防合起来评估"组合拳"效果

# 手工给的范围（安全边界；没列到的按 ±25% 兜底）
RANGES = {
    "shoot.kMinOpen": (2.0, 30.0), "shoot.kAngleFull": (8.0, 45.0),
    "shoot.kWOpen": (0.05, 0.9), "shoot.kBankWOpen": (0.05, 0.9),
    "shoot.kBankMinQ": (0.2, 0.8), "shoot.kBankMargin": (0.0, 0.4),
    "shoot.kBankMaxDist": (60.0, 160.0), "shoot.kBankDirectWeak": (0.1, 0.7),
    "shoot.kBankPrepDist": (10.0, 70.0), "shoot.kBankPrepMargin": (0.0, 20.0),
    "roles.kPrepDist": (10.0, 70.0), "roles.kPrepPosTol": (2.0, 20.0),
    "roles.kPrepAngTol": (5.0, 60.0), "roles.kShootAlignTimeout": (10.0, 200.0),
    "roles.kDribAngTol": (5.0, 60.0), "roles.kMaxShootPushes": (1, 10),
    "pass.PASS_MAX_DIST": (30.0, 120.0), "pass.BLOCK_THRESHOLD": (5.0, 40.0),
    "pass.OFFSET_BASE": (0.0, 30.0), "pass.THREAT_RADIUS": (10.0, 70.0),
    "defense.kInterceptLineDist": (20.0, 110.0), "defense.kMinBallSpeed": (1.0, 10.0),
    "defense.kMySpeed": (1.0, 6.0), "defense.kReachMargin": (0.2, 5.0),
    "defense.kDoubleTeamDangerDist": (40.0, 160.0), "defense.kDoubleTeamLateral": (10.0, 80.0),
    "defense.kDoubleTeamCarryDist": (10.0, 80.0), "defense.kDoubleTeamCoverDist": (10.0, 80.0),
    "roles.kCoverLineDanger": (0.2, 4.0), "roles.kCoverLineTta": (8.0, 60.0),
    "roles.kCoverLineDist": (20.0, 140.0), "roles.kCoverLineGiveUp": (85.0, 215.0),
    "roles.kReboundRushSpeed": (0.5, 6.0), "roles.kReboundRushDist": (20.0, 150.0),
    "roles.kGkNoPushDist": (20.0, 100.0), "roles.kGkSideClear": (10.0, 70.0),
    "roles.kGkBackOff": (2.0, 40.0), "roles.kGkBehindMargin": (2.0, 30.0),
    "motion.kAlignedGain": (0.05, 1.0), "motion.kMaxRotW": (4.0, 25.0),
    "motion.kMinRotW": (0.5, 8.0), "motion.kNearDist": (4.0, 40.0),
    "motion.kCreepMax": (5.0, 60.0),
}

PEN_SHOts = 0.02    # 被射门恶化 1% → 扣 0.02 球
PEN_FB = 0.02       # 争球重置恶化 1% → 扣 0.02 球
PEN_GAF = 0.25      # 门区纪律恶化 1 帧/场 → 扣 0.25 球
# 系数选择史（都是实测，不是拍脑袋）：
#   0.10 → 搜出"多压上、后院挤人"的解：净胜球 +1.27（两独立种子集都复现）但门区纪律 2.6→6.6 帧/场；
#   0.35 + 硬判死(1.5×) → 把有效候选全毙了，留出集 Δnet 反而 ≈0（复制轮次 74 的教训）；
#   0.25 + 宽松硬限(3×) → 既不让纪律崩到 10 倍，也不误杀"用一点纪律换更多进球"的解。
GAF_HARD_MULT = 3.0


def load_baseline_params():
    """读 params_基线.txt → {名字: 默认值}，并标出哪些是整数参数。"""
    vals, ints = {}, set()
    with open(PARAM_DUMP, encoding="utf-8") as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 2:
                continue
            name, v = parts
            vals[name] = float(v)
            if "." not in v and "e" not in v.lower():
                ints.add(name)
    return vals, ints


def run_task(binary, extra, games, seed, params_path):
    cmd = [binary, "--games", str(games), "--seed", str(seed)] + extra.split()
    if params_path:
        cmd += ["--params", params_path]
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    for line in r.stdout.splitlines():
        m = FIT_RE.match(line.strip())
        if m:
            return {"net": float(m.group(2)), "gf": float(m.group(3)), "ga": float(m.group(4)),
                    "poss": float(m.group(5)), "shots": float(m.group(6)),
                    "gaf": float(m.group(7)), "fb": float(m.group(8))}
    return None


def make_tasks(games, seeds, opp_filter=None):
    tasks = []
    for name, extra, w in OPPONENTS:
        if opp_filter and name not in opp_filter:
            continue
        for s in seeds:
            tasks.append({"name": name, "extra": extra, "w": w, "games": games, "seed": s})
    return tasks


def eval_vector(x, names, tasks, ints, workers, tmpdir, tag=""):
    """把一个参数向量写成文件并跑完所有任务，返回加权指标 + 每任务结果。"""
    pfile = os.path.join(tmpdir, f"p_{tag or 'x'}.txt")
    with open(pfile, "w", encoding="ascii") as f:
        for n, v in zip(names, x):
            f.write("%s %.6g\n" % (n, round(v) if n in ints else v))
    with ThreadPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(lambda t: run_task(BENCH, t["extra"], t["games"], t["seed"], pfile), tasks))
    if any(r is None for r in res):
        return None
    tot_w = sum(t["w"] for t in tasks)
    agg = {k: sum(r[k] * t["w"] for r, t in zip(res, tasks)) / tot_w
           for k in ("net", "shots", "fb", "gaf", "poss", "gf", "ga")}
    agg["per_task"] = res
    return agg


def fitness(agg, base):
    """适应度：净胜球提升为正值，被射门/卡球/门区纪律退化扣分。

    硬约束：门区纪律（我方禁区里挤 2 人以上的帧数）超过基线 1.5 倍 → 直接判死。
    理由是这指标直接对应"被判点球"，不能用进球去换。
    """
    d_net = agg["net"] - base["net"]
    shots_w = max(0.0, (agg["shots"] - base["shots"]) / max(base["shots"], 1e-9) * 100.0)
    fb_w = max(0.0, (agg["fb"] - base["fb"]) / max(base["fb"], 1e-9) * 100.0)
    gaf_w = max(0.0, agg["gaf"] - base["gaf"])
    if agg["gaf"] > base["gaf"] * GAF_HARD_MULT:
        return -9e9          # 硬约束：纪律崩了，不管进多少球都不要
    return d_net - PEN_SHOts * shots_w - PEN_FB * fb_w - PEN_GAF * gaf_w


def cmd_baseline(args, base_vals, ints):
    tasks = make_tasks(args.games, args.seeds, args.opp)
    t0 = time.time()
    names = [n for n in GROUPS[args.group]]
    x0 = [base_vals[n] for n in names]
    agg = eval_vector(x0, names, tasks, ints, args.workers, args.tmpdir, "base")
    agg["games_per_task"] = args.games
    agg["seeds"] = args.seeds
    agg["opponents"] = sorted({t["name"] for t in tasks})
    out = args.baseline          # 基线写 --baseline 指定的 JSON（搜索结果才用 --out）
    with open(out, "w", encoding="utf-8") as f:
        json.dump(agg, f, ensure_ascii=False, indent=1)
    print(f"✓ 基线已保存 → {out}")
    print("   净胜球 %.3f/场 被射门 %.1f 争球重置 %.1f 门区纪律 %.2f 控球 %.1f%% (%.0fs)"
          % (agg["net"], agg["shots"], agg["fb"], agg["gaf"], agg["poss"], time.time() - t0))
    return 0


def cmd_search(args, base_vals, ints):
    rnd = random.Random(args.rng_seed)
    with open(args.baseline, encoding="utf-8") as f:
        base = json.load(f)
    tasks = make_tasks(args.games, args.seeds, args.opp)
    if not tasks:
        print("✗ 任务池为空，检查 --opp / --seeds"); return 2
    names = GROUPS[args.group]
    dim = len(names)
    lo, hi = [], []
    for n in names:
        d = base_vals[n]
        if n in RANGES:
            a, b = RANGES[n]
        else:
            a, b = d * 0.75, d * 1.25
        a, b = min(a, b), max(a, b)
        a = max(a, d - abs(d) * 0.6 - 0.5)      # 兜底范围别离默认值太远（防"跑飞"）
        b = min(b, d + abs(d) * 0.6 + 0.5)
        lo.append(min(a, d)); hi.append(max(b, d))
    x0 = [base_vals[n] for n in names]
    if args.init:
        # 从已验证的参数文件出发继续搜（坐标上升式：先攻后守，后一组以前一组结果为起点）
        with open(args.init, encoding="utf-8") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line:
                    continue
                p = line.split()
                if len(p) == 2 and p[0] in names:
                    x0[names.index(p[0])] = float(p[1])
        print(f"（起点来自 {args.init}）")
    print(f"=== 差分进化搜索：{dim} 个参数，种群 {args.pop}，{args.gens} 代，"
          f"每候选 {len(tasks)} 个任务（{args.games} 局/任务），{args.workers} 并发 ===")
    print(f"    对手池: {sorted({t['name'] for t in tasks})}  种子: {args.seeds}")

    cache = {}

    def evaluate(x, tag):
        key = tuple(round(v, 4) for v in x)
        if key in cache:
            return cache[key]
        agg = eval_vector(x, names, tasks, ints, args.workers, args.tmpdir, tag)
        f = -9e9 if agg is None else fitness(agg, base)
        cache[key] = (f, agg)
        return cache[key]

    pop_x = [list(x0)]
    for i in range(1, args.pop):
        pop_x.append([min(hi[k], max(lo[k], x0[k] + rnd.gauss(0, 0.15 * (hi[k] - lo[k]))))
                      for k in range(dim)])
    pop_f = []
    for i, x in enumerate(pop_x):
        f, agg = evaluate(x, f"g0_{i}")
        pop_f.append(f)
        print(f"  [初始 {i+1}/{args.pop}] fitness={f:+.4f}  净胜球={agg['net']:.2f} "
              f"被射门={agg['shots']:.0f} 门区={agg['gaf']:.2f} 争球={agg['fb']:.0f}")

    t0 = time.time()
    log_path = os.path.join(args.tmpdir, "train_log.csv")
    with open(log_path, "w", encoding="utf-8") as log:
        log.write("gen,best_fit,mean_fit,best_net,best_shots,best_gaf,best_fb\n")
        for g in range(1, args.gens + 1):
            for i in range(args.pop):
                # DE/rand/1/bin
                idxs = [k for k in range(args.pop) if k != i]
                a, b, c = rnd.sample(idxs, 3)
                trial = []
                jrand = rnd.randrange(dim)
                for k in range(dim):
                    if rnd.random() < args.cr or k == jrand:
                        v = pop_x[a][k] + args.f * (pop_x[b][k] - pop_x[c][k])
                        trial.append(min(hi[k], max(lo[k], v)))
                    else:
                        trial.append(pop_x[i][k])
                ft, aggt = evaluate(trial, f"g{g}_{i}")
                if ft > pop_f[i]:
                    pop_x[i], pop_f[i] = trial, ft
            best = max(range(args.pop), key=lambda k: pop_f[k])
            f, agg = cache[tuple(round(v, 4) for v in pop_x[best])]
            mean_f = sum(pop_f) / args.pop
            log.write("%d,%.4f,%.4f,%.4f,%.2f,%.3f,%.2f\n"
                      % (g, f, mean_f, agg["net"], agg["shots"], agg["gaf"], agg["fb"]))
            log.flush()
            print(f"  第 {g}/{args.gens} 代: 最好 fitness={f:+.4f} 均值={mean_f:+.4f} "
                  f"净胜球={agg['net']:.2f} 被射门={agg['shots']:.0f} 门区={agg['gaf']:.2f} "
                  f"争球={agg['fb']:.0f}  [{time.time()-t0:.0f}s, 已评估 {len(cache)} 个候选]")

    best = max(range(args.pop), key=lambda k: pop_f[k])
    xb = pop_x[best]
    with open(args.out, "w", encoding="ascii") as f:
        for n, v in zip(names, xb):
            f.write("%s %.6g\n" % (n, round(v) if n in ints else v))
    print(f"✓ 最优参数 → {args.out}（fitness={pop_f[best]:+.4f}）")
    for n, v0, v1 in zip(names, x0, xb):
        mark = "  " if abs(v1 - v0) < 1e-9 else "→ "
        print(f"    {n:34s} {v0:>8.4g} {mark}{v1:>8.4g}")
    return 0


def cmd_eval(args, base_vals, ints):
    with open(args.baseline, encoding="utf-8") as f:
        base = json.load(f)
    seeds = args.holdout_seeds if args.holdout else args.seeds
    tasks = make_tasks(args.games, seeds, args.opp)
    names = GROUPS[args.group]
    xb = [base_vals[n] for n in names]
    if args.params:
        with open(args.params, encoding="utf-8") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line:
                    continue
                p = line.split()
                if len(p) == 2 and p[0] in names:
                    xb[names.index(p[0])] = float(p[1])
    agg = eval_vector(xb, names, tasks, ints, args.workers, args.tmpdir, "eval")
    f = fitness(agg, base)
    print(f"=== 评估（{'留出集' if args.holdout else '训练集'} 种子 {list(seeds)}）===")
    print(f"  净胜球 {agg['net']:.3f}/场（基线 {base['net']:.3f}，Δ{agg['net']-base['net']:+.3f}）")
    print(f"  被射门 {agg['shots']:.1f}（基线 {base['shots']:.1f}）"
          f"  门区纪律 {agg['gaf']:.2f}（基 {base['gaf']:.2f}）"
          f"  争球 {agg['fb']:.1f}（基 {base['fb']:.1f}）")
    print(f"  fitness = {f:+.4f}  {'✓ 可接受' if f > 0 else '✗ 未提升'}")
    return 0


def main():
    global BENCH          # 必须在任何使用之前声明（--binary 的默认值也用到它）
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["baseline", "search", "eval"])
    ap.add_argument("--group", default="pilot", choices=sorted(GROUPS))
    ap.add_argument("--games", type=int, default=10, help="每个任务（对手×种子）跑几局")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--holdout-seeds", type=int, nargs="+", default=[1001, 1002, 1003])
    ap.add_argument("--opp", nargs="*", default=None, help="只保留这些对手")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--pop", type=int, default=12)
    ap.add_argument("--gens", type=int, default=6)
    ap.add_argument("--f", type=float, default=0.6, help="DE 差分权重")
    ap.add_argument("--cr", type=float, default=0.9, help="DE 交叉率")
    ap.add_argument("--rng-seed", type=int, default=20260916)
    ap.add_argument("--baseline", default=os.path.join("docs", "work", "tune_baseline.json"))
    ap.add_argument("--out", default=os.path.join("docs", "work", "best_params.txt"))
    ap.add_argument("--params", default=None, help="eval 模式：要评估的参数文件")
    ap.add_argument("--init", default=None, help="search 模式：从该参数文件出发（缺省=默认值）")
    ap.add_argument("--holdout", action="store_true")
    ap.add_argument("--tmpdir", default=os.path.join("build", "tune"))
    ap.add_argument("--binary", default=BENCH)
    a = ap.parse_args()
    BENCH = a.binary
    os.makedirs(a.tmpdir, exist_ok=True)
    base_vals, ints = load_baseline_params()
    if a.mode == "baseline":
        return cmd_baseline(a, base_vals, ints)
    if a.mode == "search":
        return cmd_search(a, base_vals, ints)
    return cmd_eval(a, base_vals, ints)


if __name__ == "__main__":
    sys.exit(main())
