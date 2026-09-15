# -*- coding: utf-8 -*-
"""calibrate_sim.py — 用真机日志"反推"仿真物理参数（系统辨识 / 自动定标）。

思路（生活化）：仿真和真机各跑一次"体检"，量同一批指标（球减速多快、撞墙弹多少、
机器人跑多快），然后**拧仿真那 11 个旋钮，让两份体检报告尽量一致**。
拧旋钮的活儿交给前面写好的差分进化（DE）。

前提：sim_bench.cpp 里的物理常量要先变成可注入旋钮（前缀 "sim."）——
      改动见 docs/06 轮次 75。本脚本只负责"怎么拧"。

真机靶子来自：
  · tools/py/calib_probe_result.json（calib_probe.py 产出）
     - HNNU 段 = 我方机器人速度/加速度
     - DEMO 段 = 官方 demo（脚本对手的参照）+ 球/墙统计（样本更多）

用法：
    python tools\\py\\calibrate_sim.py --games 3 --gens 12 --pop 14      # 自动定标
    python tools\\py\\calibrate_sim.py --eval-only                        # 只看当前差距
"""
import argparse
import json
import os
import random
import subprocess
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_stats  # 同一口径的仿真侧统计量

BENCH = os.path.join("build", "Release", "sim_bench.exe")
PROBE = os.path.join("tools", "py", "calib_probe_result.json")

# 要拧的仿真旋钮：(名字, 当前值, 下界, 下界说明见 docs/06 轮次 75)
SIM_PARAMS = {
    "sim.kSpeed":     (0.9,   0.6,  2.2),    # 轮速→cm/s 缩放（真机比现在快得多）
    "sim.kAccel":     (300.0, 150.0, 700.0),  # 轮速最大加速度 cm/s²
    "sim.kWheelBase": (10.0,  6.0,  20.0),   # 轮距（转向灵敏度）
    "sim.kBallDecay": (0.985, 0.988, 0.9999),# 球每帧衰减（真机实测 0.993~1.000）
    "sim.kWallRest":  (0.66,  0.25, 0.95),   # 撞墙法向恢复（真机实测 ~0.46）
    "sim.kWallFric":  (0.81,  0.5,  1.0),    # 撞墙切向保持（真机实测 ~0.78）
    "sim.kDeflect":   (0.45,  0.1,  0.9),    # 守门员挡球恢复
    "sim.kContact":   (5.5,   3.0,  9.0),    # 球-机器人最小分离
    "sim.kCarryR":    (9.0,   5.0,  14.0),   # 携带区半径
    "sim.kCarryArc":  (40.0,  15.0, 80.0),   # 携带区前向半弧（度）
    "sim.kRobotR":    (6.0,   3.0,  10.0),   # 机器人最小间距
    "sim.kPushKeep":  (0.3,   0.0,  0.7),    # 推球动量：保留旧球速比例（当前硬编码 0.3）
    "sim.kPushGain":  (0.7,   0.3,  1.5),    # 推球动量：机器人速度注入比例（当前硬编码 0.7）
    "sim.kActDelay":  (0.0,   0.0,  3.0),    # 新增：指令生效延迟帧数（真机至少 1 帧）
    "sim.kWheelDead": (0.0,   0.0,  15.0),   # 新增：轮速死区（小命令推不动）
    # —— 脚本对手速度：从写死的 80/30/50/40 拆成独立旋钮 ——
    # 实测根因：脚本对手慢一半不是物理，而是这几条命令上限太小
    #   （80*0.9=72cm/s 正好等于实测仿真对手 p99=1.80cm/帧）。
    # 不能靠调 strength 解决：strength 还会改掉照真机 demo 校准过的"手抖/反应延迟"。
    "sim.oppChaseSpeed":     (80.0, 40.0, 150.0),   # 追击手（远球）
    "sim.oppChaseNearSpeed": (30.0, 10.0, 80.0),    # 追击手（近球 20cm 内）
    "sim.oppSupportSpeed":   (50.0, 20.0, 120.0),   # 协防
    "sim.oppFormationSpeed": (40.0, 15.0, 120.0),   # 阵型站位
    "sim.oppGkSpeed":        (30.0, 10.0, 80.0),    # 门将横向（cm/s）
}

# 要匹配的统计量：仿真字段 → (真机字段, 真机来源段, 权重)
MATCH = [
    ("blue_spd_p50",   "blue_spd_p50",   "Hnnu", 1.0),
    ("blue_spd_p90",   "blue_spd_p90",   "Hnnu", 1.0),
    ("blue_spd_p95",   "blue_spd_p95",   "Hnnu", 1.5),
    ("blue_spd_p99",   "blue_spd_p99",   "Hnnu", 1.0),
    ("blue_acc_p90",   "blue_acc_p90",   "Hnnu", 1.0),
    ("blue_acc_p95",   "blue_acc_p95",   "Hnnu", 1.0),
    ("ball_spd_p50",   "ball_spd_p50",   "Hnnu", 1.0),
    ("ball_spd_p90",   "ball_spd_p90",   "Hnnu", 1.5),
    ("ball_spd_p95",   "ball_spd_p95",   "Hnnu", 2.0),
    ("ball_spd_p99",   "ball_spd_p99",   "Hnnu", 1.5),
    ("yellow_spd_p50", "yellow_spd_p50", "DEMO", 1.0),
    ("yellow_spd_p90", "yellow_spd_p90", "DEMO", 1.0),
    ("yellow_spd_p95", "yellow_spd_p95", "DEMO", 1.5),
    ("yellow_spd_p99", "yellow_spd_p99", "DEMO", 1.0),
    ("yellow_acc_p95", "yellow_acc_p95", "DEMO", 1.0),
    ("decay_2_3",      "decay_2_3",      "DEMO", 1.0),
    ("decay_3_5",      "decay_3_5",      "DEMO", 2.0),
    ("decay_5p",       "decay_5p",       "DEMO", 2.0),
    ("rest_x",         "rest_x",         "DEMO", 2.0),
    ("rest_y",         "rest_y",         "DEMO", 2.0),
    ("fric_y",         "fric_y",         "DEMO", 1.0),
]


def load_targets():
    with open(PROBE, encoding="utf-8") as f:
        probe = json.load(f)
    tgt = {}
    for sim_key, real_key, section, w in MATCH:
        sec = probe.get(section) or probe.get(section.upper()) or probe.get(section.capitalize()) or {}
        v = sec.get(real_key)
        if v is None or (isinstance(v, float) and v != v):   # None / NaN
            continue
        tgt[sim_key] = (float(v), float(w))
    return tgt


def write_params(x, names, path):
    with open(path, "w", encoding="ascii") as f:
        for n, v in zip(names, x):
            f.write("%s %.6g\n" % (n, v))


def sim_stats_for(x, names, games, seeds, tmpdir, tag, frames=24000, binary=None):
    """跑 games 局（多种子）仿真并汇总统计量。"""
    pfile = os.path.join(tmpdir, "simpact_%s.txt" % tag)
    write_params(x, names, pfile)
    all_frames = []
    for s in seeds[:games]:
        traj = os.path.join(tmpdir, "traj_%s_%d.csv" % (tag, s))
        cmd = [binary or BENCH, "--games", "1", "--frames", str(frames), "--seed", str(s),
               "--traj", traj, "--params", pfile]
        r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
        if not os.path.exists(traj):
            return None
        all_frames.extend(sim_stats.load(traj))
        try:
            os.remove(traj)          # 统计量算完就删（单文件 ~1.5MB，一轮 130 次评估会攒到几百 MB）
        except OSError:
            pass
    if len(all_frames) < 3000:
        return None
    return sim_stats.compute(all_frames)


def loss_of(st, tgt):
    """加权相对平方误差；缺项的统计量跳过并计数。"""
    tot_w, acc, used, missing = 0.0, 0.0, 0, 0
    for k, (rv, w) in tgt.items():
        sv = st.get(k)
        if sv is None or (isinstance(sv, float) and sv != sv):
            missing += 1
            continue
        denom = max(abs(rv), 1e-3)
        acc += w * ((sv - rv) / denom) ** 2
        tot_w += w
        used += 1
    if tot_w <= 0:
        return 1e9, used, missing
    return acc / tot_w, used, missing


def report(st, tgt, title):
    print(f"\n=== {title} ===")
    print(f"{'统计量':>16} {'真机':>9} {'仿真':>9} {'相对误差':>9}")
    worst = []
    for k, (rv, w) in sorted(tgt.items()):
        sv = st.get(k)
        if sv is None or sv != sv:
            print(f"{k:>16} {rv:>9.4f} {'缺':>9}")
            continue
        rel = (sv - rv) / max(abs(rv), 1e-3)
        print(f"{k:>16} {rv:>9.4f} {sv:>9.4f} {rel*100:>8.1f}%")
        worst.append((abs(rel), k, rel))
    worst.sort(reverse=True)
    print("  偏差最大三项:", ", ".join(f"{k}({r*100:+.0f}%)" for _, k, r in worst[:3]))


def main():
    global BENCH          # 必须在任何使用之前声明（--binary 默认值也用它）
    ap = argparse.ArgumentParser()
    ap.add_argument("--games", type=int, default=3, help="每次评估跑几局（多种子汇总统计量）")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--gens", type=int, default=12)
    ap.add_argument("--pop", type=int, default=14)
    ap.add_argument("--f", type=float, default=0.6)
    ap.add_argument("--cr", type=float, default=0.9)
    ap.add_argument("--rng-seed", type=int, default=20260916)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--out", default=os.path.join("docs", "work", "sim_params_calibrated.txt"))
    ap.add_argument("--tmpdir", default=os.path.join("build", "calib"))
    ap.add_argument("--frames", type=int, default=24000, help="每局跑多少帧（降成本用）")
    ap.add_argument("--binary", default=BENCH,
                    help="用哪个 sim_bench（定标必须用打了 sim.* 补丁的那个）")
    a = ap.parse_args()
    BENCH = a.binary
    os.makedirs(a.tmpdir, exist_ok=True)

    tgt = load_targets()
    print(f"真机靶子 {len(tgt)} 项（来自 {PROBE}）")
    names = list(SIM_PARAMS)
    lo = [SIM_PARAMS[n][1] for n in names]
    hi = [SIM_PARAMS[n][2] for n in names]
    x0 = [SIM_PARAMS[n][0] for n in names]

    cache = {}

    def evaluate(x, tag):
        key = tuple(round(v, 5) for v in x)
        if key in cache:
            return cache[key]
        st = sim_stats_for(x, names, a.games, a.seeds, a.tmpdir, tag, a.frames, a.binary)
        if st is None:
            res = (1e9, None)
        else:
            l, used, missing = loss_of(st, tgt)
            res = (l, st)
        cache[key] = res
        return res

    l0, st0 = evaluate(x0, "base")
    report(st0, tgt, "定标前（当前仿真 vs 真机）")
    print(f"  损失 = {l0:.5f}")
    if a.eval_only:
        return 0

    rnd = random.Random(a.rng_seed)
    dim = len(names)
    pop_x = [list(x0)]
    for _ in range(a.pop - 1):
        pop_x.append([min(hi[k], max(lo[k], x0[k] * (1.0 + rnd.gauss(0, 0.25))))
                      for k in range(dim)])
    pop_f = []
    t0 = time.time()
    for i, x in enumerate(pop_x):
        f, _ = evaluate(x, "g0_%d" % i)
        pop_f.append(f)
        print(f"  [初始 {i+1}/{a.pop}] loss={f:.5f}  [{time.time()-t0:.0f}s]")
    with open(os.path.join(a.tmpdir, "calib_log.csv"), "w", encoding="utf-8") as log:
        log.write("gen,best,mean\n")
        for g in range(1, a.gens + 1):
            for i in range(a.pop):
                idxs = [k for k in range(a.pop) if k != i]
                p, q, r = rnd.sample(idxs, 3)
                trial, jr = [], rnd.randrange(dim)
                for k in range(dim):
                    if rnd.random() < a.cr or k == jr:
                        v = pop_x[p][k] + a.f * (pop_x[q][k] - pop_x[r][k])
                        trial.append(min(hi[k], max(lo[k], v)))
                    else:
                        trial.append(pop_x[i][k])
                ft, _ = evaluate(trial, "g%d_%d" % (g, i))
                if ft < pop_f[i]:        # 定标是最小化
                    pop_x[i], pop_f[i] = trial, ft
            b = min(range(a.pop), key=lambda k: pop_f[k])
            log.write("%d,%.5f,%.5f\n" % (g, pop_f[b], sum(pop_f) / a.pop))
            log.flush()
            # 每代都把"当前最优"单独落盘：万一最后一步出问题（第一次跑就踩了
            # UnicodeEncodeError 把结果写空），也不会白跑一场
            with open(os.path.join(a.tmpdir, "calib_best_sofar.txt"), "w", encoding="utf-8") as bf:
                for n, v in zip(names, pop_x[b]):
                    bf.write("%s %.6g\n" % (n, v))
            print(f"  第 {g}/{a.gens} 代: 最好 loss={pop_f[b]:.5f} 均值={sum(pop_f)/a.pop:.5f}"
                  f"  [{time.time()-t0:.0f}s, 已评估 {len(cache)}]")
    b = min(range(a.pop), key=lambda k: pop_f[k])
    xb = pop_x[b]
    _, stb = evaluate(xb, "best")
    report(stb, tgt, "定标后（最优参数 vs 真机）")
    print(f"  损失 {l0:.5f} → {pop_f[b]:.5f}")
    with open(a.out, "w", encoding="utf-8") as f:      # 必须 utf-8：注释里有中文
        for n, v0, v1 in zip(names, x0, xb):
            f.write("%s %.6g   # 原 %.6g\n" % (n, v1, v0))
    print(f"→ {a.out}")
    for n, v0, v1 in zip(names, x0, xb):
        print(f"    {n:18s} {v0:>9.4g} → {v1:>9.4g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
