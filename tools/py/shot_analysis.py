"""shot_analysis.py — 射门与"有效射门"指标（真机 rlg 轨迹 / sim --traj 同一把尺子）

为什么需要它：sim_bench 现有的"射门"统计口径 = 球进入对方门区 30cm 内的帧计数
（一场 380~420 次），它衡量的是"压迫"，**不是射门**。要优化"有效射门少"，
必须先能算出：射门机会（球在射程内且我方贴球）→ 射门尝试 → 射正 → 进球。

用法：
    python tools/py/shot_analysis.py build/traj_before.csv [--our b|y] [--range 110]

输入 CSV 列名：b0_x,b0_y,...,b4_x,b4_y, y0_x,y0_y,...,y4_x,y4_y, ball_x,ball_y（40Hz，cm）
    蓝队守 x=220（攻 x=0），黄队守 x=0（攻 x=220）——见 AGENTS.md / formation.cpp。

判据（全部可从轨迹复算，无需球权/PlayMode 字段）：
  · 射门尝试：球在距门线 range cm 内、朝该门运动且球速 ≥ v_min 的一段连续帧（去重去抖）
  · 射正：该段的匀速外推与门线交点落在门框内 [70,110]
  · 进球：球位真正越过门线且 y 在门框内（连续帧去重）
  · 射门机会帧：球在距对方门线 range 内 **且 我方最近球员距球 <25cm**（=我们在球跟前有射门位）
    —— 这是定位瓶颈的关键：机会多而射门少 → 射门函数问题；机会本身就少 → 上游（控球/推进）问题
输出 ASCII，避免 GBK 控制台乱码。
"""
import argparse
import csv
import math
import sys

DT = 1.0 / 40.0
GOAL_LO, GOAL_HI = 70.0, 110.0     # 门框 y 范围（TeamContext GOAL_WIDTH=40，中心 90）
COOLDOWN = 30                      # 帧：同一段射门去重
V_MIN_SHOT = 20.0                  # cm/s：算一次"射门"的最低球速
TOUCH_LOOKBACK = 10                # 帧：要求此前 10 帧内我方贴过球（=我方推出去的，排除解围/折射）


def load(path):
    with open(path, encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def nearest_own(rows, i, side):
    """我方最近球员到球的距离（side='b' 为蓝队=我们）。"""
    bx, by = float(rows[i]["ball_x"]), float(rows[i]["ball_y"])
    return min(math.hypot(bx - float(rows[i][f"{side}{k}_x"]),
                          by - float(rows[i][f"{side}{k}_y"])) for k in range(5))


def analyse(rows, goal_x, shooter_side, label, rng_cm):
    """goal_x：被攻的门线 x；shooter_side：'b'/'y' 或 None（不区分谁在攻）"""
    n = len(rows)
    shots, ontarget, goals = [], [], []
    cd = 0
    for i in range(1, n):
        bx, by = float(rows[i]["ball_x"]), float(rows[i]["ball_y"])
        pbx, pby = float(rows[i - 1]["ball_x"]), float(rows[i - 1]["ball_y"])
        vx, vy = (bx - pbx) / DT, (by - pby) / DT
        # —— 疑似进球：上一帧球在门线附近、y 在门框内、朝门运动；本帧瞬移回中圈 ——
        #   平台/ sim 在进球当帧就把球重置，轨迹里看不到越线，只能靠"门内→中圈"跳变识别
        toward = (vx > 0) if goal_x > 110.0 else (vx < 0)
        near_goal_prev = abs(pbx - goal_x) <= 12.0
        mouth_prev = GOAL_LO <= pby <= GOAL_HI
        reset_now = abs(bx - 110.0) < 25.0 and abs(by - 90.0) < 25.0
        if near_goal_prev and mouth_prev and reset_now and cd == 0:
            goals.append((i, pby))
            cd = COOLDOWN
            continue
        if math.hypot(vx, vy) > 800.0:          # 其他重置瞬移：跳过
            continue
        d_goal = abs(bx - goal_x)
        if cd > 0:
            cd -= 1
        if d_goal <= rng_cm and toward and abs(vx) > V_MIN_SHOT:
            # "射门"= 我方把球朝对方门推出去：要求近若干帧内我方有人贴过球（排除解围/折射）
            touched = True
            if shooter_side:
                touched = False
                for j in range(max(0, i - TOUCH_LOOKBACK), i + 1):
                    if nearest_own(rows, j, shooter_side) < 25.0:
                        touched = True
                        break
            if touched and cd == 0:
                cd = COOLDOWN
                shots.append((i, bx, by))
                t = (goal_x - bx) / vx
                y_line = by + vy * t
                if GOAL_LO <= y_line <= GOAL_HI:
                    ontarget.append((i, y_line))
    # 射门机会帧：球在射程内 + 我方贴球
    opp_frames = 0
    if shooter_side:
        for i in range(n):
            bx = float(rows[i]["ball_x"])
            if abs(bx - goal_x) > rng_cm:
                continue
            if nearest_own(rows, i, shooter_side) < 25.0:
                opp_frames += 1
    mins = n * DT / 60.0
    print(f"--- {label} (攻 x={goal_x:.0f} 门) ---")
    print(f"  时长 {n*DT:.0f}s")
    print(f"  射门尝试 {len(shots)} 次（{len(shots)/mins:.1f}/分钟）")
    print(f"  射正     {len(ontarget)} 次（射正率 {100.0*len(ontarget)/max(1,len(shots)):.0f}%）")
    print(f"  疑似进球 {len(goals)} 个（判据：门前→瞬移回中圈；以比分日志为准）")
    if shooter_side:
        print(f"  射门机会帧（球在 {rng_cm:.0f}cm 内 + 我方贴球<25cm）: {opp_frames} 帧"
              f"（{100.0*opp_frames/n:.1f}% 时间，{opp_frames/mins:.1f} 帧/分钟）")


def align_stats(rows, goal_x, side, label):
    """对准率：我方推球瞬间，"机头方向"与"瞄准线（球→对方门中心）"的夹角。

    为什么量这个：本平台没有踢球动作，球出射方向 ≈ 撞球瞬间机头方向。
      机头−瞄准线 = 因（可控）；球出射方向−瞄准线 = 果（=射正）。
      优化要盯"因"：把机头误差压到 ≤10°，1m 处横向偏差 = 100·tan10° ≈ 17.6cm < 门半宽 20cm。
    局限：真机 CSV 无 rotation 列 → 用"撞球前几帧机器人自身运动方向"当机头代理
      （机器人是朝前开过去撞球的，两者≈一致；位移不足 0.5cm 时退回"机器人→球"方向）。
    """
    n = len(rows)
    head_err, out_err = [], []
    i = 3
    while i < n - 2:
        bx, by = float(rows[i]["ball_x"]), float(rows[i]["ball_y"])
        pbx, pby = float(rows[i - 1]["ball_x"]), float(rows[i - 1]["ball_y"])
        vx, vy = (bx - pbx) / DT, (by - pby) / DT
        spd = math.hypot(vx, vy)
        toward = (vx > 0) if goal_x > 110.0 else (vx < 0)
        if spd > 18.0 and toward and abs(bx - goal_x) < 200.0:
            # 我方最近球员在球附近（≤30cm）→ 认定为"我方推出去的"
            best, bd = None, 1e9
            for k in range(5):
                d = math.hypot(bx - float(rows[i][f"{side}{k}_x"]),
                               by - float(rows[i][f"{side}{k}_y"]))
                if d < bd:
                    bd, best = d, k
            if bd < 30.0:
                # 瞄准线：球 → 对方门中心
                aim = math.degrees(math.atan2(90.0 - by, goal_x - bx))
                # 机头代理：机器人最近 3 帧运动方向；位移太小则用"机器人→球"方向
                hx = float(rows[i][f"{side}{best}_x"]) - float(rows[i - 3][f"{side}{best}_x"])
                hy = float(rows[i][f"{side}{best}_y"]) - float(rows[i - 3][f"{side}{best}_y"])
                if math.hypot(hx, hy) > 0.5:
                    head = math.degrees(math.atan2(hy, hx))
                else:
                    head = math.degrees(math.atan2(by - float(rows[i][f"{side}{best}_y"]),
                                                   bx - float(rows[i][f"{side}{best}_x"])))
                head_err.append(abs(((head - aim + 180.0) % 360.0) - 180.0))
                # 出射方向（果）：接下来 3 帧平均
                ex = ey = 0.0
                for j in range(i, min(i + 3, n)):
                    ex += float(rows[j]["ball_x"]) - float(rows[j - 1]["ball_x"])
                    ey += float(rows[j]["ball_y"]) - float(rows[j - 1]["ball_y"])
                if math.hypot(ex, ey) > 1e-6:
                    out = math.degrees(math.atan2(ey, ex))
                    out_err.append(abs(((out - aim + 180.0) % 360.0) - 180.0))
                i += 15                       # 去重：一次推球只记一次
                continue
        i += 1
    if not head_err:
        print(f"--- {label} 对准率 --- 无有效推球样本")
        return
    head_err.sort()
    out_err.sort()

    def share(v, lim):
        return 100.0 * sum(1 for e in v if e <= lim) / len(v)

    print(f"--- {label} 对准率（n={len(head_err)} 次推球）---")
    print(f"  机头−瞄准线（因，要压这个）: p50={head_err[len(head_err)//2]:.1f}°  "
          f"p90={head_err[int(len(head_err)*0.9)]:.1f}°  ≤10°: {share(head_err,10):.0f}%  "
          f"≤20°: {share(head_err,20):.0f}%  >40°: {share(head_err,1e9)-share(head_err,40):.0f}%")
    if out_err:
        print(f"  出射−瞄准线（果）: p50={out_err[len(out_err)//2]:.1f}°  "
              f"≤10°: {share(out_err,10):.0f}%")
    return head_err


def shot_posture_stats(rows, goal_x, side, label, rng_cm=110.0):
    """射门姿态下的对准率：只统计"我方在球后方（球在人与门之间）+ 球在射程内"的推球。

    为什么要这个口径：全量推球里绝大多数是争抢/解围时的无意触碰（sim 里 ≈1 次/秒），
    它们的机头朝向由追逐路径决定，跟射门决策无关，会把统计稀释成噪声。
    射门姿态 = 真正该被射门逻辑管住的那一批：
      · 球在距对方门线 rng_cm 内（有射门价值）
      · 我方最近球员在球后方：dot((robot−ball), (goal−ball)) < 0（球在人和门之间）
      · 该球员距球 < 25cm（球在脚下）
    """
    n = len(rows)
    errs = []
    i = 3
    while i < n - 2:
        bx, by = float(rows[i]["ball_x"]), float(rows[i]["ball_y"])
        pbx, pby = float(rows[i - 1]["ball_x"]), float(rows[i - 1]["ball_y"])
        vx, vy = (bx - pbx) / DT, (by - pby) / DT
        spd = math.hypot(vx, vy)
        toward = (vx > 0) if goal_x > 110.0 else (vx < 0)
        if spd > 18.0 and toward and abs(bx - goal_x) < rng_cm:
            best, bd = None, 1e9
            for k in range(5):
                d = math.hypot(bx - float(rows[i][f"{side}{k}_x"]),
                               by - float(rows[i][f"{side}{k}_y"]))
                if d < bd:
                    bd, best = d, k
            if best is not None and bd < 25.0:
                rx = float(rows[i][f"{side}{best}_x"]) - bx
                ry = float(rows[i][f"{side}{best}_y"]) - by
                gx, gy = goal_x - bx, 90.0 - by
                behind = (rx * gx + ry * gy) < 0.0        # 球在人与门之间
                if behind:
                    aim = math.degrees(math.atan2(gy, gx))
                    hx = float(rows[i][f"{side}{best}_x"]) - float(rows[i - 3][f"{side}{best}_x"])
                    hy = float(rows[i][f"{side}{best}_y"]) - float(rows[i - 3][f"{side}{best}_y"])
                    if math.hypot(hx, hy) > 0.5:
                        head = math.degrees(math.atan2(hy, hx))
                    else:
                        head = math.degrees(math.atan2(-ry, -rx))
                    errs.append(abs(((head - aim + 180.0) % 360.0) - 180.0))
                    i += 15
                    continue
        i += 1
    if not errs:
        print(f"--- {label} 射门姿态对准率 --- 无样本")
        return
    errs.sort()
    s10 = 100.0 * sum(1 for e in errs if e <= 10.0) / len(errs)
    s20 = 100.0 * sum(1 for e in errs if e <= 20.0) / len(errs)
    print(f"--- {label} 射门姿态对准率（球在门侧后方+射程 {rng_cm:.0f}cm 内，n={len(errs)}）---")
    print(f"  p50={errs[len(errs)//2]:.1f}°  p90={errs[int(len(errs)*0.9)]:.1f}°  "
          f"≤10°: {s10:.0f}%  ≤20°: {s20:.0f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--our", default="b", choices=["b", "y", "none"],
                    help="哪一侧是'我们'（默认 b=蓝）；none=不分敌我")
    ap.add_argument("--range", type=float, default=110.0, help="射程 cm（默认 110，同 kMaxShot）")
    args = ap.parse_args()
    rows = load(args.csv)
    print(f"===== shot_analysis: {args.csv}, {len(rows)} frames ~ {len(rows)*DT:.0f}s, "
          f"射程={args.range:.0f}cm =====")
    our = None if args.our == "none" else args.our
    # 我们攻哪边：蓝队守 x=220 → 攻 x=0；黄队反之
    if our == "b":
        analyse(rows, 0.0, "b", "我们(蓝)进攻", args.range)
        analyse(rows, 220.0, None, "对手(黄)进攻", args.range)
        align_stats(rows, 0.0, "b", "我们(蓝)推球")
        shot_posture_stats(rows, 0.0, "b", "我们(蓝)")
    elif our == "y":
        analyse(rows, 220.0, "y", "我们(黄)进攻", args.range)
        analyse(rows, 0.0, None, "对手(蓝)进攻", args.range)
        align_stats(rows, 220.0, "y", "我们(黄)推球")
        shot_posture_stats(rows, 220.0, "y", "我们(黄)")
    else:
        analyse(rows, 0.0, None, "左边门", args.range)
        analyse(rows, 220.0, None, "右边门", args.range)


if __name__ == "__main__":
    sys.exit(main())
