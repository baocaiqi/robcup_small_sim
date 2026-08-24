# -*- coding: utf-8 -*-
"""
mark_analysis.py — 人盯人（man-marking）行为复盘

对自博弈 .rlg 逐帧检查 PASSIVE 球员（index 4）是否真的贴住了
「进攻威胁最大」的对方球员，并统计真实比分（去重进球）。

用法：
    python mark_analysis.py 比赛.rlg
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import rlg_analyzer as R

MARK_DIST = 16.0     # 与 C++ mark_dist() 一致
K = 10.0             # 威胁打分平滑
NEAR_TOL = 20.0      # 判「贴住」的容差（motion 有超调，12cm 太紧）


def dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def ball_danger_speed(vx, vy, bx, by, gx):
    """球朝己方门心(gx,90)的速度分量（cm/帧），背离=0。与 C++ ball_danger_speed 一致"""
    dx, dy = gx - bx, 90.0 - by
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0
    return max(0.0, (vx * dx + vy * dy) / d)


def ball_approach_speed(vx, vy, bx, by, px, py):
    """球朝某点(px,py)的速度分量（cm/帧），背离=0。与 C++ ball_approach_speed 一致"""
    dx, dy = px - bx, py - by
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0
    return max(0.0, (vx * dx + vy * dy) / d)


def mark_threat(d_ball, d_goal, approach_speed, danger_speed, is_dribbler):
    s = 50.0 / (d_ball + K) + 25.0 / (d_goal + K)
    reach = 1.0 / (1.0 + d_ball / 20.0)
    s += 0.4 * approach_speed * reach
    if is_dribbler:
        s += 0.3 * danger_speed
    return s


def mark_position(opp, gx, gy):
    """被盯者 opp 的目标点（挡在其与己方门之间 12cm）"""
    dx, dy = gx - opp["x"], gy - opp["y"]
    l = math.hypot(dx, dy)
    if l < 1e-6:
        return opp["x"], opp["y"]
    dx, dy = dx / l, dy / l
    return opp["x"] + dx * MARK_DIST, opp["y"] + dy * MARK_DIST


def analyze_side(frames, home_key, opp_key, gx, gy, own_half_fn, name):
    """home_key 队的 PASSIVE(index4) 盯 opp_key 队；gx,gy 为己方门"""
    total_def = 0        # 球在己方半场（防守）的帧数
    mark_hit = 0         # 贴住「最威胁」对手的帧数
    mark_any = 0         # 贴住任一对手的帧数
    d_mark_sum = 0.0     # 到正确贴人点平均距离
    d_ball_sum = 0.0     # 到球平均距离（判断在盯人还是追球）

    for i in range(1, len(frames)):
        f = frames[i]
        bx, by = f["ball"]["x"], f["ball"]["y"]
        if not own_half_fn(bx):
            continue
        total_def += 1
        h = f[home_key][4]
        hx, hy = h["x"], h["y"]
        opp = f[opp_key]

        # 球速：相邻帧差分（cm/帧）
        px, py = frames[i - 1]["ball"]["x"], frames[i - 1]["ball"]["y"]
        vx, vy = bx - px, by - py
        danger = ball_danger_speed(vx, vy, bx, by, gx)

        # 持球者 = 离球最近对手
        dribbler, dmin = -1, 1e9
        for t, op in enumerate(opp):
            db = dist(bx, by, op["x"], op["y"])
            if db < dmin:
                dmin, dribbler = db, t
        # 威胁分取 argmax（接球威胁 approach + 持球突破 danger）
        best, best_score = -1, -1e9
        for t, op in enumerate(opp):
            db = dist(bx, by, op["x"], op["y"])
            dg = abs(op["x"] - gx)
            appr = ball_approach_speed(vx, vy, bx, by, op["x"], op["y"])
            sc = mark_threat(db, dg, appr, danger, t == dribbler)
            if sc > best_score:
                best_score, best = sc, t

        mx, my = mark_position(opp[best], gx, gy)
        d_mark = dist(hx, hy, mx, my)
        d_ball = dist(hx, hy, bx, by)
        d_mark_sum += d_mark
        d_ball_sum += d_ball

        if d_mark < NEAR_TOL:
            mark_hit += 1
        for op in opp:
            if 5.0 <= dist(hx, hy, op["x"], op["y"]) <= 25.0:
                mark_any += 1
                break

    if total_def == 0:
        print(f"[{name}] 无防守帧（球从未进己方半场）")
        return
    print(f"\n===== {name} 人盯人复盘 =====")
    print(f"  防守帧(球在己方半场): {total_def}")
    print(f"  贴住「最威胁」对手率 : {mark_hit/total_def*100:5.1f}%  "
          f"({mark_hit}/{total_def})")
    print(f"  贴住任一对手率       : {mark_any/total_def*100:5.1f}%")
    print(f"  平均到正确贴人点距离 : {d_mark_sum/total_def:5.1f} cm  "
          f"(<=20 视为贴住)")
    print(f"  平均到球距离         : {d_ball_sum/total_def:5.1f} cm  "
          f"(大=在盯人而非追球)")


def dedup_goals(frames):
    """去重进球：连续越过门线的帧算 1 个球"""
    goals = []
    in_goal = None
    for i, f in enumerate(frames):
        bx, by = f["ball"]["x"], f["ball"]["y"]
        side = None
        if bx > 220.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            side = "yellow_scores"   # 球过 x=220(蓝队右门) → 黄得分
        elif bx < 0.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            side = "blue_scores"     # 球过 x=0(黄队左门) → 蓝得分
        if side and side != in_goal:
            goals.append((i, side))
        in_goal = side
    return goals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg")
    args = ap.parse_args()
    frames = R.parse_rlg(args.rlg)

    goals = dedup_goals(frames)
    print(f"===== 真实比分去重: {os.path.basename(args.rlg)} =====")
    print(f"总帧数 {len(frames)} 约 {len(frames)/40:.1f}s")
    blue = sum(1 for _, s in goals if s == "blue_scores")
    yellow = sum(1 for _, s in goals if s == "yellow_scores")
    print(f"进球: {len(goals)} 个 → 蓝 {blue} : {yellow} 黄")
    for i, s in goals:
        who = "蓝队得分" if s == "blue_scores" else "黄队得分"
        print(f"  帧 {i} (~{i/40:.1f}s): {who}")

    # 蓝队（守 x=220 右门）PASSIVE=blue[4] 盯 yellow；己方半场 x>110
    analyze_side(frames, "blue", "yellow", 220.0, 90.0,
                 lambda x: x > 110.0, "蓝队 PASSIVE(blue[4]) 盯黄")
    # 黄队（守 x=0 左门）PASSIVE=yellow[4] 盯 blue；己方半场 x<110
    analyze_side(frames, "yellow", "blue", 0.0, 90.0,
                 lambda x: x < 110.0, "黄队 PASSIVE(yellow[4]) 盯蓝")


if __name__ == "__main__":
    main()
