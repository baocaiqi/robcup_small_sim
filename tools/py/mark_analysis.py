# -*- coding: utf-8 -*-
"""
mark_analysis.py — 人盯人（man-marking）行为复盘

对自博弈 .rlg 逐帧检查 PASSIVE 球员（index 4）是否真的贴住了
「进攻威胁最大」的对方球员，并统计真实比分（去重进球）。

尺子已与 C++ 逐帧对齐（2026-08-26）：
  · 只在 threat_level>=0.6（人盯人激活）帧计数，排除区域防守帧；
  · 选人镜像 pick_mark_target（含 1.1 滞回 + 危险门限）；
  · 站位镜像 run_passive（速度前馈 mark_lead + 传球线分支 + goal-side + 罚球区推出）。

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

# 与 C++ 一致的标记参数（见 include/simuro5/defense.hpp + roles.cpp run_passive）
MARK_LEAD = 3.0               # mark_lead()：速度前馈预测帧数（与 C++ 同步 6→3）
MARK_PASS_LANE_DIST = 40.0    # mark_pass_lane_dist()：传球线判定上界
MARK_DRIBBLE_DIST = 15.0      # run_passive 持球者判定下界（<15 视为持球，堵射门非传球）
MARK_ENGAGE_BALL_DIST = 40.0  # mark_engage_ball_dist()：危险门限
MARK_ENGAGE_GOAL_DIST = 40.0  # mark_engage_goal_dist()：危险门限
K_THREAT_DANGER_SPEED = 6.0   # threat_from_state 朝门速度阈值
K_STATE_HYST = 3              # team_state 滞回帧数
K_HAS_BALL_DIST = 20.0        # we_have_ball 离球阈值
K_MAX_OPP_VEL = 8.0            # world_model.cpp kMaxOppVel：单帧位移超此值视为复位跳变、速度清零
PENALTY_PUSH_X = 85.0         # run_passive 罚球区推出 x 偏移


def dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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


def in_penalty_area(x, y, gx, attack_dir):
    """罚球区（大禁区 80×35）：镜像 C++ in_penalty_area"""
    x_lo = min(gx, gx + attack_dir * 80.0)
    x_hi = max(gx, gx + attack_dir * 80.0)
    return x_lo <= x <= x_hi and 72.5 <= y <= 107.5


def mark_position(opp, opp_vx, opp_vy, bx, by, gx, attack_dir):
    """镜像 C++ run_passive 站位：速度前馈 + 传球线分支 + goal-side + 罚球区推出"""
    gy = 90.0
    px = opp["x"] + opp_vx * MARK_LEAD
    py = opp["y"] + opp_vy * MARK_LEAD
    d_ball = dist(bx, by, opp["x"], opp["y"])
    ref_x, ref_y = gx, gy                      # 默认 goal-side（封射门/再传）
    if MARK_DRIBBLE_DIST < d_ball < MARK_PASS_LANE_DIST:
        ref_x, ref_y = bx, by                  # 堵传球线（接球者且离球够近）
    dx, dy = ref_x - px, ref_y - py
    l = math.hypot(dx, dy)
    if l < 1e-6:
        mx, my = px, py
    else:
        dx, dy = dx / l, dy / l
        mx, my = px + dx * MARK_DIST, py + dy * MARK_DIST
    if in_penalty_area(mx, my, gx, attack_dir):
        mx = gx + attack_dir * PENALTY_PUSH_X
        my = clamp(py, 72.5, 107.5)
    mx = clamp(mx, 0.0, C.FIELD_LENGTH)
    my = clamp(my, 0.0, C.FIELD_WIDTH)
    return mx, my


def pick_mark_target(opp, vx, vy, bx, by, danger, gx, current_target):
    """镜像 C++ pick_mark_target：argmax(mark_threat) + 1.1 滞回 + 危险门限"""
    # 持球者 = 离球最近对手
    dribbler, dmin = -1, 1e9
    for t, op in enumerate(opp):
        db = dist(bx, by, op["x"], op["y"])
        if db < dmin:
            dmin, dribbler = db, t
    # 逐人打分取 argmax
    best, best_score = -1, -1e9
    for t, op in enumerate(opp):
        db = dist(bx, by, op["x"], op["y"])
        dg = abs(op["x"] - gx)
        appr = ball_approach_speed(vx, vy, bx, by, op["x"], op["y"])
        sc = mark_threat(db, dg, appr, danger, t == dribbler)
        if sc > best_score:
            best_score, best = sc, t
    # 滞回：新目标分没超过当前目标 10% 就不换
    if 0 <= current_target < len(opp) and best != current_target:
        t = current_target
        db = dist(bx, by, opp[t]["x"], opp[t]["y"])
        dg = abs(opp[t]["x"] - gx)
        appr = ball_approach_speed(vx, vy, bx, by, opp[t]["x"], opp[t]["y"])
        cur_score = mark_threat(db, dg, appr, danger, t == dribbler)
        if best_score <= cur_score * 1.05:
            best = current_target
    # 危险门限：离球且离门都远 → -1 回区域防守
    if best >= 0:
        db = dist(bx, by, opp[best]["x"], opp[best]["y"])
        dg = abs(opp[best]["x"] - gx)
        if db > MARK_ENGAGE_BALL_DIST and dg > MARK_ENGAGE_GOAL_DIST:
            return -1
    return best


def analyze_side(frames, home_key, opp_key, gx, attack_dir, name):
    """home_key 队的 PASSIVE(index4) 盯 opp_key 队；gx 为己方门线 x，attack_dir 进攻方向"""
    total_def = 0        # 人盯人激活帧（threat>=0.6 且选到目标）
    mark_hit = 0         # 贴住「最威胁」对手的帧数
    mark_any = 0         # 贴住任一对手的帧数
    d_mark_sum = 0.0     # 到正确贴人点平均距离
    d_ball_sum = 0.0     # 到球平均距离（判断在盯人还是追球）

    # 跨帧状态：team_state 滞回 + 人盯人目标滞回
    is_attack = False
    possession_frames = 0
    no_possession_frames = 0
    mark_target = -1

    prev = frames[0]
    prev_opp = frames[0][opp_key]

    for i in range(1, len(frames)):
        f = frames[i]
        bx, by = f["ball"]["x"], f["ball"]["y"]
        vx, vy = bx - prev["ball"]["x"], by - prev["ball"]["y"]
        prev = f

        opp = f[opp_key]
        home = f[home_key]
        h = home[4]

        # 对手速度差分（速度前馈用；每帧更新，保证是相邻帧差分）
        #   单帧位移超 K_MAX_OPP_VEL 视为复位跳变、速度清零（镜像 world_model.cpp）
        opp_vx = [0.0 if abs(opp[t]["x"] - prev_opp[t]["x"]) > K_MAX_OPP_VEL
                  else opp[t]["x"] - prev_opp[t]["x"] for t in range(len(opp))]
        opp_vy = [0.0 if abs(opp[t]["y"] - prev_opp[t]["y"]) > K_MAX_OPP_VEL
                  else opp[t]["y"] - prev_opp[t]["y"] for t in range(len(opp))]
        prev_opp = opp

        # 球权 we_have_ball = 最近自己人比最近对手近 且 <20cm（镜像 situation.cpp）
        our_min = min(dist(bx, by, r["x"], r["y"]) for r in home)
        opp_min = min(dist(bx, by, r["x"], r["y"]) for r in opp)
        we_have_ball = (our_min < opp_min) and (our_min < K_HAS_BALL_DIST)

        # team_state 滞回（镜像 strategy.cpp update_team_state）
        if we_have_ball:
            possession_frames += 1
            no_possession_frames = 0
        else:
            no_possession_frames += 1
            possession_frames = 0
        if is_attack and no_possession_frames >= K_STATE_HYST:
            is_attack = False
        elif (not is_attack) and possession_frames >= K_STATE_HYST:
            is_attack = True

        # threat_level 重建（镜像 strategy.cpp threat_from_state）
        danger = ball_danger_speed(vx, vy, bx, by, gx)
        if is_attack:
            threat = 0.1
        elif in_penalty_area(bx, by, gx, attack_dir):
            threat = 1.0
        else:
            our_half = (bx < 110.0) if attack_dir > 0 else (bx > 110.0)
            threat = 0.6 if our_half else 0.4
            if danger > K_THREAT_DANGER_SPEED:
                threat = 0.8 if our_half else 0.6

        if threat < 0.6:
            continue   # 非人盯人帧：PASSIVE 在做区域防守，不计入贴住率

        # 选目标（镜像 pick_mark_target，含滞回）
        mark_target = pick_mark_target(opp, vx, vy, bx, by, danger, gx, mark_target)
        if mark_target < 0:
            continue   # 无危险目标，PASSIVE 回区域

        total_def += 1
        mx, my = mark_position(opp[mark_target], opp_vx[mark_target], opp_vy[mark_target],
                               bx, by, gx, attack_dir)
        d_mark = dist(h["x"], h["y"], mx, my)
        d_ball = dist(h["x"], h["y"], bx, by)
        d_mark_sum += d_mark
        d_ball_sum += d_ball
        if d_mark < NEAR_TOL:
            mark_hit += 1
        for op in opp:
            if 5.0 <= dist(h["x"], h["y"], op["x"], op["y"]) <= 25.0:
                mark_any += 1
                break

    if total_def == 0:
        print(f"[{name}] 无人盯人激活帧（threat 从未 >=0.6 或从未选到目标）")
        return
    print(f"\n===== {name} 人盯人复盘（仅 threat>=0.6 帧）=====")
    print(f"  人盯人激活帧        : {total_def}")
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

    # 蓝队（守 x=220 右门）PASSIVE=blue[4] 盯 yellow；attack_dir=-1
    analyze_side(frames, "blue", "yellow", 220.0, -1.0, "蓝队 PASSIVE(blue[4]) 盯黄")
    # 黄队（守 x=0 左门）PASSIVE=yellow[4] 盯 blue；attack_dir=+1
    analyze_side(frames, "yellow", "blue", 0.0, +1.0, "黄队 PASSIVE(yellow[4]) 盯蓝")


if __name__ == "__main__":
    main()
