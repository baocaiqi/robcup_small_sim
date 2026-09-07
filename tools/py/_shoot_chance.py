# -*- coding: utf-8 -*-
"""
_shoot_chance.py — 量化「后卫给对方留射门机会」程度。

对一场 .rlg 逐帧统计：当对方持球者带球进入我方防守三区（离门 1/3 场内）时，
离他最近的我方防守者（非门将）有多远。距离越大 = 越没人逼抢 = 越大的射门空档。

用法：python _shoot_chance.py 比赛.rlg
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import rlg_analyzer as R

DRIBBLE_HOLD = 15.0   # 持球者判定：离球 <15cm
THIRD = C.FIELD_LENGTH / 3.0   # 防守三区深度 = 220/3 ≈ 73.3cm


def dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def analyze(frames, home_key, opp_key, gx, attack_dir, name):
    # 我方非门将防守者 = home[1..4]；门将 = home[0]
    defenders = range(1, 5)
    in_third = 0          # 对方持球者进入我方防守三区的帧数
    open_shots = 0        # 其中「最近防守者 > 25cm」的帧数（= 无人逼抢）
    dist_sum = 0.0        # 最近防守者距离累计
    # 分桶：<15 贴身 / 15-25 逼抢 / 25-40 松 / >40 大空档
    bucket = [0, 0, 0, 0]

    for i in range(1, len(frames)):
        f = frames[i]
        bx, by = f["ball"]["x"], f["ball"]["y"]
        opp = f[opp_key]
        home = f[home_key]

        # 对方持球者 = 离球最近
        dmin, dribbler = 1e9, -1
        for t, o in enumerate(opp):
            d = dist(bx, by, o["x"], o["y"])
            if d < dmin:
                dmin, dribbler = d, t
        if dribbler < 0 or dmin >= DRIBBLE_HOLD:
            continue   # 球不在对方脚下

        ox, oy = opp[dribbler]["x"], opp[dribbler]["y"]
        # 是否在我方防守三区
        if attack_dir > 0:
            in3 = ox < THIRD
        else:
            in3 = ox > C.FIELD_LENGTH - THIRD
        if not in3:
            continue

        # 离他最近的我方非门将防守者
        nearest = 1e9
        for j in defenders:
            d = dist(ox, oy, home[j]["x"], home[j]["y"])
            nearest = min(nearest, d)

        in_third += 1
        dist_sum += nearest
        if nearest > 25.0:
            open_shots += 1
        if nearest < 15.0:
            bucket[0] += 1
        elif nearest < 25.0:
            bucket[1] += 1
        elif nearest < 40.0:
            bucket[2] += 1
        else:
            bucket[3] += 1

    print(f"\n===== {name} 射门空档统计 =====")
    if in_third == 0:
        print("  对方持球者从未进入我方防守三区")
        return
    print(f"  持球者进防守三区帧数 : {in_third}")
    print(f"  无人逼抢(>25cm)帧数  : {open_shots}  ({open_shots/in_third*100:5.1f}%)")
    print(f"  平均最近防守者距离   : {dist_sum/in_third:5.1f} cm")
    print(f"  分布  <15cm贴身 : {bucket[0]/in_third*100:5.1f}%  "
          f"15-25逼抢:{bucket[1]/in_third*100:5.1f}%  "
          f"25-40松:{bucket[2]/in_third*100:5.1f}%  "
          f">40大空档:{bucket[3]/in_third*100:5.1f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg")
    args = ap.parse_args()
    frames = R.parse_rlg(args.rlg)

    # 蓝队守 x=220 右门（attack_dir=-1），统计蓝队防守时对方(黄)的射门空档
    analyze(frames, "blue", "yellow", 220.0, -1.0, "蓝队后卫盯黄队(我方 vs demo)")


if __name__ == "__main__":
    main()
