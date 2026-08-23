# -*- coding: utf-8 -*-
"""
constants.py — FIRA 仿真 5v5 场地常量
（与 C++ 端 include/simuro5/field_info.hpp 保持一致）
坐标系：原点场地左下角 (0,0)，x 向右，y 向上，单位 cm，角度制。
"""
import math

# 场地
FIELD_LENGTH = 220.0   # x 方向
FIELD_WIDTH = 180.0    # y 方向
CENTER_RADIUS = 25.0   # 中圈半径

# 球门（位于球门线中点 y=90 两侧 ±20）
GOAL_WIDTH = 40.0
GOAL_Y_MID = 90.0
GOAL_Y_LOW = GOAL_Y_MID - GOAL_WIDTH / 2.0   # 70
GOAL_Y_HIGH = GOAL_Y_MID + GOAL_WIDTH / 2.0  # 110

# 门区（小禁区 A）：球门前 50×15
GOAL_AREA_DEPTH = 50.0
GOAL_AREA_HALF_W = 15.0
# 罚球区（大禁区 A+B）：球门前 80×35
PENALTY_DEPTH = 80.0
PENALTY_HALF_W = 35.0

# 点球判罚区 120×80、争球点（1/4 场中心）
PENALTY_JUDGE_W = 120.0
PENALTY_JUDGE_D = 80.0
FREEBALL_POINTS = {
    1: (55.0, 135.0),   # PM_FreeBall_LeftTop
    2: (55.0, 45.0),    # PM_FreeBall_LeftBot
    3: (165.0, 135.0),  # PM_FreeBall_RightTop
    4: (165.0, 45.0),   # PM_FreeBall_RightBot
}

# PlayMode（与 C++ simuro_interface.hpp 一致）
PLAY_MODE_NAMES = {
    0: "PlayOn", 1: "FreeBall_LeftTop", 2: "FreeBall_LeftBot",
    3: "FreeBall_RightTop", 4: "FreeBall_RightBot",
    5: "PlaceKick_Yellow", 6: "PlaceKick_Blue",
    7: "PenaltyKick_Yellow", 8: "PenaltyKick_Blue",
    9: "FreeKick_Yellow", 10: "FreeKick_Blue",
    11: "GoalKick_Yellow", 12: "GoalKick_Blue",
}
WHOS_BALL_NAMES = {0: "未知", 1: "蓝队", 2: "黄队"}

# .rlg 日志帧结构（二进制，来自平台作者 README）
# LogRobot = pos(x,y,z) + rotation = 32B；LogEnvironment = 蓝5 + 黄5 + 球 + gs + whosBall = 352B
FRAME_BYTES = 352
LOG_ROBOT_BYTES = 32
LOG_OFFSET_X = 0.0   # 日志坐标系与策略坐标系一致（2023 版 SimuroSot5 平台原生 [0,220]：
                      # 官方 demo Strategy4Blue 门将 x=215、Strategy4Yellow 门将 x=5，
                      # .rlg raw 蓝门将≈214.8/黄门将≈5.1，与策略输出 210/10 完全吻合，无需偏移）
