# -*- coding: utf-8 -*-
"""
geometry.py — 几何工具（移植自中型组 nubot_strategy_py/geometry.py）
单位：cm / 度
"""
import math


def normalize_angle(a):
    """归一化到 (-180, 180]"""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def angle_diff(a, b):
    return normalize_angle(a - b)


def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def angle_to(x1, y1, x2, y2):
    return normalize_angle(math.degrees(math.atan2(y2 - y1, x2 - x1)))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    if len2 < 1e-9:
        return dist(px, py, ax, ay)
    t = ((px - ax) * dx + (py - ay) * dy) / len2
    t = clamp(t, 0.0, 1.0)
    return dist(px, py, ax + t * dx, ay + t * dy)


def in_rect(px, py, left, right, bottom, top):
    return left <= px <= right and bottom <= py <= top
