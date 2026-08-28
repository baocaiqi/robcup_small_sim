# -*- coding: utf-8 -*-
"""临时：进球/乌龙球复盘分析（去重后的进球事件 + 最后触球者判定）"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
from rlg_analyzer import parse_rlg

TOUCH_DIST = 8.0    # 判定「机器人碰到球」的中心距(cm)
LOOKBACK = 120      # 回找最后触球的最大帧数（40Hz 下 3s）

def goal_events(frames):
    """去重：把连续多帧「球越线」合并成单个进球事件，返回 (首帧下标, side)"""
    events = []
    prev = None  # (side, last_frame)
    for i, f in enumerate(frames):
        bx, by = f['ball']['x'], f['ball']['y']
        side = None
        if bx > 220.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            side = 'blue'
        elif bx < 0.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            side = 'yellow'
        if side is None:
            prev = None
            continue
        if prev is not None and prev[0] == side and i - prev[1] <= 3:
            prev = (side, i)  # 同一事件延续
        else:
            events.append([i, side])
            prev = (side, i)
    return events

def last_touch(frames, i):
    """从 i 帧往前找最后一个碰球的机器人，返回 (team, idx, ball_x, ball_y) 或 None"""
    for j in range(i - 1, max(-1, i - LOOKBACK - 1), -1):
        fg = frames[j]
        bx, by = fg['ball']['x'], fg['ball']['y']
        best = (1e9, None, None)
        for team, label in ((fg['blue'], 'blue'), (fg['yellow'], 'yellow')):
            for k, r in enumerate(team):
                d = ((r['x'] - bx) ** 2 + (r['y'] - by) ** 2) ** 0.5
                if d < best[0]:
                    best = (d, label, k)
        if best[0] < TOUCH_DIST:
            return best[1], best[2], bx, by
    return None

def report(path):
    frames = parse_rlg(path)
    events = goal_events(frames)
    print(f"\n{'='*64}")
    print(f"文件: {os.path.basename(path)}   总帧 {len(frames)} (~{len(frames)/40:.0f}s)")
    print(f"{'='*64}")
    blue_lost = yellow_lost = 0
    own = 0
    clean = 0
    unid = 0
    for i, side in events:
        t = last_touch(frames, i)
        defname = '蓝' if side == 'blue' else '黄'
        if side == 'blue':
            blue_lost += 1
        else:
            yellow_lost += 1
        if t is None:
            unid += 1
            print(f"  帧{i:>5} ({i/40:>5.1f}s)  {defname}队失球  ← 未识别最后触球(超{LOOKBACK}帧远射/定位球)")
            continue
        tt, ti, sx, sy = t
        tname = '蓝' if tt == 'blue' else '黄'
        is_own = (tt == side)
        own += is_own
        clean += (not is_own)
        tag = '【乌龙】' if is_own else '【正常进球】'
        # 触球点距失球方球门线距离
        glx = 220.0 if side == 'blue' else 0.0
        dist_goal = abs(sx - glx)
        print(f"  帧{i:>5} ({i/40:>5.1f}s)  {defname}队失球  ← 最后触球 {tname}{ti} "
              f"触球点({sx:.0f},{sy:.0f}) 距门线{dist_goal:.0f}cm  {tag}")
    n = len(events)
    print(f"\n  比分: 蓝 {yellow_lost} : {blue_lost} 黄   （蓝守右门x=220 / 黄守左门x=0）")
    print(f"  进球事件 {n} 个：乌龙 {own}，正常 {clean}，未识别 {unid}")
    if own + clean > 0:
        print(f"  可识别进球中乌龙占比 {own/(own+clean)*100:.0f}%")
    return own, clean, unid

if __name__ == '__main__':
    for p in sys.argv[1:]:
        report(p)
