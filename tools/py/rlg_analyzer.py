# -*- coding: utf-8 -*-
"""
rlg_analyzer.py — .rlg 比赛日志解析与复盘统计

用法：
    python rlg_analyzer.py 比赛日志.rlg [--frame N] [--csv out.csv]

功能：
    1. 解析二进制日志（352B/帧：蓝队5机 + 黄队5机 + 球 + gameState + whosBall）
    2. 输出比赛统计：总帧数/时长、各状态占比、球权分布、进球事件
    3. --frame N 打印第 N 帧全场快照（调试策略时核对坐标）
    4. --csv 导出每帧球轨迹 CSV（可用 Excel 画图看跑位）
"""
import argparse
import csv
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C


def parse_rlg(path):
    """返回 frames: list[dict]"""
    data = open(path, "rb").read()
    n = len(data) // C.FRAME_BYTES
    frames = []
    fmt = "<" + "d" * 6 + "d"  # 每个 LogRobot: pos(x,y,z)+rotation = 4 doubles
    # 实际每机器人 32B = 4 doubles；每帧 352B = 11*32 = 44 doubles
    fmt_all = "<" + "d" * 44
    for i in range(n):
        chunk = data[i * C.FRAME_BYTES:(i + 1) * C.FRAME_BYTES]
        vals = struct.unpack(fmt_all, chunk)
        idx = 0
        blue = []
        for _ in range(5):
            x, y, z, rot = vals[idx:idx + 4]
            idx += 4
            blue.append(dict(x=x + C.LOG_OFFSET_X, y=y, rot=rot))
        yellow = []
        for _ in range(5):
            x, y, z, rot = vals[idx:idx + 4]
            idx += 4
            yellow.append(dict(x=x + C.LOG_OFFSET_X, y=y, rot=rot))
        ball = dict(x=vals[idx] + C.LOG_OFFSET_X, y=vals[idx + 1])
        gs = int(vals[idx + 2])
        whos = int(vals[idx + 3])
        frames.append(dict(blue=blue, yellow=yellow, ball=ball, gs=gs, whos=whos))
    return frames


def frame_snapshot(f, i):
    """打印第 i 帧全场快照"""
    print(f"===== 帧 {i} =====  状态={C.PLAY_MODE_NAMES.get(f['gs'], f['gs'])}  "
          f"球权={C.WHOS_BALL_NAMES.get(f['whos'], f['whos'])}")
    print(f"球   : ({f['ball']['x']:.1f}, {f['ball']['y']:.1f})")
    for team, label in ((f['blue'], "蓝"), (f['yellow'], "黄")):
        for j, r in enumerate(team):
            print(f"{label}{j}: ({r['x']:.1f},{r['y']:.1f}) rot={r['rot']:.0f}")


def report(frames, path):
    """统计报告"""
    n = len(frames)
    gs_counts = {}
    whos_counts = {}
    goals = []          # 进球事件（去重：连续帧同一事件只记一次，球回场后才计下一球）
    last_tag = None
    for i, f in enumerate(frames):
        gs_counts[f["gs"]] = gs_counts.get(f["gs"], 0) + 1
        whos_counts[f["whos"]] = whos_counts.get(f["whos"], 0) + 1
        bx, by = f["ball"]["x"], f["ball"]["y"]
        # 蓝队守 x=220 门，黄队守 x=0 门（日志坐标系已偏移到 0..220）
        if bx > 220.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            tag = "黄队得分(破蓝门)"
        elif bx < 0.0 and C.GOAL_Y_LOW <= by <= C.GOAL_Y_HIGH:
            tag = "蓝队得分(破黄门)"
        else:
            tag = None
        # 同一事件连续帧只记一次（球进网后平台立刻回中圈，下一球必须重新越线）
        if tag is not None and tag != last_tag:
            goals.append((i, tag))
        last_tag = tag

    print(f"===== 比赛统计: {os.path.basename(path)} =====")
    print(f"总帧数: {n}   约 {n / 40:.1f} 秒 (按40Hz；rlg 常只录部分时段，进球数可能少于整场)")
    print("\n比赛状态分布:")
    for gs in sorted(gs_counts):
        print(f"  {C.PLAY_MODE_NAMES.get(gs, gs):<18} {gs_counts[gs]:>6} 帧 ({gs_counts[gs]/n*100:4.1f}%)")
    print("\n球权分布:")
    for w in sorted(whos_counts):
        print(f"  {C.WHOS_BALL_NAMES.get(w, w):<6} {whos_counts[w]:>6} 帧 ({whos_counts[w]/n*100:4.1f}%)")
    print("\n进球事件（去重）:")
    if goals:
        for i, desc in goals:
            print(f"  帧 {i} (~{i/40:.1f}s): {desc}")
        yellow_goals = sum(1 for _, d in goals if d.startswith("黄队得分"))
        blue_goals = sum(1 for _, d in goals if d.startswith("蓝队得分"))
        # 与官方 SimuroSot5.log 口径一致：log 比分 = (黄 : 蓝)，首位=黄队（守左门 x=0）
        print(f"  比分 黄 {yellow_goals} : {blue_goals} 蓝  (与官方 log (黄:蓝) 口径一致)")
    else:
        print("  （未检测到进球，可调帧率或检查坐标偏移）")


def export_csv(frames, path):
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["frame", "gs", "whos", "ball_x", "ball_y",
                    "b0_x", "b0_y", "b1_x", "b1_y", "b2_x", "b2_y", "b3_x", "b3_y", "b4_x", "b4_y",
                    "y0_x", "y0_y", "y1_x", "y1_y", "y2_x", "y2_y", "y3_x", "y3_y", "y4_x", "y4_y"])
        for i, f in enumerate(frames):
            row = [i, f["gs"], f["whos"], f["ball"]["x"], f["ball"]["y"]]
            for r in f["blue"]:
                row += [r["x"], r["y"]]
            for r in f["yellow"]:
                row += [r["x"], r["y"]]
            w.writerow(row)


def main():
    ap = argparse.ArgumentParser(description="FIRA 仿真 5v5 .rlg 日志分析")
    ap.add_argument("rlg", help=".rlg 日志文件")
    ap.add_argument("--frame", type=int, help="打印指定帧的全场快照")
    ap.add_argument("--csv", help="导出球/机器人轨迹 CSV")
    args = ap.parse_args()

    frames = parse_rlg(args.rlg)
    if args.frame is not None:
        if not 0 <= args.frame < len(frames):
            print(f"帧 {args.frame} 越界（共 {len(frames)} 帧）")
            return 1
        frame_snapshot(frames[args.frame], args.frame)
    if args.csv:
        export_csv(frames, args.csv)
        print(f"已导出 {args.csv}")
    report(frames, args.rlg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
