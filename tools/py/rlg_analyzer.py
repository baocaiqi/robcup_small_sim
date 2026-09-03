# -*- coding: utf-8 -*-
"""
rlg_analyzer.py — .rlg 比赛日志解析与复盘统计

用法：
    python rlg_analyzer.py 比赛日志.rlg [--frame N] [--csv out.csv] [--official SimuroSot5.log]

功能：
    1. 解析二进制日志（352B/帧：蓝队5机 + 黄队5机 + 球 + gameState + whosBall）
    2. 输出比赛统计：总帧数/时长、各状态占比、球权分布、进球事件
    3. --frame N 打印第 N 帧全场快照（调试策略时核对坐标）
    4. --csv 导出每帧球轨迹 CSV（可用 Excel 画图看跑位）
    5. --official 官方 log：自动按文件名时间匹配场次，输出权威比分并与 rlg 判定
       对照（✅一致 / ⚠️不一致以官方为准）

⚠️ 重要：rlg 以 40Hz 采样球位，官方判进球在物理引擎内按"球整体过门线"判定，
   rlg 会（a）漏掉采样间隙的最深越线帧（真进球但 rlg 只见 x≈221~223），
   （b）把擦线/反弹球（x≈220~223）误报为进球。故【比分一律以官方 log 为准】，
   rlg 只用于站位/轨迹/丢球过程分析。
"""
import argparse
import csv
import os
import re
import struct
import sys
from datetime import datetime

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


def parse_official_log(log_path):
    """解析官方 SimuroSot5.log，返回 [(场次开始 datetime, 终局比分 "黄:蓝"), ...]。

    口径：log 比分 (a : b) = (黄 : 蓝)，Place Kick X = X 队开球 = 对方刚进球。
    每场由 "Starting Controller" 分段；开场 Place Kick（Time≥285 且 0:0）不算进球；
    终局比分取该场最后一个非开场的 Place Kick 事件（Time 最小者）。
    """
    entries = []          # (datetime, "黄:蓝")
    cur_start = None
    cur_pk = []           # [(time_left, team, a, b)]
    pat_ctrl = re.compile(r"Starting Controller at (\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2}):(\d{2})")
    pat_pk = re.compile(r"Place Kick (\w+)\s+--\s+\((\d+)\s*:\s*(\d+)\)\s+Time\s*:\s*(\d+)")

    def flush():
        nonlocal cur_start, cur_pk
        if cur_start is None:
            return
        meaningful = [e for e in cur_pk if not (e[0] >= 285 and e[2] == 0 and e[3] == 0)]
        if meaningful:
            latest = min(meaningful, key=lambda x: x[0])
            score = "{}:{}".format(latest[2], latest[3])
        else:
            score = "0:0"
        entries.append((cur_start, score))
        cur_pk = []

    with open(log_path, encoding="utf-8", errors="replace") as fp:
        for raw in fp:
            ln = raw.strip()
            m = pat_ctrl.search(ln)
            if m:
                flush()
                y, mo, d, h, mi, s = (int(g) for g in m.groups())
                cur_start = datetime(y, mo, d, h, mi, s)
                continue
            if cur_start is None:
                continue
            m = pat_pk.search(ln)
            if m:
                team = m.group(1)
                a, b, t = int(m.group(2)), int(m.group(3)), int(m.group(4))
                cur_pk.append((t, team, a, b))
    flush()
    return entries


def match_official(rlg_path, entries):
    """按 rlg 文件名的 14 位时间戳匹配官方场次（差 >15 分钟视为找不到）。"""
    base = os.path.basename(rlg_path)
    m = re.match(r"(\d{4})(\d{2})(\d{2})(\d{2})(\d{2})(\d{2})", base)
    if not m:
        return None
    y, mo, d, h, mi, s = (int(g) for g in m.groups())
    try:
        rlg_dt = datetime(y, mo, d, h, mi, s)
    except ValueError:
        return None
    if not entries:
        return None
    best = min(entries, key=lambda e: abs((e[0] - rlg_dt).total_seconds()))
    if abs((best[0] - rlg_dt).total_seconds()) > 15 * 60:
        return None
    return best


def report(frames, path, official_score=None, official_time=None):
    """统计报告。official_score: "黄:蓝"（来自官方 log，权威比分）；official_time: 场次开始时间"""
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
    if official_time is not None:
        print(f"官方场次: {official_time.strftime('%Y-%m-%d %H:%M:%S')}  (rlg 按文件名时间戳匹配)")
    print("\n比赛状态分布:")
    for gs in sorted(gs_counts):
        print(f"  {C.PLAY_MODE_NAMES.get(gs, gs):<18} {gs_counts[gs]:>6} 帧 ({gs_counts[gs]/n*100:4.1f}%)")
    print("\n球权分布:")
    for w in sorted(whos_counts):
        print(f"  {C.WHOS_BALL_NAMES.get(w, w):<6} {whos_counts[w]:>6} 帧 ({whos_counts[w]/n*100:4.1f}%)")

    # rlg 判定（仅供参考——40Hz 采样会漏最深越线帧，也会误报擦线球）
    yellow_goals = sum(1 for _, d in goals if d.startswith("黄队得分"))
    blue_goals = sum(1 for _, d in goals if d.startswith("蓝队得分"))

    print("\n进球事件（rlg 判定，去重）:")
    if goals:
        for i, desc in goals:
            print(f"  帧 {i} (~{i/40:.1f}s): {desc}")
        print(f"  [rlg] 黄 {yellow_goals} : {blue_goals} 蓝")
    else:
        print("  （rlg 未检测到越线事件）")

    # 官方比分对照（权威）——口径：log 比分 = (黄 : 蓝)，首位=黄队(守左门 x=0)
    mark = ""
    if official_score is not None:
        oy, ob = (int(v) for v in official_score.split(":"))
        if oy == yellow_goals and ob == blue_goals:
            mark = "  ✅ 与官方一致"
        else:
            mark = "  ⚠️ 与官方不一致 → 以官方为准（rlg 采样/擦线误报）"
        print(f"  [官方] 黄 {oy} : {ob} 蓝{mark}")
    elif goals:
        print("  （未提供 --official，此比分仅供参考，请以 C:\\Strategy\\SimuroSot5.log 为准）")


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
    ap.add_argument("--official", metavar="SimuroSot5.log", nargs="?",
                    const=r"C:\Strategy\SimuroSot5.log", default=None,
                    help="官方 log 路径（默认查找 C:\\Strategy\\SimuroSot5.log）；"
                         "自动按文件名时间匹配场次并输出权威比分对照，比分以官方为准")
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

    official_score, official_time = None, None
    if args.official is not None:
        log_path = args.official
        if not os.path.isfile(log_path):
            print(f"⚠️ 官方 log 不存在: {log_path}（跳过对照）")
        else:
            entries = parse_official_log(log_path)
            hit = match_official(args.rlg, entries)
            if hit is None:
                print(f"⚠️ 官方 log 中未找到与 {os.path.basename(args.rlg)} 匹配的场次（跳过对照）")
            else:
                official_time, official_score = hit
    report(frames, args.rlg, official_score=official_score, official_time=official_time)
    return 0


if __name__ == "__main__":
    sys.exit(main())
