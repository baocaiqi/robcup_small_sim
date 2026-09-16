# -*- coding: utf-8 -*-
"""analyze_matches.py — 真机复盘：官方比分/判罚 + 我方门区纪律（只读，不改任何东西）。

回答三个问题：
  ① 每场比分与**双方被判点球次数**（官方日志，权威）
  ② 我方"门区挤 2 人以上"的帧数与次数 —— 这是被判点球的根因（目标：不劣化）
  ③ 球位分布 / 球权 / 停表（争球重置）次数 —— 判断我们是被压着打还是压着对方

数据来源：
  · C:\\Strategy\\SimuroSot5.log   官方事件日志（比分、点球、开球；注意比分格式是 **黄:蓝**）
  · C:\\Strategy\\hnnu_blackbox.csv 我们的黑匣子（F=帧摘要 R=我方5机 O=对方5机 P=摆位 B=球）

用法：
    python tools\\py\\analyze_matches.py                 # 官方日志分场 + 黑匣子分 session
    python tools\\py\\analyze_matches.py --sessions 8     # 只看最近 8 个 session
"""
import argparse
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

OFFICIAL = r"C:\Strategy\SimuroSot5.log"
BLACKBOX = r"C:\Strategy\hnnu_blackbox.csv"
# 我方球门 / 门区 / 禁区（我们守 x=220）
OUR_GOAL_X = 220.0
GA_X, GA_Y1, GA_Y2 = 190.0, 75.0, 105.0     # 门区 50×30
PA_X, PA_Y1, PA_Y2 = 140.0, 72.5, 107.5     # 禁区 80×35


def parse_official(path):
    """按 'Starting Controller at <时间>' 切场次（官方日志里唯一的场次边界）。

    返回 [(时间串, 黄分, 蓝分, 我方被判点球数, 对方被判点球数, 事件数)]
    注意：'Penalty Kick Yellow' = **黄队主罚** ⇒ 我方犯规（我们守蓝侧）。
    """
    if not os.path.exists(path):
        return []
    lines = open(path, encoding="gbk", errors="replace").read().splitlines()
    ev = re.compile(r"(Penalty Kick|Place Kick|Goal Kick|Free Kick|Corner Kick)\s+(Yellow|Blue)"
                    r".*?\((\d+)\s*:\s*(\d+)\)")
    starts = [(i, m.group(1)) for i, l in enumerate(lines)
              for m in [re.match(r"Starting Controller at (.+)", l.strip())] if m]
    out = []
    for k, (idx, ts) in enumerate(starts):
        end = starts[k + 1][0] if k + 1 < len(starts) else len(lines)
        ys = bs = py = pb = n = 0
        for l in lines[idx:end]:
            m = ev.search(l)
            if not m:
                continue
            kind, who, y, b = m.group(1), m.group(2), int(m.group(3)), int(m.group(4))
            ys, bs = max(ys, y), max(bs, b)
            n += 1
            if kind == "Penalty Kick":
                if who == "Yellow":
                    py += 1
                else:
                    pb += 1
        out.append((ts.strip(), ys, bs, py, pb, n))
    return out


def read_blackbox(path, max_sessions):
    """按 '# ---- new session ----' 切分；返回 [session] ，每个 session 是 list[行]"""
    if not os.path.exists(path):
        return []
    sessions, cur = [], None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("# ---- new session"):
                if cur:
                    sessions.append(cur)
                cur = []
            elif cur is not None:
                cur.append(line)
    if cur:
        sessions.append(cur)
    return sessions[-max_sessions:]


def session_stats(rows):
    """统计一个 session：门区挤人、禁区挤人、球位分布、球权、停表"""
    ga_frames = ga_eps = pa_frames = 0
    opp_ga_frames = opp_ga_eps = 0        # 我方攻入**对方门区**挤 2 人以上（也是判罚来源！）
    prev_ga = prev_pa = prev_oga = False
    poss_us = poss_them = 0
    zone = [0, 0, 0]          # 我方三区 / 中场 / 对方三区（按球 x）
    frames_n = 0
    last_frame = 0
    deadball = 0
    prev_bx = prev_by = None
    blue = {}
    yellow = {}
    for line in rows:
        p = line.rstrip("\n").split(",")
        if not p:
            continue
        t = p[0]
        if t == "F":
            frames_n += 1
            last_frame = max(last_frame, int(p[1]))
            try:
                bx, by = float(p[4]), float(p[5])
            except (ValueError, IndexError):
                continue
            # 球位分三区（我们守 x=220，攻 x=0）
            zone[0 if bx > 146 else (1 if bx > 74 else 2)] += 1
            # 停表/争球重置：球位突然跳回中圈附近
            if prev_bx is not None:
                jump = ((bx - prev_bx) ** 2 + (by - prev_by) ** 2) ** 0.5
                if jump > 40:
                    deadball += 1
            prev_bx, prev_by = bx, by
        elif t in ("R", "O"):
            # R 行格式：R,<帧>,<isBlue>, 然后 5 组 (x,y,rot,role)
            # O 行格式：O,<帧>,           然后 5 组 (x,y,rot)
            if t == "R" and len(p) >= 23:
                vals = p[3:]
                d = blue
                step = 4
            elif t == "O" and len(p) >= 17:
                vals = p[2:]
                d = yellow
                step = 3
            else:
                continue
            pts = []
            step = 4 if t == "R" else 3
            for i in range(0, len(vals) - step + 1, step):
                try:
                    pts.append((float(vals[i]), float(vals[i + 1])))
                except ValueError:
                    break
            if not pts:
                continue
            d["pts"] = pts
            if t == "R":
                # 门区/禁区挤人（不含门将 0 号，与 sim 口径一致）
                in_ga = sum(1 for (x, y) in pts[1:] if x >= GA_X and GA_Y1 <= y <= GA_Y2)
                in_pa = sum(1 for (x, y) in pts[1:] if x >= PA_X and PA_Y1 <= y <= PA_Y2)
                # 镜像：对方门区（我们攻 x=0）——攻方挤在人家门区同样会被判罚
                in_oga = sum(1 for (x, y) in pts[1:] if x <= 30.0 and GA_Y1 <= y <= GA_Y2)
                ga = in_ga >= 2
                pa = in_pa >= 2
                oga = in_oga >= 2
                ga_frames += 1 if ga else 0
                pa_frames += 1 if pa else 0
                opp_ga_frames += 1 if oga else 0
                if ga and not prev_ga:
                    ga_eps += 1
                if oga and not prev_oga:
                    opp_ga_eps += 1
                prev_ga, prev_pa, prev_oga = ga, pa, oga
                # 球权：球离最近我方机器人 <15cm
                if prev_bx is not None:
                    if min((((prev_bx - x) ** 2 + (prev_by - y) ** 2) ** 0.5) for (x, y) in pts) < 15.0:
                        poss_us += 1
            else:
                if prev_bx is not None:
                    if min((((prev_bx - x) ** 2 + (prev_by - y) ** 2) ** 0.5) for (x, y) in pts) < 15.0:
                        poss_them += 1
    tot_z = max(sum(zone), 1)
    tot_p = max(poss_us + poss_them, 1)
    return dict(frames=frames_n, seconds=last_frame / 40.0,
                ga_frames=ga_frames, ga_eps=ga_eps, pa_frames=pa_frames,
                opp_ga_frames=opp_ga_frames, opp_ga_eps=opp_ga_eps,
                zone=[100.0 * z / tot_z for z in zone],
                poss_us=100.0 * poss_us / tot_p, deadball=deadball)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", type=int, default=8)
    a = ap.parse_args()

    print("=" * 78)
    print("① 官方日志：每场比分与双方被判点球（权威口径，比分格式 = 黄 : 蓝）")
    print("=" * 78)
    ms = parse_official(OFFICIAL)
    if not ms:
        print("  读不到官方日志")
    print(f"{'时间':>20} {'黄:蓝':>8} {'结果':>7} {'我方被判点球':>12} {'对方被判点球':>12} {'事件数':>6}")
    for ts, y, b, py, pb, n in ms[-10:]:
        res = "我们赢" if b > y else ("输" if b < y else "平")
        print(f"{ts:>20} {y:>4} : {b:<3} {res:>7} {py:>12} {pb:>12} {n:>6}")

    print()
    print("=" * 78)
    print(f"② 黑匣子：最近 {a.sessions} 个 session 的纪律与球位（口径同 sim：门区不含门将）")
    print("=" * 78)
    ss = read_blackbox(BLACKBOX, a.sessions)
    if not ss:
        print("  读不到黑匣子 CSV")
        return 1
    print(f"{'session':>8} {'时长s':>7} {'自家门区2+人':>12} {'次数':>5} {'对方门区2+人':>12} {'次数':>5} "
          f"{'球在我方%':>9} {'中场%':>7} {'在对方%':>8} {'我方控球%':>9} {'球位移>40cm':>11}")
    for i, rows in enumerate(ss, 1):
        st = session_stats(rows)
        if st["frames"] == 0:
            continue
        print(f"{i:>8} {st['seconds']:>7.0f} {st['ga_frames']:>10} {st['ga_eps']:>5} "
              f"{st['opp_ga_frames']:>12} {st['opp_ga_eps']:>5} "
              f"{st['pa_frames']:>10} {st['zone'][0]:>9.1f} {st['zone'][1]:>7.1f} "
              f"{st['zone'][2]:>8.1f} {st['poss_us']:>9.1f} {st['deadball']:>11}")
    print("\n（自家门区2+人 / 对方门区2+人 = 除门将外有 ≥2 人同时待在该门区；两者都是被判罚的来源）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
