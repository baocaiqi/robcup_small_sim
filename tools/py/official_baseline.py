# -*- coding: utf-8 -*-
"""
official_baseline.py — 官方 SimuroSot5.log 战绩统计（口径勘误版）

⚠️ 2026-08-29 勘误：官方 log 比分格式 = **(黄 : 蓝)**（首位=黄队=守左门 x=0），
   进球后失球方开球（Place Kick X = X 队刚失球）。此前把首位当蓝队导致
   "11 胜 2 负 2 平、2.7:0.8 赢 demo" 的错误结论（真实是 2 胜 1 平 11 负）。

用法：
    python official_baseline.py [C:\\Strategy\\SimuroSot5.log] [--rlg-dir C:\\Strategy]

输出：
    1. 每场比赛（Starting Controller 会话段）：开始时间、终场比分 (黄:蓝)、
       判罚统计（Penalty Kick Blue/Yellow = 蓝/黄被罚点球次数）、完整度
    2. 汇总：完整场次胜负平（黄:蓝 口径）、平均比分、点球判罚场均
"""
import argparse
import glob
import os
import re
import sys

# 段起始标记
CTRL_RE = re.compile(r"Starting Controller at ([\d\-]+ [\d:]+)")
EVENT_RE = re.compile(r"^(.*?)\s*--\s*\((\d+)\s*:\s*(\d+)\)\s*Time\s*:\s*([\d.]+)")
GAME_RE = re.compile(r"Game started!\s+\*+\s*([\d\-]+ [\d:]+)")


def parse_log(path):
    """按 Starting Controller 分段，返回 list[dict]"""
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    segs = []
    cur = None
    for ln in lines:
        m = CTRL_RE.search(ln)
        if m:
            cur = dict(start=m.group(1), events=[], game_started=None)
            segs.append(cur)
            continue
        if cur is None:
            continue
        g = GAME_RE.search(ln)
        if g:
            cur["game_started"] = g.group(1)
            continue
        ev = EVENT_RE.search(ln.strip())
        if ev:
            cur["events"].append(dict(
                name=ev.group(1).strip(),
                yellow=int(ev.group(2)),   # 首位 = 黄队（守左门 x=0）
                blue=int(ev.group(3)),     # 次位 = 蓝队（守右门 x=220）
                time=float(ev.group(4)),
            ))
    return segs


def team_name_from_rlg(rlg_dir, start_dt):
    """按比赛开始时间从 rlg 文件名推断队名（文件名: YYYYMMDDHHMMSS-5-黄队名-蓝队名.rlg）"""
    if not rlg_dir:
        return None, None
    for p in sorted(glob.glob(os.path.join(rlg_dir, "*.rlg")), reverse=True):
        base = os.path.basename(p)
        m = re.match(r"(\d{14})-5-(.*)-(.*)\.rlg$", base)
        if not m:
            continue
        stamp = m.group(1)
        # 比赛开始时间戳与 log 会话时间接近（±5 分钟）
        try:
            from datetime import datetime
            dt = datetime.strptime(start_dt, "%Y-%m-%d %H:%M:%S")
            ts = datetime.strptime(stamp, "%Y%m%d%H%M%S")
            if abs((dt - ts).total_seconds()) <= 600:
                return m.group(2), m.group(3)
        except ValueError:
            continue
    return None, None


def main():
    ap = argparse.ArgumentParser(description="官方 SimuroSot5.log 战绩统计（(黄:蓝) 口径）")
    ap.add_argument("log", nargs="?", default=r"C:\Strategy\SimuroSot5.log")
    ap.add_argument("--rlg-dir", default=r"C:\Strategy", help="rlg 目录（用于队名映射，可省略）")
    args = ap.parse_args()

    segs = parse_log(args.log)
    if not segs:
        print("未解析到任何比赛会话")
        return 1

    print("=" * 72)
    print("官方 SimuroSot5.log 战绩（比分口径 = (黄 : 蓝)，首位黄队=守左门 x=0）")
    print("=" * 72)

    rows = []
    for s in segs:
        if not s["events"]:
            continue
        last = s["events"][-1]
        complete = last["time"] <= 30.0   # 末事件接近 0 秒 = 完整场次
        pk_blue = sum(1 for e in s["events"] if "Penalty Kick Blue" in e["name"])
        pk_yellow = sum(1 for e in s["events"] if "Penalty Kick Yellow" in e["name"])
        yname, bname = team_name_from_rlg(args.rlg_dir, s["start"])
        name_str = f"  黄={yname} 蓝={bname}" if yname else ""
        flag = "完整" if complete else "未完/截断"
        rows.append(dict(start=s["start"], yellow=last["yellow"], blue=last["blue"],
                         complete=complete, pk_blue=pk_blue, pk_yellow=pk_yellow,
                         time=last["time"], name=name_str, flag=flag))
        print(f"{s['start']}  {last['yellow']} : {last['blue']}  ({flag}, 末事件T={last['time']:.0f}s)"
              f"  点球判罚: 蓝被罚{pk_blue} 黄被罚{pk_yellow}{name_str}")

    full = [r for r in rows if r["complete"]]
    print("\n" + "=" * 72)
    if full:
        wins = sum(1 for r in full if r["yellow"] > r["blue"])
        draws = sum(1 for r in full if r["yellow"] == r["blue"])
        losses = sum(1 for r in full if r["yellow"] < r["blue"])
        ty = sum(r["yellow"] for r in full)
        tb = sum(r["blue"] for r in full)
        tpk = sum(r["pk_blue"] for r in full) + sum(r["pk_yellow"] for r in full)
        print(f"完整场次 {len(full)} 场：黄(左门队) {wins} 胜 {draws} 平 {losses} 负")
        print(f"总比分  黄 {ty} : {tb} 蓝   （场均 黄 {ty/len(full):.1f} : {tb/len(full):.1f} 蓝）")
        print(f"点球判罚合计 {tpk} 次，场均 {tpk/len(full):.1f} 次（蓝被罚 {sum(r['pk_blue'] for r in full)}，黄被罚 {sum(r['pk_yellow'] for r in full)}）")
        print("提示：我们 = 蓝队（MyTeam-Blue）时，黄=对手；若我们打黄队位置，把黄蓝对调。")
    else:
        print("无完整场次（所有会话截断或未开赛）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
