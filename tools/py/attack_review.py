# -*- coding: utf-8 -*-
"""attack_review.py — 进攻端真机复盘（块②A/②B 验证用）

用法: python tools/py/attack_review.py <rlg路径> [官方log路径]
输出:
  1. 我方持球段统计（死带检测：贴球停滞>30帧 / 门角死带 / 被围回传）
  2. 门前推线事件（球距门<8cm 且我方最近：推过/未过/被清）
  3. 我方进球方式（运动战/点球/对方失误）与官方比分核对
"""
import sys, os, re, glob
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

def parse_rlg_local(path):
    from rlg_analyzer import parse_rlg
    return parse_rlg(path)

def official_score(log_path, tag):
    if not log_path or not os.path.exists(log_path):
        return "?"
    txt = open(log_path, encoding="utf-8", errors="replace").read()
    segs = re.split(r"Starting Controller at ", txt)
    last = segs[-1]
    scores = re.findall(r"\((\d+)\s*:\s*(\d+)\)", last)
    pk_y = len(re.findall(r"Penalty Kick Yellow", last))
    pk_b = len(re.findall(r"Penalty Kick Blue", last))
    return scores[-1] if scores else "?", pk_y, pk_b

def main():
    if len(sys.argv) < 2:
        print("usage: attack_review.py <rlg> [SimuroSot5.log]"); return
    rlg = sys.argv[1]
    log = sys.argv[2] if len(sys.argv) > 2 else None
    frames = parse_rlg_local(rlg)
    n = len(frames)
    print(f"### {os.path.basename(rlg)}  ({n/40:.0f}s)")

    # 我方进球/失球（断档法）+ 进球方式分类
    our_g = []; opp_g = []
    for i in range(1, n):
        b0, b1 = frames[i-1]["ball"], frames[i]["ball"]
        d = ((b0["x"]-b1["x"])**2 + (b0["y"]-b1["y"])**2) ** 0.5
        if d > 40:
            if b0["x"] < 1.5 and 70 < b0["y"] < 110: our_g.append(i)
            if b0["x"] > 218.5 and 70 < b0["y"] < 110: opp_g.append(i)
    print(f"   我方进球断档: {our_g}   对方进球断档: {opp_g}")
    # 进球分类：断档帧前 0.5s 我方最近者距球 <25cm = 我方贴球终结（运动战）；
    #   >25cm = 球自己滚进（demo 失误/推大）
    for g in our_g:
        p = max(0, g - 20)
        b = frames[p]["ball"]
        dmin = min(((frames[p]["blue"][j]["x"]-b["x"])**2 + (frames[p]["blue"][j]["y"]-b["y"])**2)**0.5 for j in range(5))
        ymin = min(((frames[p]["yellow"][j]["x"]-b["x"])**2 + (frames[p]["yellow"][j]["y"]-b["y"])**2)**0.5 for j in range(5))
        typ = "运动战(我方贴球)" if dmin < 25 else "demo失误/球自滚"
        print(f"   我方进球@{g/40:.1f}s: {typ}  我方最近{dmin:.0f}cm demo最近{ymin:.0f}cm")

    # 死带检测：我方(非GK)贴球(d<15) 且 球速<2(帧间差分) 且 持续>30帧
    stalls = []
    cur = None
    for i in range(n):
        b = frames[i]["ball"]
        bmin, dmin = -1, 15.0
        for j in range(1, 5):
            d = ((frames[i]["blue"][j]["x"]-b["x"])**2 + (frames[i]["blue"][j]["y"]-b["y"])**2)**0.5
            if d < dmin: dmin = d; bmin = j
        spd = 0.0
        if i > 0:
            b0 = frames[i-1]["ball"]
            spd = ((b["x"]-b0["x"])**2 + (b["y"]-b0["y"])**2) ** 0.5
        slow = spd < 2.0
        own_touch = bmin >= 1
        if own_touch and slow and cur is None:
            cur = [i, i, round(b["x"]), round(b["y"])]
        elif own_touch and slow and cur is not None:
            cur[1] = i
        else:
            if cur is not None:
                if cur[1]-cur[0] > 30: stalls.append(cur)
                cur = None
    if cur is not None and cur[1]-cur[0] > 30: stalls.append(cur)
    print(f"   死带停滞(贴球>30帧): {len(stalls)} 段")
    for s in stalls[:10]:
        print(f"     帧{s[0]}-{s[1]}({s[0]/40:.0f}s) {s[1]-s[0]}帧 @({s[2]},{s[3]})")

    # 门前推线事件：球距对方门(0,90)<10 且 我方最近——统计每次贴线事件结局
    line_events = []
    cur = None
    for i in range(n):
        b = frames[i]["ball"]
        dgoal = ((b["x"])**2 + (b["y"]-90)**2)**0.5
        bmin, dmin = -1, 25.0
        for j in range(0, 5):
            d = ((frames[i]["blue"][j]["x"]-b["x"])**2 + (frames[i]["blue"][j]["y"]-b["y"])**2)**0.5
            if d < dmin: dmin = d; bmin = j
        near_line = dgoal < 12.0
        if near_line and cur is None:
            cur = [i, i, bmin]
        elif near_line and cur is not None:
            cur[1] = i
        else:
            if cur is not None:
                if cur[1]-cur[0] >= 5: line_events.append(cur)
                cur = None
    print(f"   门前推线事件(距门<12cm ≥5帧): {len(line_events)}")
    for e in line_events[:12]:
        i0, i1 = e[0], min(e[1]+3, n-1)
        b_end = frames[i1]["ball"]
        b0 = frames[max(0,e[0]-1)]["ball"]
        crossed = b_end["x"] < -0.5
        print(f"     帧{i0}-{i1} 最近B{e[2]} 后球({b_end['x']:.0f},{b_end['y']:.0f}) {'过线!' if crossed else ''}")

    # 官方 log 比分核对
    if log:
        sc = official_score(log, None)
        print(f"   官方log: 终分={sc[0]} 我方被罚点球={sc[1]} 我方主罚={sc[2]}")

if __name__ == "__main__":
    main()
