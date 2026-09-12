# -*- coding: utf-8 -*-
"""
referee_diag.py — 真机判罚归因诊断（rlg 逐帧）

用法：
    python tools/py/referee_diag.py 比赛日志.rlg [--window 20] [--dur 20] [--list] [--thresh 40]

背景：.rlg 只录 PlayOn 帧（gs/whos 字段恒 0），死球/摆位时段不落盘。
      所以「判罚」只能靠**摆位瞬移**识别：连续 PlayOn 帧之间若球或机器人位置跳变，
      说明中间发生过一次死球重置（门球/点球/任意球/争球/开球）。

输出三块：
  1. 停表事件（摆位瞬移）次数 + 每次前 20 帧的犯规特征（黄队在我方门区人数、离门将距离…）
  2. 违规「持续段」统计（>=dur 帧才算真判罚红线，单帧穿过不算）
  3. 全场汇总，用于改前/改后对照

口径来源：include/simuro5/field_info.hpp + docs/00-比赛规则速查.md
  蓝队守 x=220 门，黄队守 x=0 门（rlg 坐标系原生 [0,220]）
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import rlg_analyzer as RA

BOX_DEPTH = 50.0           # 门区(小禁区)纵深；进攻方判定域 y = 90 ± 27.5 = [62.5,117.5]
BOX_HALF_W = 27.5
OWN_GA_HALF_W = 15.0       # 守方门区 y ∈ [75,105]
PEN_DEPTH = 80.0           # 罚球区纵深；y = 90 ± 17.5 = [72.5,107.5]
PEN_HALF_W = 17.5


def in_box(x, y, goal_x, depth, half_w):
    return (goal_x - depth) <= x <= goal_x and (90.0 - half_w) <= y <= (90.0 + half_w)


def frame_counts(f):
    y_box = sum(1 for r in f["yellow"] if in_box(r["x"], r["y"], 220.0, BOX_DEPTH, BOX_HALF_W))
    y_pen = sum(1 for r in f["yellow"] if in_box(r["x"], r["y"], 220.0, PEN_DEPTH, PEN_HALF_W))
    b_own = sum(1 for r in f["blue"][1:] if in_box(r["x"], r["y"], 220.0, BOX_DEPTH, OWN_GA_HALF_W))
    b_opp = sum(1 for r in f["blue"][1:] if in_box(r["x"], r["y"], 0.0, BOX_DEPTH, BOX_HALF_W))
    gk = f["blue"][0]
    gk_d = min(math.hypot(r["x"] - gk["x"], r["y"] - gk["y"]) for r in f["yellow"])
    # 黄队（守方）己方门区纪律：除守门员外不得在门区(小禁区)内，罚球区不得 4 人
    y_own = sum(1 for r in f["yellow"][1:] if in_box(r["x"], r["y"], 0.0, BOX_DEPTH, OWN_GA_HALF_W))
    y_own_pen = sum(1 for r in f["yellow"][1:] if in_box(r["x"], r["y"], 0.0, PEN_DEPTH, PEN_HALF_W))
    return dict(y_box=y_box, y_pen=y_pen, b_own=b_own, b_opp=b_opp, gk_d=gk_d,
                y_own=y_own, y_own_pen=y_own_pen)


def episodes(frames, pred, dur):
    """条件满足且连续 >= dur 帧的段；返回 [(起始帧, 长度)]"""
    out = []
    start = None
    for i, f in enumerate(frames):
        if pred(f):
            if start is None:
                start = i
        else:
            if start is not None and i - start >= dur:
                out.append((start, i - start))
            start = None
    if start is not None and len(frames) - start >= dur:
        out.append((start, len(frames) - start))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg")
    ap.add_argument("--window", type=int, default=20)
    ap.add_argument("--dur", type=int, default=20, help="违规持续帧阈值（平台红线 20 周期）")
    ap.add_argument("--thresh", type=float, default=40.0, help="摆位瞬移判定阈值 cm")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    frames = RA.parse_rlg(args.rlg)
    n = len(frames)
    play_s = n / 40.0
    print(f"===== 判罚归因: {os.path.basename(args.rlg)} =====")
    print(f"PlayOn 帧 {n}（约 {play_s:.1f}s 净比赛时间；死球时段未落盘）")

    # ---------- 1. 停表事件（摆位瞬移）----------
    stops = []
    for i in range(1, n):
        bd = math.hypot(frames[i]["ball"]["x"] - frames[i - 1]["ball"]["x"],
                        frames[i]["ball"]["y"] - frames[i - 1]["ball"]["y"])
        rd = 0.0
        for t in ("blue", "yellow"):
            for a, b in zip(frames[i][t], frames[i - 1][t]):
                rd = max(rd, math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
        if max(bd, rd) > args.thresh:
            stops.append((i, bd, rd))
    print(f"\n-- 停表/摆位事件 {len(stops)} 次（净比赛 {play_s/60:.2f} 分钟 → "
          f"{len(stops)/max(play_s/60,1e-9):.1f} 次/分钟）--")
    if args.list:
        print(f"{'帧':>7}{'时间(s)':>9}{'球跳':>8}{'机跳':>8}   {'停前黄队门区峰值':>16}{'停前黄→蓝门将':>14}"
              f"{'停前球位':>18}{'重启球位':>18}")
    for i, bd, rd in stops:
        lo = max(0, i - args.window)
        win = frames[lo:i]
        pk = max((frame_counts(w)["y_box"] for w in win), default=0)
        gkd = min((frame_counts(w)["gk_d"] for w in win), default=99.0)
        if args.list:
            print(f"{i:>7}{i/40:>9.1f}{bd:>8.1f}{rd:>8.1f}   {pk:>16}{gkd:>14.1f}"
                  f"{'({:.0f},{:.0f})'.format(frames[i-1]['ball']['x'], frames[i-1]['ball']['y']):>18}"
                  f"{'({:.0f},{:.0f})'.format(frames[i]['ball']['x'], frames[i]['ball']['y']):>18}")

    # ---------- 2. 违规持续段 ----------
    conds = [
        ("黄队在我方门区>=2人 (点球/门球红线)", lambda f: frame_counts(f)["y_box"] >= 2),
        ("黄队在我方罚球区>=4人", lambda f: frame_counts(f)["y_pen"] >= 4),
        ("蓝队非门将在我方门区 (守门员须独守)", lambda f: frame_counts(f)["b_own"] >= 1),
        ("蓝队非门将在对方门区>=2人 (我方进攻聚集)", lambda f: frame_counts(f)["b_opp"] >= 2),
        ("黄队非门将进己方门区 (守门员须独守黄门区)", lambda f: frame_counts(f)["y_own"] >= 1),
        ("黄队非门将进己方罚球区>=3人", lambda f: frame_counts(f)["y_own_pen"] >= 3),
    ]
    print(f"\n-- 违规持续段（连续 >= {args.dur} 帧才算，单帧穿过不算）--")
    for name, pred in conds:
        eps = episodes(frames, pred, args.dur)
        tot = sum(e[1] for e in eps)
        detail = ", ".join(f"帧{s}({l}帧)" for s, l in eps[:8])
        more = " …" if len(eps) > 8 else ""
        print(f"  {name:<36} {len(eps):>3} 段 / {tot:>5} 帧 ({tot/40:5.1f}s)")
        if eps:
            print(f"      {detail}{more}")

    # ---------- 3. 汇总 ----------
    y_box_frames = sum(1 for f in frames if frame_counts(f)["y_box"] >= 2)
    b_own_frames = sum(1 for f in frames if frame_counts(f)["b_own"] >= 1)
    # 停表前的犯规特征分布（关键：黄队是否在我方门区聚集 / 是否贴到门将）
    sig = dict(ge1=0, ge2=0, gk12=0, gk20=0)
    for i, _bd, _rd in stops:
        win = frames[max(0, i - args.window):i]
        pk = max((frame_counts(w)["y_box"] for w in win), default=0)
        gkd = min((frame_counts(w)["gk_d"] for w in win), default=99.0)
        if pk >= 1:
            sig["ge1"] += 1
        if pk >= 2:
            sig["ge2"] += 1
        if gkd < 12.0:
            sig["gk12"] += 1
        if gkd < 20.0:
            sig["gk20"] += 1
    print("\n-- 停表前 20 帧的犯规特征（占总停表事件的比例）--")
    if stops:
        for k, lab in (("ge1", "黄队有人在我方门区"), ("ge2", "黄队在我方门区>=2人"),
                       ("gk12", "黄队贴到蓝门将<12cm"), ("gk20", "黄队贴到蓝门将<20cm")):
            print(f"  {lab:<24}: {sig[k]:>3}/{len(stops)}  ({sig[k]/len(stops)*100:5.1f}%)")
    print("\n-- 汇总（改前/改后对照）--")
    print(f"  停表事件/净分钟          : {len(stops)/max(play_s/60,1e-9):.1f}")
    print(f"  黄队门区>=2人（任意帧）  : {y_box_frames} 帧 ({y_box_frames/40:.1f}s)")
    print(f"  蓝队非门将进己方门区(帧) : {b_own_frames} 帧 ({b_own_frames/40:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
