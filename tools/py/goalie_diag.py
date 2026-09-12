# -*- coding: utf-8 -*-
"""
goalie_diag.py — 守门员专项诊断（从真机 .rlg 算，只读，不碰平台）

用法：
    python tools/py/goalie_diag.py <a.rlg> [b.rlg ...] [--goal-x 220]

为什么需要它：9/11 上机的第 44/47/49 轮改动让门将"看起来慢半拍"，光看速度分布分不清
是「该动时不动（角区守卫/判定没触发）」还是「动了但末段慢（制动包线）」。
本工具把两类证据分开量（门将 = blue[0]）：

  net_s    净比赛秒数（rlg 只录 PlayOn）
  p50/p90/p99/idle<20%/travel   门将速度画像（cm/s、静止帧占比、总行程）
  near45   球距我方门 <45cm 的帧数（门前告急帧）
  onball   这些帧里"门将距球 ≤25cm"的比例 ← 真正在门前管球的比重（低=漏救）
  threats  威胁事件数：球距门<130cm 且（朝门 vx>2cm/s 或距门<60cm），事件间隔>12 帧
  react%   威胁出现后 15 帧内门将位移 ≥3cm 的比例（该动就动）
  delay    反应延迟中位数（帧）：威胁后首次单帧位移 ≥0.6cm 的帧号
  tail     门将移动段（≥10cm）中"最后 7cm"耗时中位数（帧）← 制动包线只作用在这段
  corner%  球在我方两角 35cm 黄区内时，门将静止（<20cm/s）占比 ← 第49轮守卫签名

改前基线（9/9 三场，旧 motion/旧守卫）：p50≈12.6~16.1、idle≈56~59%、corner%≈32~47%
改后（9/11 三场，第44/47/49轮）：      p50≈0.1~3.7、idle≈64~75%、corner%≈59~100%
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlg_analyzer as RA

CORNER_R = 35.0


def analyze(path, goal_x=220.0):
    fr = RA.parse_rlg(path)
    n = len(fr)
    if n < 2:
        return None
    CORN = [(goal_x, 0.0), (goal_x, 180.0)]

    def near_corner(x, y):
        return any(((x - cx) ** 2 + (y - cy) ** 2) ** 0.5 < CORNER_R for cx, cy in CORN)

    speeds, travel, idle = [], 0.0, 0
    guard_frames = guard_idle = 0
    near45 = onball = 0
    wall_n = wall_side_ok = 0
    wall_post_d = []
    seg, segments = [], []

    def flush():
        if len(seg) >= 4:
            segments.append(list(seg))
        seg.clear()

    eps = []          # 威胁事件起点
    for i in range(1, n):
        f = fr[i]
        gk, gk0 = f["blue"][0], fr[i - 1]["blue"][0]
        d = ((gk["x"] - gk0["x"]) ** 2 + (gk["y"] - gk0["y"]) ** 2) ** 0.5
        speeds.append(d * 40.0)
        travel += d
        if d * 40.0 < 20.0:
            idle += 1
        if d > 0.2:
            seg.append((gk["x"] - gk0["x"], gk["y"] - gk0["y"]))
        else:
            flush()
        bx, by = f["ball"]["x"], f["ball"]["y"]
        dist_goal = goal_x - bx
        if near_corner(bx, by):
            guard_frames += 1
            if d * 40.0 < 20.0:
                guard_idle += 1
        if dist_goal < 45.0:
            near45 += 1
            if ((gk["x"] - bx) ** 2 + (gk["y"] - by) ** 2) ** 0.5 <= 25.0:
                onball += 1
        vx = bx - fr[i - 1]["ball"]["x"]
        # 墙边来球专项（docs/21 P1）：球贴边墙且朝我方门、距门<120cm
        if dist_goal < 120.0 and (by < 25.0 or by > 155.0) and vx > 0.0:
            wall_n += 1
            near_side = (by < 90.0)
            if (gk["y"] < 90.0) == near_side:
                wall_side_ok += 1
            wall_post_d.append(abs(gk["y"] - (76.0 if near_side else 104.0)))
        if dist_goal < 130.0 and (vx > 2.0 or dist_goal < 60.0):
            if not eps or i - eps[-1] > 12:
                eps.append(i)
        gk_prev = gk
    flush()

    react, delays = 0, []
    for si in eps:
        moved, delay = False, None
        for k in range(1, 16):
            if si + k >= n:
                break
            a, b = fr[si + k - 1]["blue"][0], fr[si + k]["blue"][0]
            dd = ((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2) ** 0.5
            if dd >= 0.6 and delay is None:
                delay = k
            if dd >= 0.075:      # 3cm/帧 太苛刻；0.075cm/帧 = 3cm/s …
                pass
            if dd * 40.0 >= 24.0:
                moved = True
        react += 1 if moved else 0
        if delay is not None:
            delays.append(delay)

    tails = []
    for s in segments:
        total = sum((dx * dx + dy * dy) ** 0.5 for dx, dy in s)
        if total < 10.0:
            continue
        acc, tf = 0.0, 0
        for dx, dy in s:
            acc += (dx * dx + dy * dy) ** 0.5
            if total - acc <= 7.0:
                tf += 1
        tails.append(tf)

    def med(v):
        v = sorted(v)
        return v[len(v) // 2] if v else float("nan")

    speeds.sort()
    m = len(speeds)

    def q(p):
        return speeds[int(p * (m - 1))] if m else 0.0

    return dict(name=os.path.basename(path), net_s=n / 40.0,
                p50=q(0.5), p90=q(0.9), p99=q(0.99), idle=idle / max(m, 1), travel=travel,
                near45=near45, onball=(onball / near45 if near45 else float("nan")),
                threats=len(eps), react=(react / len(eps) if eps else float("nan")),
                delay=med(delays), tail=med(tails),
                guard=(guard_idle / guard_frames if guard_frames else float("nan")),
                guard_frames=guard_frames,
                wall_n=wall_n, wall_side=(wall_side_ok / wall_n if wall_n else float("nan")),
                wall_post=(med(wall_post_d) if wall_post_d else float("nan")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg", nargs="+")
    ap.add_argument("--goal-x", type=float, default=220.0)
    args = ap.parse_args()
    rows = [r for r in (analyze(p, args.goal_x) for p in args.rlg) if r]
    print(f"{'日志':<20}{'净比赛':>7}{'p50':>6}{'p90':>6}{'p99':>6}{'idle':>7}{'行程':>7}"
          f"{'门前45':>7}{'在球边':>7}{'威胁':>6}{'反应率':>7}{'延迟':>5}{'末段7cm':>8}{'角区静止':>8}"
          f"{'墙边来球':>9}{'在近柱侧':>9}{'距近柱':>7}")
    for r in rows:
        print(f"{r['name'][:18]:<20}{r['net_s']:>6.0f}s{r['p50']:>6.1f}{r['p90']:>6.1f}{r['p99']:>6.1f}"
              f"{r['idle']*100:>6.1f}%{r['travel']:>6.0f}{r['near45']:>7}"
              f"{r['onball']*100 if r['near45'] else float('nan'):>6.0f}%{r['threats']:>6}"
              f"{r['react']*100 if r['threats'] else float('nan'):>6.0f}%{r['delay']:>5.0f}"
              f"{r['tail']:>8.1f}{r['guard']*100 if r['guard_frames'] else float('nan'):>7.0f}%"
              f"{r['wall_n']:>9}{r['wall_side']*100 if r['wall_n'] else float('nan'):>8.0f}%"
              f"{r['wall_post']:>7.1f}")
    print("\n看四处：① 在球边%（门前告急时门将在不在球旁）② 反应率/延迟（该动就动吗）"
          "③ 末段7cm（包线代价）④ 角区静止%（第49轮守卫代价）"
          "⑤ 墙边来球：在近柱侧% 与 距近柱(cm)（docs/21 P1 的直接指标）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
