# -*- coding: utf-8 -*-
"""ball_control_diag.py — 球权诊断：球旁边站的到底是谁的人？（只读黑匣子 CSV）

回答一个具体问题：**我们持球率低，是"运动控制不行"还是"站位策略不抢球"？**
判据（用同一份逐帧数据，双方口径完全对称）：
  · 每帧"离球 30cm 内的我方人数 / 对方人数"（门将都排除）
  · 最近机器人到球的距离（中位）：我方 vs 对方
  · "我方最近者比对方更近"的帧占比 ⇒ 谁先到球

如果**我方人数明显少、但最近距离接近** ⇒ 是**站位策略**让球（设计选择：固定分工、只在合适时才上前）；
如果**人数接近但最近距离明显差** ⇒ 才是**运动控制/响应**的问题。

用法：python tools\\py\\ball_control_diag.py [--sessions 8]
"""
import argparse
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
CSV = r"C:\Strategy\hnnu_blackbox.csv"


def segments(path, n):
    segs, cur = [], None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("# ---- new session"):
                if cur:
                    segs.append(cur)
                cur = []
            elif cur is not None:
                cur.append(line)
    if cur:
        segs.append(cur)
    return segs[-n:]


def stats(rows):
    ours_n = theirs_n = frames = first_us = 0
    our_d, their_d = [], []
    ball = None
    for line in rows:
        p = line.rstrip("\n").split(",")
        if p[0] == "F" and len(p) > 5:
            try:
                ball = (float(p[4]), float(p[5]))
            except ValueError:
                ball = None
        elif p[0] == "R" and len(p) >= 23 and ball:
            pts = [(float(p[3 + 4 * i]), float(p[4 + 4 * i])) for i in range(5)]
            dd = [((ball[0] - x) ** 2 + (ball[1] - y) ** 2) ** 0.5 for (x, y) in pts[1:]]
            ours_n += sum(1 for v in dd if v < 30.0)
            our_d.append(min(dd))
        elif p[0] == "O" and len(p) >= 17 and ball:
            pts = [(float(p[2 + 3 * i]), float(p[3 + 3 * i])) for i in range(5)]
            dd = [((ball[0] - x) ** 2 + (ball[1] - y) ** 2) ** 0.5 for (x, y) in pts[1:]]
            theirs_n += sum(1 for v in dd if v < 30.0)
            their_d.append(min(dd))
            if our_d and len(our_d) == len(their_d):     # 同一帧两边都有数据
                frames += 1
                if our_d[-1] < their_d[-1]:
                    first_us += 1
    m = lambda a: sorted(a)[len(a) // 2] if a else float("nan")
    n = max(frames, 1)
    return dict(frames=len(our_d), ours_near=ours_n / n, theirs_near=theirs_n / n,
                our_med=m(our_d), their_med=m(their_d),
                first_us=100.0 * first_us / n, our_p10=sorted(our_d)[len(our_d) // 10] if our_d else float("nan"))


ap = argparse.ArgumentParser()
ap.add_argument("--sessions", type=int, default=8)
a = ap.parse_args()
print(f"{'session':>8} {'帧':>7} {'我方近球人数':>12} {'对方近球人数':>12} "
      f"{'我方最近(中位)':>14} {'对方最近(中位)':>14} {'我方先到球%':>11}")
for i, rows in enumerate(segments(CSV, a.sessions), 1):
    st = stats(rows)
    if st["frames"] < 50:
        continue
    print(f"{i:>8} {st['frames']:>7} {st['ours_near']:>12.2f} {st['theirs_near']:>12.2f} "
          f"{st['our_med']:>14.1f} {st['their_med']:>14.1f} {st['first_us']:>11.1f}")
print("\n（近球人数 = 除门将外离球 <30cm 的机器人数，双方同一口径）")
