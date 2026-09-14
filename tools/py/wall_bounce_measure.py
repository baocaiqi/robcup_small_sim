# -*- coding: utf-8 -*-
"""临时探针 v2：真机 .rlg 实测球撞墙后的速度变化（修正符号 + 排除进球/瞬移）。

判据（本地极值法，不要求球真的贴到墙上）：
  · 底墙：vy 由负转正，且 y 是局部最小、y < 8cm
  · 顶墙：vy 由正转负，且 y 是局部最大、y > 172cm
  · 门线墙：vx 变号、x 贴到 0 或 220 附近，且球的 y 在球门口 [70,110] **之外**
    （在球门口之内 = 进球，不是弹墙）
  · 剔除：单帧位移 >30cm（平台瞬移）、出射速度比入射大 1.2 倍以上（瞬移/误检）
"""
import glob
import os
import struct
import sys

sys.path.insert(0, os.path.join("tools", "py"))
import constants as C  # noqa: E402

FRAME, FMT, OFF = C.FRAME_BYTES, "<44d", C.LOG_OFFSET_X
JUMP, V_MIN = 30.0, 1.5


def ball_series(path):
    data = open(path, "rb").read()
    n = len(data) // FRAME
    out = []
    for i in range(n):
        v = struct.unpack_from(FMT, data, i * FRAME)
        out.append((v[40] + OFF, v[41], int(v[42]), int(v[43])))
    return out


def scan(path, S):
    pts = ball_series(path)
    for i in range(1, len(pts) - 1):
        x0, y0, gs0, _ = pts[i - 1]
        x1, y1, gs1, _ = pts[i]
        x2, y2, gs2, _ = pts[i + 1]
        if gs0 or gs1 or gs2:
            continue
        vix, viy = x1 - x0, y1 - y0
        vox, voy = x2 - x1, y2 - y1
        if max(abs(vix), abs(viy), abs(vox), abs(voy)) > JUMP:
            continue
        # 底墙 y=0
        if viy < -V_MIN and voy > V_MIN and y1 < 8.0 and y1 <= y0 and y1 <= y2:
            r = voy / (-viy)
            if r < 1.2:
                S["y0_n"].append(r)
                if abs(vix) > V_MIN:
                    S["y0_t"].append(vox / vix)
        # 顶墙 y=180
        if viy > V_MIN and voy < -V_MIN and y1 > 172.0 and y1 >= y0 and y1 >= y2:
            r = (-voy) / viy
            if r < 1.2:
                S["y180_n"].append(r)
                if abs(vix) > V_MIN:
                    S["y180_t"].append(vox / vix)
        # 门线墙 x=0（球门口之外才是弹墙）
        in_mouth = 70.0 <= y1 <= 110.0
        if vix < -V_MIN and vox > V_MIN and x1 - OFF < 8.0 and x1 <= x0 and x1 <= x2 and not in_mouth:
            r = vox / (-vix)
            if r < 1.2:
                S["x0_n"].append(r)
                if abs(viy) > V_MIN:
                    S["x0_t"].append(voy / viy)
        # 门线墙 x=220
        if vix > V_MIN and vox < -V_MIN and x1 - OFF > C.FIELD_LENGTH - 8.0 and x1 >= x0 and x1 >= x2 and not in_mouth:
            r = (-vox) / vix
            if r < 1.2:
                S["x220_n"].append(r)
                if abs(viy) > V_MIN:
                    S["x220_t"].append(voy / viy)


def med(a):
    b = sorted(a)
    return b[len(b) // 2] if b else float("nan")


def pct(a, q):
    b = sorted(a)
    return b[min(len(b) - 1, int(len(b) * q))] if b else float("nan")


files = sorted(glob.glob(sys.argv[1] if len(sys.argv) > 1 else r"C:\Strategy\*.rlg"))
S = {k: [] for k in ("y0_n", "y0_t", "y180_n", "y180_t", "x0_n", "x0_t", "x220_n", "x220_t")}
for p in files:
    try:
        scan(p, S)
    except Exception as e:
        print("skip", os.path.basename(p), e)

print(f"扫了 {len(files)} 个 .rlg（{sum(len(v) for v in S.values())} 个弹墙样本）\n")
print(f"{'墙':<12}{'样本':>6}{'法向恢复 p25/中位/p75':>26}{'切向保持 中位':>16}")
for name, nk, tk, label in (
        ("底墙 y=0", "y0_n", "y0_t", "y=0"),
        ("顶墙 y=180", "y180_n", "y180_t", "y=180"),
        ("门线 x=0", "x0_n", "x0_t", "x=0"),
        ("门线 x=220", "x220_n", "x220_t", "x=220")):
    n = S[nk]
    if not n:
        print(f"{name:<12}{0:>6}{'—':>26}{'—':>16}")
        continue
    print(f"{name:<12}{len(n):>6}{pct(n,.25):>9.2f} /{med(n):>7.2f} /{pct(n,.75):>7.2f}"
          f"{med(S[tk]) if S[tk] else float('nan'):>16.2f}")
print("\n对照：sim_bench 用的 kWallRest=0.45（法向）、kWallFric=0.90（切向）")
print("       defense.hpp 的 predict_y_at_x_reflect 用的是理想镜面（法向 = 1.0）")
