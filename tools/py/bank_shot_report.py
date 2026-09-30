# -*- coding: utf-8 -*-
"""bank_shot_report.py — 从真机 .rlg 找「借墙射门 / 借墙进球」证据（纯读日志，不改任何东西）

为什么要它：借墙射门（bank shot）的门槛、反弹点几何都是算出来的，但真机到底用没用上、
用上了进没进，一直只有 sim 数据（docs/06 第 65 / 95 轮）。这份工具把 .rlg 逐帧翻一遍，
直接给出「撞边墙 → 之后进球」的场次与球数——第 96 轮真机借墙测试就是用它的输出判效果。

判据（与 tools/py/wall_bounce_measure.py 同源口径）：
  · 撞墙事件：球在法向坐标上的局部极值帧、离墙 <8cm、出入射速度比 <1.2（剔除瞬移/误检）
      底墙 y=0（vy 由负转正）、顶墙 y=180（vy 由正转负）
  · 我方进球：球 x ≤ 3.5 且 y∈[68,112] 且 vx<0（蓝队攻 x=0 的球门，守 x=220）；
      另用平台 PlaceKick 事件（gameState 变 5 = PlaceKick_Yellow = 蓝队刚得分）交叉校验，
      两者 30 帧内视为同一球
  · 借墙进球：进球前 --lookback 帧内出现过一次底/顶墙撞击（默认 100 帧 = 2.5s @40Hz）
  · 剔除：单帧位移 >30cm（平台摆位/重开时的瞬移）不作为物理证据

用法：
  python tools/py/bank_shot_report.py                    # 扫 C:\\Strategy\\*.rlg，只列有借墙进球的场次
  python tools/py/bank_shot_report.py --all               # 每场都列（含 0 次的）
  python tools/py/bank_shot_report.py --lookback 60       # 收紧"撞墙后多久内进球算借墙"
  python tools/py/bank_shot_report.py 某个.rlg [另一个.rlg]

⚠️ 比分一律以官方 `SimuroSot5.log` 为准（rlg 40Hz 采样会漏帧/误报，见 rlg_analyzer.py 文件头）。
"""
import glob
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C  # noqa: E402

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

FRAME = C.FRAME_BYTES
OFF = C.LOG_OFFSET_X
JUMP = 30.0        # cm/帧：超过它 = 平台瞬移（摆位/重开）
V_MIN = 1.5        # cm/帧：法向速度门槛
DEFAULT_LOOKBACK = 100


def ball_series(path):
    """逐帧取 (球x, 球y, gameState, whosBall)。日志布局：蓝5×4 + 黄5×4 = 40 个 double，球在 40/41。"""
    data = open(path, "rb").read()
    n = len(data) // FRAME
    out = []
    for i in range(n):
        v = struct.unpack_from("<44d", data, i * FRAME)
        out.append((v[40] + OFF, v[41], int(v[42]), int(v[43])))
    return out


def analyse(path, lookback):
    pts = ball_series(path)
    n = len(pts)
    bounces = []      # (frame, wall_label, 球x, 球y, 恢复系数)
    goals = []        # (frame, 判定来源)
    last_goal = -999
    for i in range(1, n - 1):
        x0, y0, _, _ = pts[i - 1]
        x1, y1, _, _ = pts[i]
        x2, y2, _, _ = pts[i + 1]
        vix, viy = x1 - x0, y1 - y0
        vox, voy = x2 - x1, y2 - y1
        jump = max(abs(vix), abs(viy), abs(vox), abs(voy)) > JUMP
        if not jump and viy < -V_MIN and voy > V_MIN and y1 < 8.0 and y1 <= y0 and y1 <= y2:
            r = voy / (-viy)
            if r < 1.2:
                bounces.append((i, "y=0(底墙)", x1, y1, r))
        if not jump and viy > V_MIN and voy < -V_MIN and y1 > 172.0 and y1 >= y0 and y1 >= y2:
            r = (-voy) / viy
            if r < 1.2:
                bounces.append((i, "y=180(顶墙)", x1, y1, r))
        # 我方进球（蓝队攻 x=0）
        if not jump and x1 <= 3.5 and 68.0 <= y1 <= 112.0 and vox < 0 and i - last_goal > 30:
            goals.append((i, "rlg过线"))
            last_goal = i
    # 平台事件交叉校验：gameState 变 5 = PlaceKick_Yellow = 蓝队得分
    for i in range(1, n):
        if pts[i][2] == 5 and pts[i - 1][2] != 5 and i - last_goal > 30:
            goals.append((i, "PlaceKick_Yellow"))
            last_goal = i
    goals.sort()
    bank_goals = []
    for gf, how in goals:
        near = [b for b in bounces if 0 < gf - b[0] <= lookback]
        if near:
            bank_goals.append((gf, how, near[-1]))
    # 撞墙位置分布：进攻半场（x<110，蓝队）还是自家半场
    att = sum(1 for b in bounces if b[2] < C.FIELD_LENGTH / 2.0)
    return dict(n=n, bounces=bounces, goals=goals, bank_goals=bank_goals, att=att)


def main():
    argv = sys.argv[1:]
    lookback = DEFAULT_LOOKBACK
    if "--lookback" in argv:
        k = argv.index("--lookback")
        lookback = int(argv[k + 1])
        del argv[k:k + 2]
    show_all = "--all" in argv
    files = [a for a in argv if not a.startswith("--")]
    if not files:
        files = sorted(glob.glob(r"C:\Strategy\*.rlg"), key=os.path.getmtime)

    tot_b = tot_g = tot_bg = tot_att = 0
    rows = []
    for p in files:
        try:
            r = analyse(p, lookback)
        except Exception as e:
            print("skip", os.path.basename(p), e)
            continue
        tot_b += len(r["bounces"])
        tot_att += r["att"]
        tot_g += len(r["goals"])
        tot_bg += len(r["bank_goals"])
        if r["bank_goals"] or show_all:
            rows.append((p, r))

    pct = (100.0 * tot_bg / tot_g) if tot_g else 0.0
    print(f"扫了 {len(files)} 个 .rlg（借墙判定窗口 {lookback} 帧）：")
    print(f"  撞边墙 {tot_b} 次（其中进攻半场 {tot_att} 次）")
    print(f"  我方进球(日志判定) {tot_g} 个，其中**撞墙后进球 {tot_bg} 个（{pct:.1f}%）**")
    print(f"  ⇒ 借墙进球越多 = 借墙射门在真机上真的打成了；一个都没有 = 门槛/几何/系数有问题\n")
    for p, r in rows:
        print(f"— {os.path.basename(p)}  帧 {r['n']}  撞墙 {len(r['bounces'])}（进攻半场 {r['att']}）"
              f"  我方进球 {len(r['goals'])}  借墙进球 {len(r['bank_goals'])}")
        pts = ball_series(p)
        for gf, how, (bf, wall, bx, by, rest) in r["bank_goals"]:
            print(f"    ★ 帧 {gf} 进球({how}) ← 帧 {bf} 撞 {wall} @ 球({bx:.0f},{by:.0f})"
                  f" 恢复系数 {rest:.2f} 间隔 {gf - bf} 帧")
    if not rows:
        print("（没有任何一场出现「撞墙后进球」）")


main()
