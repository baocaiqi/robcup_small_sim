# -*- coding: utf-8 -*-
"""
penalty_diag.py — 点球专项复盘（只读 rlg）：一次点球里，我们到底做了什么？

用法：
    python tools/py/penalty_diag.py <rlg> [<rlg> ...]

判据：点球时平台会把球摆到罚球点（门前 92cm）并摆位 → 在 PlayOn 帧里表现为
  · 我方主罚：球出现在 (92, 90) 附近（我们攻对方球门 x=0）
  · 对方主罚：球出现在 (128, 90) 附近（我们守 x=220）
随后分析接下来最多 400 帧（10s）：

  t_ball     球第一次被推动的帧数（球速 >1cm/帧）＝ 我们多久才出手；None = 全程没碰球
  t_act      主罚者（blue[1]）第一次进入球 25cm 内的帧数
  dmin       主罚者离球的最近距离（cm）
  vmax       球被推动后的峰值球速（cm/帧）
  goal       球是否最终进门（越过对方门线 x<0 且在门宽内）
  stall      是否又出现一次摆位（＝被平台重置/僵局）

结论怎么读：
  · t_ball=None & dmin 很大 → 压根没去（决策/站位问题，不是速度）
  · t_act 很大/很小但 dmin 只有 20+ → 去了但够不到/擦边（姿态·对准问题）
  · t_ball 有 & vmax 小 → 推得太轻（力量/速度问题）
  · 有推 & vmax 大但没进 → 方向/门将封堵问题
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlg_analyzer as RA

SPOT_BLUE = (92.0, 90.0)     # 我方主罚
SPOT_YELLOW = (128.0, 90.0)  # 对方主罚
WINDOW = 400


def d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def analyze(path):
    fr = RA.parse_rlg(path)
    n = len(fr)
    print(f"===== {os.path.basename(path)}（{n/40:.0f}s）=====")
    found = 0
    i = 1
    while i < n:
        b = fr[i]["ball"]
        p = fr[i - 1]["ball"]
        jump = d((b["x"], b["y"]), (p["x"], p["y"])) > 40.0
        if jump and (d((b["x"], b["y"]), SPOT_BLUE) < 4.0 or d((b["x"], b["y"]), SPOT_YELLOW) < 4.0):
            who = "我方主罚" if d((b["x"], b["y"]), SPOT_BLUE) < 4.0 else "对方主罚"
            found += 1
            end = min(n, i + WINDOW)
            t_ball = t_act = None
            dmin = 1e9
            vmax = 0.0
            stall = False
            goal = False
            for j in range(i + 1, end):
                bb, pp = fr[j]["ball"], fr[j - 1]["ball"]
                v = d((bb["x"], bb["y"]), (pp["x"], pp["y"]))
                if v > 1.0 and t_ball is None:
                    t_ball = j - i
                vmax = max(vmax, v)
                act = fr[j]["blue"][1]
                dd = d((act["x"], act["y"]), (bb["x"], bb["y"]))
                dmin = min(dmin, dd)
                if dd <= 25.0 and t_act is None:
                    t_act = j - i
                if d((bb["x"], bb["y"]), (pp["x"], pp["y"])) > 40.0:   # 又摆位
                    stall = True
                    break
                if who == "我方主罚" and bb["x"] < 0.0 and 70.0 <= bb["y"] <= 110.0:
                    goal = True
            print(f"  [{found}] 帧{i} (~{i/40:.0f}s) {who}："
                  f"出手={('%.2fs' % (t_ball/40)) if t_ball is not None else '全程没碰球'}，"
                  f"主罚者进 25cm={('%.2fs' % (t_act/40)) if t_act is not None else '没进入'}，"
                  f"最近离球={dmin:.0f}cm，峰值球速={vmax:.1f}cm/帧，"
                  f"进球={'是' if goal else '否'}，被重置={'是' if stall else '否'}")
            i = end
        else:
            i += 1
    if found == 0:
        print("  本场没检测到点球摆位（球没落在 (92,90) 或 (128,90) 附近）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg", nargs="+")
    a = ap.parse_args()
    for p in a.rlg:
        analyze(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
