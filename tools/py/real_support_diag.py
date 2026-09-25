"""真机「有没有人上前」诊断：从 .rlg 逐帧量化我方非门将球员对球的逼近。

用途：对比两次真机录像（例如「传球任务开/关」两版 DLL）在**同一对手**下的接应/上前行为，
把「球在前面但没人上前」这种观感变成可比的数字。

指标（只在 PlayOn 帧上统计；蓝 = 我方，blue[0] = 门将不计）：
  min_d           最近非门将球员到球的距离（cm，越小 = 越有人上前）
  close30/close50  30cm / 50cm 内有人的帧占比
  ahead            我方球员 x < 球 x（即越过球、朝对方球门方向）的人数均值
  ball_x           球平均 x（越小 = 越靠对方门，蓝队攻向 x=0）
  lone_ball_pct    球在我方进攻半场（x<110）且最近非门将 > 60cm 的帧占比 = 「球在前面没人上前」

用法：
  python tools/py/real_support_diag.py "C:\\Strategy\\*.rlg" ...
  python tools/py/real_support_diag.py --label pass_off a.rlg --label pass_on b.rlg
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import rlg_analyzer as RA

PLAY_ON = 0


def dist(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def analyze(path):
    frames = RA.parse_rlg(path)
    play = [f for f in frames if f["gs"] == PLAY_ON]
    if not play:
        return None

    mins, close30, close50, ahead, ball_xs = [], 0, 0, [], []
    lone = 0
    deep = 0            # 球在进攻半场（x<110）的 PlayOn 帧数
    for f in play:
        ball = f["ball"]
        ds = [dist(f["blue"][i], ball) for i in (1, 2, 3, 4)]
        m = min(ds)
        mins.append(m)
        if m <= 30:
            close30 += 1
        if m <= 50:
            close50 += 1
        ahead.append(sum(1 for i in (1, 2, 3, 4) if f["blue"][i]["x"] < ball["x"]))
        ball_xs.append(ball["x"])
        if ball["x"] < 110:
            deep += 1
            if m > 60:
                lone += 1

    n = len(play)
    return dict(
        frames=len(frames), playon=n,
        min_d=sum(mins) / n,
        min_d_p90=sorted(mins)[int(n * 0.9)],
        close30=close30 / n * 100,
        close50=close50 / n * 100,
        ahead=sum(ahead) / n,
        ball_x=sum(ball_xs) / n,
        lone_pct=lone / deep * 100 if deep else float("nan"),
        deep_pct=deep / n * 100,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlgs", nargs="+")
    ap.add_argument("--label", action="append", default=None)
    args = ap.parse_args()

    labels = args.label or [os.path.basename(p) for p in args.rlgs]
    rows = []
    for lab, path in zip(labels, args.rlgs):
        st = analyze(path)
        if st is None:
            print(f"{lab}: 无 PlayOn 帧，跳过")
            continue
        rows.append((lab, path, st))

    header = ["标签", "PlayOn帧", "最近非门将离球cm", "P90cm", "≤30cm占比%", "≤50cm占比%",
              "越过球人数", "球均x", "球在进攻半场%", "球在前无人上前%"]
    print("| " + " | ".join(header) + " |")
    print("|" + "---|" * len(header))
    for lab, path, st in rows:
        print("| {} | {} | {:.1f} | {:.1f} | {:.1f} | {:.1f} | {:.2f} | {:.1f} | {:.1f} | {:.1f} |".format(
            lab, st["playon"], st["min_d"], st["min_d_p90"], st["close30"], st["close50"],
            st["ahead"], st["ball_x"], st["deep_pct"], st["lone_pct"]))
    for lab, path, _ in rows:
        print(f"\n{lab}: {path}")


if __name__ == "__main__":
    main()
