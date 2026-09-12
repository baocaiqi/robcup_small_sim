# -*- coding: utf-8 -*-
"""
session_trend.py — 多场趋势表：一场一行，看门将/丢球模式在几场里有没有变好（只读）

用法：
    python tools/py/session_trend.py                 # 默认看 C:\\Strategy 里最新 6 场
    python tools/py/session_trend.py --last 10
    python tools/py/session_trend.py --since 20260912   # 只看某天之后的

每行：净比赛 / 比分(黄:蓝) / 丢球数 / 其中"门将在球外侧"的丢球数 / 门将在球边% / 角区静止%
      / 停表次每分 / 推球犯规(蓝/黄)
丢球口径：球先出现在我方门前(x>200) → 随后跳回中圈(110,90) 记一次丢球；
「门将在球外侧」= 丢球前 10 帧里门将 x 中位数 < 球 x（球在门将与球门之间）→ 那是挡不住的站位。
"""
import argparse
import math
import os
import re
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goalie_diag as GD
import rlg_analyzer as RA
import verify_demo_discipline as VD

CENTER = (110.0, 90.0)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def goals_conceded(fr, window=60):
    """返回 [(帧, 进门点y, 门将前后中位数, 横向偏中位数)]"""
    out = []
    for i in range(1, len(fr)):
        b, p = fr[i]["ball"], fr[i - 1]["ball"]
        if dist((b["x"], b["y"]), CENTER) < 25.0 and dist((p["x"], p["y"]), CENTER) > 60.0 \
                and p["x"] > 200.0:
            win = fr[max(0, i - window):i]
            if not win:
                continue
            last = win[-10:] if len(win) >= 10 else win
            dx = sorted(w["blue"][0]["x"] - w["ball"]["x"] for w in last)[len(last) // 2]
            dy = sorted(abs(w["blue"][0]["y"] - w["ball"]["y"]) for w in last)[len(last) // 2]
            out.append((i, win[-1]["ball"]["y"], dx, dy))
    return out


def log_scores(log_path):
    """{开始时间: (黄, 蓝)} 以及每场 NoPush 蓝/黄"""
    sec, res, push = None, {}, {}
    if not os.path.isfile(log_path):
        return res, push
    for ln in open(log_path, encoding="utf-8", errors="replace"):
        m = re.search(r"Starting Controller at (\d{4}-\d{2}-\d{2} \d\d:\d\d:\d\d)", ln)
        if m:
            sec = m.group(1)
            push[sec] = [0, 0]
            continue
        if sec is None:
            continue
        if "Violated No pushing" in ln:
            push[sec][0 if "blue" in ln else 1] += 1
            continue
        m = re.match(r"\s*[A-Za-z ]+?\s+--\s+\((\d+)\s*:\s*(\d+)\)\s+Time", ln)
        if m:
            res[sec] = (int(m.group(1)), int(m.group(2)))
    return res, push


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"C:\Strategy")
    ap.add_argument("--last", type=int, default=6)
    ap.add_argument("--since", default=None, help="YYYYMMDD，只看这之后")
    args = ap.parse_args()

    files = [os.path.join(args.dir, f) for f in os.listdir(args.dir) if f.endswith(".rlg")]
    if args.since:
        files = [f for f in files if os.path.basename(f)[:8] >= args.since]
    files.sort(key=os.path.getmtime)
    files = files[-args.last:]
    res, push = log_scores(os.path.join(args.dir, "SimuroSot5.log"))

    print(f"{'场次':<16}{'净比赛':>7}{'比分(黄:蓝)':>12}{'丢球':>5}{'门将球外侧':>11}"
          f"{'在球边':>7}{'角区静止':>9}{'停表/分':>8}{'推球犯规蓝/黄':>14}")
    tot_goals = tot_outside = 0
    for p in files:
        fr = RA.parse_rlg(p)
        net = len(fr) / 40.0
        g = goals_conceded(fr)
        outside = sum(1 for _i, _y, dx, _dy in g if dx < 0)
        gm = GD.analyze(p) or {}
        per_min = VD.metrics(p)["per_min"]
        t = os.path.basename(p)[:14]
        dt = datetime.strptime(t, "%Y%m%d%H%M%S")
        key = min(res, key=lambda k: abs((datetime.strptime(k, "%Y-%m-%d %H:%M:%S") - dt).total_seconds())) \
            if res else None
        score = f"{res[key][0]}:{res[key][1]}" if key else "?"
        np_ = f"{push[key][0]}/{push[key][1]}" if key else "?"
        print(f"{t:<16}{net:>6.0f}s{score:>12}{len(g):>5}{outside:>11}"
              f"{(gm.get('onball', float('nan')) or 0)*100:>6.0f}%"
              f"{(gm.get('guard', float('nan')) or 0)*100:>8.0f}%"
              f"{per_min:>8.1f}{np_:>14}")
        tot_goals += len(g)
        tot_outside += outside
    print(f"\n合计：丢球 {tot_goals} 个，其中**门将站在球外侧**的 {tot_outside} 个"
          f"（{tot_outside/max(tot_goals,1)*100:.0f}%）")
    print("看趋势：①『门将球外侧』占比降下来＝P3 要治的问题在变好；"
          "②『在球边%』升、『角区静止%』降＝门将更主动；③ 推球犯规别回升。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
