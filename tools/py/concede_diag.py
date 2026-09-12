# -*- coding: utf-8 -*-
"""
concede_diag.py — 丢球复盘：把每一个丢球"回放"一遍，看是怎么丢的（只读 rlg）

用法：
    python tools/py/concede_diag.py <rlg> [--window 60] [--list-frames]

口径：.rlg 只录 PlayOn 帧，进球后平台会立刻把球摆回中圈。
  所以「丢球事件」= 球先出现在我方门前/越线附近（x>200），随后跳回中圈(≈110,90)。
  对每个丢球，回溯比赛帧窗口（默认 60 帧 = 1.5s），统计：
    · 球从哪边来：最后那段的 y 走向（上/下/中路）与是否贴边墙(|y|<25 或 >155)
    · 门将：窗口内离球最近距离、是否动过（速度 >20cm/s）、离门线多远
    · 最近蓝队球员（除门将）离球距离 → 有没有人上前干扰
    · 是否紧接死球重启（窗口内出现摆位瞬移）
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlg_analyzer as RA

GOAL_X = 220.0
CENTER = (110.0, 90.0)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def analyze(path, window=60, list_frames=False):
    fr = RA.parse_rlg(path)
    n = len(fr)
    # 1) 找"跳回中圈"的帧（进球后重启）
    resets = []
    for i in range(1, n):
        b, p = fr[i]["ball"], fr[i - 1]["ball"]
        if dist((b["x"], b["y"]), CENTER) < 25.0 and dist((p["x"], p["y"]), CENTER) > 60.0:
            # 上一帧在哪 → 判断是我们丢球还是我们进球
            deep_our = p["x"] > 200.0
            deep_their = p["x"] < 20.0
            resets.append((i, "丢球" if deep_our else ("进球" if deep_their else "?")))
    print(f"===== {os.path.basename(path)} =====")
    print(f"PlayOn 帧 {n}（约 {n/40:.0f}s）；检测到球回中圈 {len(resets)} 次")

    events = [(i, kind) for i, kind in resets if kind == "丢球"]
    if not events:
        print("本场没有检测到丢球（或 rlg 未覆盖到）")
        return
    print(f"\n-- 逐个丢球（窗口 {window} 帧 ≈ {window/40:.1f}s）--")
    for k, (i, _k) in enumerate(events, 1):
        lo = max(0, i - window)
        win = fr[lo:i]
        if not win:
            continue
        # 门将
        gk_d = [dist((w["blue"][0]["x"], w["blue"][0]["y"]), (w["ball"]["x"], w["ball"]["y"])) for w in win]
        gk_speed = []
        for a, b in zip(win, win[1:]):
            gk_speed.append(dist((a["blue"][0]["x"], a["blue"][0]["y"]),
                                 (b["blue"][0]["x"], b["blue"][0]["y"])) * 40.0)
        moved = sum(1 for s in gk_speed if s > 20.0)
        # 最近的非门将蓝队球员
        df_d = []
        for w in win:
            ds = [dist((r["x"], r["y"]), (w["ball"]["x"], w["ball"]["y"])) for r in w["blue"][1:]]
            df_d.append(min(ds))
        # 球从哪边来：窗口里球的 y 走向 + 是否贴边墙
        ys = [w["ball"]["y"] for w in win]
        xs = [w["ball"]["x"] for w in win]
        wall = any(y < 25.0 or y > 155.0 for y in ys)
        side = "下边(近 y=0)" if ys[-1] < 90 else "上边(近 y=180)"
        entry_y = ys[-1]
        # 死球重启？
        tele = any(dist((a["ball"]["x"], a["ball"]["y"]), (b["ball"]["x"], b["ball"]["y"])) > 40.0
                   for a, b in zip(win, win[1:]))
        print(f"  丢球{k}: 帧{i} (~{i/40:.0f}s)  进门点 y≈{entry_y:.0f}（{side}）  "
              f"窗口内贴过边墙={'是' if wall else '否'}  紧接死球={'是' if tele else '否'}")
        # 最后 10 帧：门将相对球的位置 —— 在球与门之间(前后>0) 还是 在球旁边(横向偏大)
        last = win[-10:] if len(win) >= 10 else win
        dxs = sorted(w["blue"][0]["x"] - w["ball"]["x"] for w in last)
        dys = sorted(abs(w["blue"][0]["y"] - w["ball"]["y"]) for w in last)
        dxm, dym = dxs[len(dxs) // 2], dys[len(dys) // 2]
        print(f"        门将最后 10 帧相对球：前后 {dxm:+.0f}cm（正=在球与门之间）"
              f"，横向偏 {dym:.0f}cm   |  该侧门柱 y={70 if entry_y < 90 else 110}，"
              f"球从门柱内侧 {abs(entry_y - (70 if entry_y < 90 else 110)):.0f}cm 进")
        # 谁最后碰的球：最后 15 帧里"离球最近的机器人"属于哪一队（多数票）→ 判乌龙
        last15 = win[-15:] if len(win) >= 15 else win
        blue_near = 0
        gk_near = 0
        for w in last15:
            db = min(((r["x"] - w["ball"]["x"]) ** 2 + (r["y"] - w["ball"]["y"]) ** 2) ** 0.5
                     for r in w["blue"])
            dy = min(((r["x"] - w["ball"]["x"]) ** 2 + (r["y"] - w["ball"]["y"]) ** 2) ** 0.5
                     for r in w["yellow"])
            if db < dy:
                blue_near += 1
            gk = w["blue"][0]
            gkd = ((gk["x"] - w["ball"]["x"]) ** 2 + (gk["y"] - w["ball"]["y"]) ** 2) ** 0.5
            if gkd <= db + 0.5:
                gk_near += 1
        tag = "⚠️ 疑似乌龙（我方最后触球）" if blue_near >= len(last15) * 0.6 else "对方打进"
        print(f"        触球方：我方最近 {blue_near}/{len(last15)} 帧（其中门将最近 {gk_near} 帧）→ {tag}")
        print(f"        门将：离球最近 {min(gk_d):.0f}cm，动过 {moved}/{len(gk_speed)} 帧  "
              f"|  最近的后卫离球 {min(df_d):.0f}cm"
              f"  |  球 x 从 {min(xs):.0f} 到 {max(xs):.0f}")
    # 2) 归类统计
    print("\n-- 归类（主要失分点）--")
    cnt = {}
    for i, _k in events:
        lo = max(0, i - window)
        win = fr[lo:i]
        if not win:
            continue
        ys = [w["ball"]["y"] for w in win]
        xs = [w["ball"]["x"] for w in win]
        wall = any(y < 25.0 or y > 155.0 for y in ys)
        tele = any(dist((a["ball"]["x"], a["ball"]["y"]), (b["ball"]["x"], b["ball"]["y"])) > 40.0
                   for a, b in zip(win, win[1:]))
        gk_min = min(dist((w["blue"][0]["x"], w["blue"][0]["y"]), (w["ball"]["x"], w["ball"]["y"])) for w in win)
        df_min = min(min(dist((r["x"], r["y"]), (w["ball"]["x"], w["ball"]["y"])) for r in w["blue"][1:]) for w in win)
        keys = []
        if tele:
            keys.append("死球/重启后立刻丢")
        if wall:
            keys.append("球从边墙一侧进来")
        if gk_min > 35.0:
            keys.append(f"门将全程离球 >35cm（最近 {gk_min:.0f}cm）")
        if df_min > 40.0:
            keys.append(f"没有后卫上前（最近 {df_min:.0f}cm）")
        if not keys:
            keys.append("其它（门前混战/正面对抗）")
        for kk in keys:
            cnt[kk] = cnt.get(kk, 0) + 1
    for kk, v in sorted(cnt.items(), key=lambda kv: -kv[1]):
        print(f"  {v} 个丢球：{kk}")
    print("\n提示：同一丢球可能命中多条；优先治命中最多、且能改的那条。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlg")
    ap.add_argument("--window", type=int, default=60)
    ap.add_argument("--list-frames", action="store_true")
    args = ap.parse_args()
    analyze(args.rlg, args.window, args.list_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
