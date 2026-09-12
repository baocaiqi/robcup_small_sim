"""点球逐帧取证：从 .rlg 里找"球静止在门区附近"的时段，看我们到底有没有去踢。

背景（2026-09-12 晚）：15:29 那场日志里有 4 次 "Penalty Kick Blue"（= 蓝队主罚），
比分一次没变 → 用户报"罚点球一次都没射门"。平台只在 RunStrategy 被调用时记录 .rlg，
所以这里用"球静止 ≥1 秒 + 停在门前"当作点球期的指纹。

用法：python tools/py/penalty_forensics.py "C:\\Strategy\\xxx.rlg"
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlg_analyzer as RA  # noqa: E402

FPS = 40.0
STILL_SPEED = 2.0     # cm/s，认为"球没动"
MIN_STILL = 20        # 连续帧数（=0.5 秒）
NEAR_GOAL = 130.0     # 只报"距被攻球门这么近"的静止球（罚球点实测可能远到 92cm）


def scan(path, goal_side=None):
    frames = RA.parse_rlg(path)
    n = len(frames)
    spd = [0.0] * n
    for i in range(1, n):
        b0, b1 = frames[i - 1]["ball"], frames[i]["ball"]
        spd[i] = ((b1["x"] - b0["x"]) ** 2 + (b1["y"] - b0["y"]) ** 2) ** 0.5 * FPS

    runs = []
    i = 1
    while i < n:
        if spd[i] < STILL_SPEED:
            j = i
            while j < n and spd[j] < STILL_SPEED:
                j += 1
            if j - i >= MIN_STILL:
                runs.append((i, j - 1))
            i = j
        else:
            i += 1

    print(f"=== {os.path.basename(path)} ===")
    print(f"总帧数 {n}（≈{n / FPS:.0f} 秒）  静止时段 {len(runs)} 段\n")
    rows = []
    for (a, b) in runs:
        bl = frames[a]["ball"]
        # 静止段里球到被攻球门的距离（蓝队攻左门 x=0）
        if goal_side is None or goal_side == "left":
            d_goal = bl["x"]
        else:
            d_goal = 220.0 - bl["x"]
        if d_goal > NEAR_GOAL:      # 只关心门前的静止球（点球/门球候选）
            continue
        # 静止段内我们（蓝）离球最近的距离，以及是否有过一脚（段后 30 帧内球速峰值）
        dmin, closest = 1e9, None
        for k in range(a, b + 1):
            for idx, r in enumerate(frames[k]["blue"]):
                d = ((r["x"] - bl["x"]) ** 2 + (r["y"] - bl["y"]) ** 2) ** 0.5
                if d < dmin:
                    dmin, closest = d, idx
        peak = max(spd[b:min(b + 40, n)]) if b + 1 < n else 0.0
        rows.append(dict(t0=a / FPS, dur=(b - a + 1) / FPS, x=bl["x"], y=bl["y"],
                         d_goal=d_goal, dmin=dmin, closest=closest, peak=peak))

    print(f"{'起始秒':>7} {'持续':>6} {'球位置':>15} {'距门':>6} {'我方最近':>8} {'谁':>3} {'随后球速峰值':>10}")
    for r in rows:
        print(f"{r['t0']:>7.1f} {r['dur']:>6.1f} ({r['x']:>6.1f},{r['y']:>5.1f}) "
              f"{r['d_goal']:>6.1f} {r['dmin']:>8.1f} {r['closest']:>3} {r['peak']:>10.1f}")
    if not rows:
        print("（门前没有静止 ≥1 秒的球 → 这段时间平台可能根本没进点球期）")

    # 再看整场：球出现在"距被攻球门 <70cm 且静止"以外的门前活动
    near = sum(1 for f in frames if f["ball"]["x"] < 70.0)
    print(f"\n球在左门前 70cm 内的帧数：{near}（占 {100.0 * near / n:.1f}%）")
    fast = sum(1 for v in spd if v > 100.0)
    print(f"球速 >100cm/s 的帧数：{fast}（占 {100.0 * fast / n:.1f}%）")

    # 球的瞬移点 = 平台重新摆球（进球后中圈 / 点球点 / 自由球点）。平台的死球摆位帧
    # 不写进 rlg，所以"点球期"只能靠这些瞬移点 + 我方摆位反推（2026-09-12 实测有效）。
    print(f"\n{'帧秒':>7} {'跳距':>6} {'跳前球位':>16} {'跳后球位':>16} {'跳后我方1号':>14} {'跳后我方门将':>15}")
    for i in range(1, n):
        a, b = frames[i - 1]["ball"], frames[i]["ball"]
        d = ((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2) ** 0.5
        if d <= 30.0:
            continue
        f = frames[i]
        print(f"{i / FPS:>7.1f} {d:>6.1f} ({a['x']:>6.1f},{a['y']:>5.1f}) ({b['x']:>6.1f},{b['y']:>5.1f}) "
              f"({f['blue'][1]['x']:>5.1f},{f['blue'][1]['y']:>5.1f}) ({f['blue'][0]['x']:>5.1f},{f['blue'][0]['y']:>5.1f})")


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else \
        r"C:\Strategy\20260912152913-5-DEMO Yellow-MyTeam-Blue.rlg"
    scan(p, "left")
