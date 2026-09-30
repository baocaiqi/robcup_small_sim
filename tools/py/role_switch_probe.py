# -*- coding: utf-8 -*-
"""role_switch_probe.py — 从真机黑匣子 CSV 量化"指派换人"与"磨蹭"（纯读，不改任何东西）

为什么要它：讨论"动态主攻 / 匈牙利（最优指派）会不会让队员磨蹭"时，唯一能当证据的是真机数据。
  这个探针把 `C:\\Strategy\\hnnu_blackbox.csv` 的 R 行（我方 5 台 x,y,rot,role）与
  F 行（平台字段 + 球位球速）按场次切开，回答三件事：

  1. **换人次数与节奏**：ROLE_ACTIVE(=1) 这台机器人的身份变了没有、变了几次、平均多少帧换一次
     —— 第 94 轮动态主攻（`kActSwapMargin=20` / `kActSwapFrames=4` / `kActSwapHold=25`）的
        "换人是否来回抖"就是看它；
  2. **动态指派是不是在跑**：固定分工下 ACTIVE 永远是 1 号机器人
     （角色枚举：GOALIE=0 / ACTIVE=1 / PASSIVE=2 / ASSIST=3 / MIDFIELD=4，
       所以固定档的 R 行角色排列长这样：`[0, 1, 3, 4, 2]`）；
  3. **磨蹭帧占比**：球在动（|v|>0.5cm/帧）时，非门将 4 台里 ≥3 台几乎不动（<0.2cm/帧）的帧占比
     —— "球在动、人不动"的直接量化。

⚠️ 这个"磨蹭帧占比"只测"全队几乎静止"，测不出"换人导致的无效转向"。
   换人的代价要另用轨迹算（谁被换下、之后朝哪儿走），别拿这个数字下结论。

用法：
  python tools/py/role_switch_probe.py --last      # 只分析最后一场（最新那次真机）
  python tools/py/role_switch_probe.py             # 全部场次总表
  python tools/py/role_switch_probe.py 某个.csv     # 指定文件
"""
import os
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

CSV = r"C:\Strategy\hnnu_blackbox.csv"
ROLE_ACTIVE = 1
MOVE_EPS = 0.2               # cm/帧：低于它算"几乎不动"
BALL_EPS = 0.5               # cm/帧：高于它算"球在动"


def read_sessions(path, only_last=False):
    """流式切场次；每场只留 R/F 行原文（黑匣子格式：'# ---- new session ----' 分场）。"""
    out, cur = [], []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("# ---- new session"):
                if cur:
                    out.append(cur)
                cur = []
                continue
            if line[:2] in ("R,", "F,"):
                cur.append(line)
    if cur:
        out.append(cur)
    return out[-1:] if only_last else out


def parse_R(line):
    p = line.rstrip("\n").split(",")
    if len(p) < 3 + 4 * 5:
        return None
    xy, roles = [], []
    for i in range(5):
        b = 3 + i * 4
        xy.append((float(p[b]), float(p[b + 1])))
        roles.append(int(float(p[b + 3])))
    return int(p[1]), int(p[2]), xy, roles      # frame, is_blue, xy, roles


def parse_F(line):
    p = line.rstrip("\n").split(",")
    return int(p[1]), float(p[6]), float(p[7])   # frame, bvx, bvy


def analyse(rows):
    ours, ball = {}, {}
    for line in rows:
        if line.startswith("R,"):
            r = parse_R(line)
            if r and r[1] == 1:                  # 只看我方（is_blue=1）
                ours[r[0]] = (r[2], r[3])
        else:
            fr, bvx, bvy = parse_F(line)
            ball[fr] = (bvx ** 2 + bvy ** 2) ** 0.5
    frames = sorted(ours)
    if not frames:
        return None
    switches, active, dyn_frames = [], None, 0
    for fr in frames:
        roles = ours[fr][1]
        a = roles.index(ROLE_ACTIVE) if ROLE_ACTIVE in roles else -1
        if a != 1:
            dyn_frames += 1
        if active is None:
            active = a
        elif a != active:
            switches.append((fr, active, a, list(roles)))
            active = a
    live = stag = 0
    for k in range(1, len(frames)):
        fr, pf = frames[k], frames[k - 1]
        if fr not in ball or pf not in ball or fr - pf != 1:
            continue
        if ball[fr] < BALL_EPS:
            continue
        live += 1
        still = 0
        for i in range(1, 5):                    # 非门将
            (x1, y1), (x0, y0) = ours[fr][0][i], ours[pf][0][i]
            if ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 < MOVE_EPS:
                still += 1
        if still >= 3:
            stag += 1
    gaps = [switches[i][0] - switches[i - 1][0] for i in range(1, len(switches))]
    return dict(n=len(frames), switches=switches, dyn_pct=100.0 * dyn_frames / len(frames),
                live=live, stag_pct=(100.0 * stag / live) if live else float("nan"),
                gap=(sum(gaps) / len(gaps)) if gaps else float("nan"))


def main():
    args = sys.argv[1:]
    only_last = "--last" in args
    files = [a for a in args if not a.startswith("--")] or [CSV]
    for path in files:
        if not os.path.exists(path):
            print(f"找不到 {path}（黑匣子只在带仪器的测试版 DLL 里写）")
            continue
        S = read_sessions(path, only_last)
        print(f"{path}：{len(S)} 场次{'（只看最后一场）' if only_last else ''}\n")
        print(f"{'场次':>5}{'帧数':>8}{'换人':>6}{'平均间隔':>10}{'ACTIVE≠1号帧占比':>18}"
              f"{'磨蹭帧占比':>12}{'球活帧':>8}")
        dyn = 0
        for si, rows in enumerate(S, 1):
            r = analyse(rows)
            if not r:
                continue
            if r["switches"]:
                dyn += 1
            gap = f"{r['gap']:.0f}帧" if r["gap"] == r["gap"] else "—"
            print(f"{si:>5}{r['n']:>8}{len(r['switches']):>6}{gap:>10}"
                  f"{r['dyn_pct']:>17.1f}%{r['stag_pct']:>11.1f}%{r['live']:>8}")
            if only_last or (r["switches"] and si > len(S) - 2):
                for fr, a0, a1, roles in r["switches"][:12]:
                    print(f"      帧 {fr:>5}: ACTIVE {a0} -> {a1}    换人后角色排列 {roles}")
        print(f"\n出现过换人的场次数：{dyn} / {len(S)}"
              f"（固定分工档 ACTIVE 恒为 1 号机器人 ⇒ 0 次）")


main()
