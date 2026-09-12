# -*- coding: utf-8 -*-
"""
verify_demo_discipline.py — 黄队「禁区纪律」真机验证：跑一场 + 出改前/改后对照表

用法（仓库根目录）：
    python tools/py/verify_demo_discipline.py                 # 只分析：对比 C:\\Strategy 里最新一场 vs 9/9 基线三场
    python tools/py/verify_demo_discipline.py --run-match 200 # 先自动跑一场（约 200s），再分析
    python tools/py/verify_demo_discipline.py --new <某.rlg>  # 指定"改后"日志

判据（口径来自 tools/py/referee_diag.py）：
    · .rlg 只录 PlayOn 帧 → 判罚用「摆位瞬移」（球/机器人跳变 >40cm）识别
    · 停表次数/净分钟、停表前 20 帧「黄队在我方门区」「≥2 人」「贴我方门将 <12cm」占比
改前基线（2026-09-09 三场，见 docs/06 第 45 轮）：停表 9.5 / 20.2 / 13.2 次/分钟，
    贴门将 73% / 76% / 79%。
"""
import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import constants as C
import rlg_analyzer as RA
import referee_diag as RD

DEFAULT_DIR = r"C:\Strategy"
BASELINES = [
    "20260909193234-5-DEMO Yellow-MyTeam-Blue.rlg",
    "20260909184653-5-DEMO Yellow-MyTeam-Blue.rlg",
    "20260909183346-5-DEMO Yellow-MyTeam-Blue.rlg",
]


def metrics(path, window=20, thresh=40.0):
    frames = RA.parse_rlg(path)
    n = len(frames)
    play_s = n / 40.0
    stops = []
    for i in range(1, n):
        bd = ((frames[i]["ball"]["x"] - frames[i - 1]["ball"]["x"]) ** 2 +
              (frames[i]["ball"]["y"] - frames[i - 1]["ball"]["y"]) ** 2) ** 0.5
        rd = 0.0
        for t in ("blue", "yellow"):
            for a, b in zip(frames[i][t], frames[i - 1][t]):
                rd = max(rd, ((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2) ** 0.5)
        if max(bd, rd) > thresh:
            stops.append(i)
    ge1 = ge2 = gk12 = 0
    for i in stops:
        win = frames[max(0, i - window):i]
        pk = max((RD.frame_counts(w)["y_box"] for w in win), default=0)
        gkd = min((RD.frame_counts(w)["gk_d"] for w in win), default=99.0)
        ge1 += pk >= 1
        ge2 += pk >= 2
        gk12 += gkd < 12.0
    eps = RD.episodes(frames, lambda f: RD.frame_counts(f)["y_box"] >= 2, 20)
    return dict(name=os.path.basename(path), net_s=play_s, stops=len(stops),
                per_min=len(stops) / max(play_s / 60.0, 1e-9),
                ge1=ge1, ge2=ge2, gk12=gk12, eps=len(eps))


def parse_log_games(log_path):
    """把官方 log 按 Starting Controller 分段 → [(起始时间, {事件名: 次数})]"""
    from datetime import datetime
    if not os.path.isfile(log_path):
        return []
    secs, cur = [], None
    pat_ctrl = re.compile(r"Starting Controller at (\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2}):(\d{2})")
    pat_ev = re.compile(r"\s*([A-Za-z ]+?)\s+--\s+\((\d+)\s*:\s*(\d+)\)\s+Time\s*:\s*(\d+)")
    # 角区推球犯规是**另一种行格式**（之前漏统计，会把判罚总量算少）：
    #   "FreeBall RightTop blue team Violated No pushing.  -- (1 : 0) Time : 244."
    pat_np = re.compile(r"^\s*([A-Za-z ]+?)\s+(blue|yellow)\s+team\s+Violated\s+No\s+pushing", re.I)
    with open(log_path, encoding="utf-8", errors="replace") as fp:
        for ln in fp:
            m = pat_ctrl.search(ln)
            if m:
                cur = (datetime(*(int(g) for g in m.groups())), {})
                secs.append(cur)
                continue
            m = pat_np.match(ln)
            if m and cur is not None:
                k = f"NoPush_{m.group(2).lower()}"
                cur[1][k] = cur[1].get(k, 0) + 1
                continue
            m = pat_ev.match(ln)
            if m and cur is not None:
                k = m.group(1).strip()
                cur[1][k] = cur[1].get(k, 0) + 1
    return secs


def match_log(rlg_path, games):
    """按 rlg 文件名里的时间戳找对应 log 段（±5 分钟）"""
    from datetime import datetime
    m = re.match(r"(\d{4})(\d{2})(\d{2})(\d{2})(\d{2})(\d{2})", os.path.basename(rlg_path))
    if not m or not games:
        return None
    t = datetime(*(int(g) for g in m.groups()))
    best = min(games, key=lambda g: abs((g[0] - t).total_seconds()))
    return best if abs((best[0] - t).total_seconds()) <= 300 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--new", nargs="*", default=None, help="改后 rlg（可多个；缺省取目录里最新的一场）")
    ap.add_argument("--run-match", type=int, default=0, metavar="SECONDS",
                    help="先自动跑一场（SECONDS 秒），再分析")
    args = ap.parse_args()

    if args.run_match:
        print(f"[0] 自动跑一场（{args.run_match}s）...", flush=True)
        subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_match.py"),
                        "--probe-seconds", "40", "--run-seconds", str(args.run_match)], check=False)

    news = args.new or []
    if not news:
        cands = [os.path.join(args.dir, f) for f in os.listdir(args.dir) if f.endswith(".rlg")]
        if not cands:
            print(f"❌ {args.dir} 下没有 .rlg")
            return 1
        news = [max(cands, key=os.path.getmtime)]

    rows = []
    for b in BASELINES:
        p = os.path.join(args.dir, b)
        if os.path.isfile(p):
            rows.append(("改前", metrics(p), p))
    for p in news:
        if os.path.isfile(p):
            rows.append(("改后", metrics(p), p))

    games = parse_log_games(os.path.join(args.dir, "SimuroSot5.log"))

    print("\n===== 停表（判罚/摆位）对照 =====")
    print(f"{'版本':<5}{'日志':<20}{'净比赛':>8}{'停表':>6}{'次/分':>7}{'黄进我方门区':>12}"
          f"{'门区>=2人':>10}{'贴门将<12cm':>12}{'门区>=2段':>10}{'判罚/分':>8}{'推球犯规蓝/黄':>14}")
    for tag, m, p in rows:
        g = match_log(p, games)
        if g is None:
            pen = np = "—"
        else:
            kick = sum(v for k, v in g[1].items() if k.startswith("Place"))
            pen = f"{(sum(g[1].values()) - kick) / max(m['net_s'] / 60.0, 1e-9):.1f}"
            np = f"{g[1].get('NoPush_blue', 0)}/{g[1].get('NoPush_yellow', 0)}"
        short = os.path.basename(p)[:18]
        print(f"{tag:<5}{short:<20}{m['net_s']:>7.0f}s{m['stops']:>6}{m['per_min']:>7.1f}"
              f"{m['ge1']:>8}/{m['stops']:<3}{m['ge2']:>6}/{m['stops']:<3}"
              f"{m['gk12']:>7}/{m['stops']:<3}{m['eps']:>10}{pen:>8}{np:>14}")
    print("\n（基线口径：停表 9.5 / 20.2 / 13.2 次/分钟；停表前贴门将 73% / 76% / 79%；"
          "判罚/分 = log 事件去掉进球后开球 PlaceKick 再除以净比赛分钟）")

    print("\n===== 官方 log：各场判罚事件明细 =====")
    for tag, m, p in rows:
        g = match_log(p, games)
        if g is None:
            continue
        detail = "  ".join(f"{k}={v}" for k, v in sorted(g[1].items(), key=lambda kv: -kv[1]))
        print(f"  [{tag}] {g[0]:%m-%d %H:%M:%S}  {detail or '（无事件）'}")
    print("\n结论看两处：① 停表次/分钟是否从 9.5~20.2 降到 <3；② 贴门将/门区聚集占比是否塌下来。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
