# -*- coding: utf-8 -*-
"""match_logger.py — 每场自动把 .rlg 转成轨迹 CSV + 出指标报告 + 汇总一行

背景：平台每场**自动**在 `C:\\Strategy` 写一个 `.rlg`（352B/帧 = 10 台机器人 + 球，
40Hz，只含 PlayOn 帧）。但它是二进制，要用 `rlg_analyzer.py` 转 CSV 才能分析。
本脚本把它变成"跑完一场就自动有报告"：

    监听目录里新出现的 *.rlg → 等文件稳定 → 转 CSV → 跑运动指标 + 射门指标
    → 报告写到 logs/report_<时间戳>.txt → 汇总一行追加到 logs/summary.csv

用法（在仓库根目录）：
    python tools/py/match_logger.py                 # 持续监听（默认 C:\\Strategy）
    python tools/py/match_logger.py --once          # 只处理已存在的 .rlg（补历史）
    python tools/py/match_logger.py --dir D:\\logs  # 换监听目录
    python tools/py/match_logger.py --interval 5    # 轮询间隔秒

产物：
    <仓库>/logs/csv/<rlg名>.csv     轨迹 CSV（25 列，与 sim --traj 同格式）
    <仓库>/logs/report_<时间戳>.txt 该场的完整指标报告
    <仓库>/logs/summary.csv         汇总表：时间/文件/帧数/时长/我方射门·射正率/对准率

⚠️ 未在真机验证过（本机 shell 当时挂了，写完后没跑过）；逻辑是"轮询 + 调已有脚本"，
   失败只会跳过该场并在控制台打印原因，不会破坏原始 .rlg。
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # 仓库根
LOGS = os.path.join(ROOT, "logs")


def run(cmd):
    """跑子命令，返回 (返回码, 输出文本)"""
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    p = subprocess.run(cmd, capture_output=True, env=env, cwd=ROOT)
    out = (p.stdout or b"").decode("utf-8", "replace") + (p.stderr or b"").decode("utf-8", "replace")
    return p.returncode, out


def stable(path, wait=2.0, tries=6):
    """文件大小连续两次相同才算写完（平台边打边写）"""
    last = -1
    for _ in range(tries):
        try:
            sz = os.path.getsize(path)
        except OSError:
            return False
        if sz > 0 and sz == last:
            return True
        last = sz
        time.sleep(wait)
    return False


def grab(report, pattern, default=""):
    m = re.search(pattern, report)
    return m.group(1) if m else default


def process(rlg, quiet=False):
    name = os.path.splitext(os.path.basename(rlg))[0]
    csv_dir = os.path.join(LOGS, "csv")
    os.makedirs(csv_dir, exist_ok=True)
    out_csv = os.path.join(csv_dir, name + ".csv")

    rc, out = run([sys.executable, os.path.join(HERE, "rlg_analyzer.py"), rlg, "--csv", out_csv])
    if rc != 0 or not os.path.exists(out_csv):
        print(f"[skip] {name}: 转 CSV 失败\n{out[-400:]}")
        return False

    parts = [f"===== {name} =====", out, ""]
    for label, cmd in (
        ("运动指标（速度/静止占比/蹭频/加减速）",
         [sys.executable, os.path.join(HERE, "motion_calib.py"), out_csv]),
        ("射门指标（射门/射正/机会帧/对准率）",
         [sys.executable, os.path.join(HERE, "shot_analysis.py"), out_csv, "--our", "b"]),
    ):
        rc2, o2 = run(cmd)
        parts += [f"----- {label} -----", o2, ""]

    report = "\n".join(parts)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rpt_path = os.path.join(LOGS, f"report_{stamp}_{name}.txt")
    with open(rpt_path, "w", encoding="utf-8") as fp:
        fp.write(report)

    # 汇总一行（从报告里抠关键数字；抠不到就留空，不猜）
    row = {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "rlg": os.path.basename(rlg),
        "frames": grab(report, r"总帧数:\s*(\d+)"),
        "duration_s": grab(report, r"约\s*([\d.]+)\s*秒"),
        "shots": grab(report, r"射门尝试\s*(\d+)\s*次"),
        "on_target": grab(report, r"射正\s*(\d+)\s*次"),
        "on_target_pct": grab(report, r"射正率\s*(\d+)%"),
        "chance_frames_pct": grab(report, r"机会帧.*?（([\d.]+)% 时间"),
        "align_p50_deg": grab(report, r"机头−瞄准线.*?p50=([\d.]+)°"),
        "align_le10_pct": grab(report, r"≤10°:\s*(\d+)%"),
        "still_pct": grab(report, r"静止帧占比|rest band.*?（([\d.]+)% 时间"),
    }
    sm_path = os.path.join(LOGS, "summary.csv")
    new = not os.path.exists(sm_path)
    with open(sm_path, "a", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)
    if not quiet:
        print(f"[ok] {os.path.basename(rlg)} → {os.path.relpath(out_csv, ROOT)}"
              f" | 射门 {row['shots']} 射正 {row['on_target']}({row['on_target_pct']}%)"
              f" 对准≤10° {row['align_le10_pct']}%")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"C:\Strategy", help="监听目录（平台写 .rlg 的地方）")
    ap.add_argument("--interval", type=float, default=5.0, help="轮询间隔秒")
    ap.add_argument("--once", action="store_true", help="只处理已存在的文件后退出")
    args = ap.parse_args()
    os.makedirs(LOGS, exist_ok=True)

    done_path = os.path.join(LOGS, "done.txt")
    done = set()
    if os.path.exists(done_path):
        done = set(l.strip() for l in open(done_path, encoding="utf-8"))
    if not os.path.isdir(args.dir):
        print(f"目录不存在：{args.dir}")
        return 1

    print(f"监听 {args.dir}（间隔 {args.interval}s）→ 报告写到 {os.path.relpath(LOGS, ROOT)}")
    while True:
        try:
            files = [os.path.join(args.dir, f) for f in os.listdir(args.dir)
                     if f.lower().endswith(".rlg")]
        except OSError as e:
            print(f"列目录失败：{e}")
            files = []
        for rlg in sorted(files, key=os.path.getmtime):
            base = os.path.basename(rlg)
            if base in done or os.path.getsize(rlg) == 0:
                continue
            if not stable(rlg):
                continue                      # 还在写
            if process(rlg):
                done.add(base)
                with open(done_path, "a", encoding="utf-8") as fp:
                    fp.write(base + "\n")
        if args.once:
            break
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
