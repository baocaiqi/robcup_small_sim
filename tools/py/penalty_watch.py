# -*- coding: utf-8 -*-
"""
penalty_watch.py — 点球取证：盯官方 log，一见"Penalty Kick"就连拍平台画面（只读+截图，不动平台）

用法（在仓库根目录，平台开着、你要开赛时运行它，然后正常手动跑）：
    python tools/py/penalty_watch.py                      # 默认盯 C:\\Strategy\\SimuroSot5.log，拍 15s
    python tools/py/penalty_watch.py --seconds 20 --interval 1.2 --minutes 40
    python tools/py/penalty_watch.py --events Penalty,Place,Goal   # 也抓别的死球事件

产出：
    <out>/log.txt                每次触发的时刻 + 原始 log 行 + 拍了几张
    <out>/HHMMSS_eNN_kk_full.png  **全屏**截图（最可靠，场地一定在里面）
    <out>/HHMMSS_eNN_kk_win.png   平台窗口裁剪图（看得清一点）

为什么这么做：`.rlg` 只录"比赛进行中"的帧，**摆位期的球/人位置看不到**；
而点球恰恰是"平台摆位 → 主罚方去踢"的流程，所以只能靠截图看现场。
"""
import argparse
import datetime
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_match as RM


def shoot(out_dir, tag, hwnd):
    """存一张全屏 + 一张窗口裁剪"""
    full = os.path.join(out_dir, f"{tag}_full.png")
    win = os.path.join(out_dir, f"{tag}_win.png")
    RM.grab_screen(full)
    if hwnd:
        try:
            from PIL import Image
            r = RM.wt.RECT()
            RM.user32.GetWindowRect(hwnd, RM.ctypes.byref(r))
            im = Image.open(full)
            box = (max(r.left, 0), max(r.top, 0), min(r.right, im.width), min(r.bottom, im.height))
            if box[2] > box[0] and box[3] > box[1]:
                im.crop(box).save(win)
                return full, win
        except Exception as e:
            print(f"   （窗口裁剪失败：{e}）", flush=True)
    return full, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=r"C:\Strategy\SimuroSot5.log")
    ap.add_argument("--out", default=os.path.join("build", "penalty_shots"))
    ap.add_argument("--interval", type=float, default=1.5, help="连拍间隔(s)")
    ap.add_argument("--seconds", type=float, default=15.0, help="每次判罚连拍多久(s)")
    ap.add_argument("--minutes", type=float, default=40.0, help="总监听时长(s)")
    ap.add_argument("--events", default="Penalty", help="逗号分隔的关键词（默认只抓 Penalty）")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    keys = [k.strip() for k in args.events.split(",") if k.strip()]
    try:
        pos = os.path.getsize(args.log)
    except OSError:
        print(f"❌ 找不到 log：{args.log}")
        return 2
    print(f"[watch] 监听 {args.log}（从文件尾开始）；关键词={keys}；"
          f"输出={args.out}；总时长 {args.minutes:.0f} 分钟", flush=True)

    t_end = time.time() + args.minutes * 60
    episodes = 0
    while time.time() < t_end:
        try:
            if os.path.getsize(args.log) < pos:
                pos = 0                      # 日志被换/截断 → 从头读
            with open(args.log, encoding="utf-8", errors="replace") as fp:
                fp.seek(pos)
                new = fp.read()
                pos = fp.tell()
        except OSError:
            time.sleep(2.0)
            continue
        for line in new.splitlines():
            s = line.strip()
            if not s or not any(k in s for k in keys):
                continue
            episodes += 1
            stamp = datetime.datetime.now().strftime("%H%M%S")
            w = RM.find_window()
            print(f"[{episodes}] {datetime.datetime.now():%H:%M:%S} 抓到：{s[:60]}"
                  f"  → 连拍 {args.seconds:.0f}s", flush=True)
            t0, k = time.time(), 0
            while time.time() - t0 < args.seconds:
                tag = f"{stamp}_e{episodes:02d}_{k:02d}"
                f1, f2 = shoot(args.out, tag, w["hwnd"] if w else None)
                if k == 0:
                    print(f"   {'✅' if w else '⚠️ 没找到平台窗口，只能全屏拍'}  {os.path.basename(f1)}", flush=True)
                k += 1
                time.sleep(args.interval)
            with open(os.path.join(args.out, "log.txt"), "a", encoding="utf-8") as fp:
                fp.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}  第{episodes}次  {s}  拍{k}张\n")
        time.sleep(1.0)
    print(f"[watch] 结束，共抓到 {episodes} 次事件", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
