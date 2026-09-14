# -*- coding: utf-8 -*-
"""record_platform.py — 录真机实测视频（抓平台球场窗口 → 直接编码成 mp4）

为什么用它：工作区里没有现成录像，而"提交材料"里的策略展示视频最好有真机实测画面。
  本机没有任何录屏软件，但我们已经装了 imageio-ffmpeg（自带静态 ffmpeg）→ 用 PIL 抓屏 +
  管道喂给 ffmpeg，能直接出 mp4，**不需要装 OBS/录屏软件**。

用法（先手动把平台跑起来、窗口拉到合适大小）：

    # 1) 看有哪些窗口（确认标题）
    python tools\\py\\record_platform.py --list

    # 2) 录 90 秒：默认抓标题含 WorldModel 的窗口（球场画面）
    python tools\\py\\record_platform.py --seconds 90 --out build\\real_match.mp4

    # 3) 想同时录比分对话框（标题 SimuroSot5）：--window SimuroSot5
    # 4) 放慢/加快：--fps 15（默认 20；帧率高对 CPU/磁盘压力大）

录完把 build\\real_match.mp4 交给 make_video.py 的 --real 参数即可融进片子：
    python tools\\py\\make_video.py --rlg <真机.rlg> --real build\\real_match.mp4
"""
import argparse
import ctypes
import ctypes.wintypes as wt
import os
import subprocess
import sys
import time

from PIL import Image, ImageGrab

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

user32 = ctypes.windll.user32
user32.SetProcessDPIAware()


def _find_ffmpeg():
    cands = ["ffmpeg", r"C:\ffmpeg\bin\ffmpeg.exe"]
    try:
        import imageio_ffmpeg
        cands.insert(0, imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:
        pass
    for c in cands:
        try:
            subprocess.run([c, "-version"], capture_output=True, check=True)
            return c
        except Exception:
            pass
    return None


def top_windows():
    out = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(hwnd, _l):
        if user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            if n:
                buf = ctypes.create_unicode_buffer(n + 1)
                user32.GetWindowTextW(hwnd, buf, n + 1)
                out.append({"hwnd": hwnd, "title": buf.value})
        return True

    user32.EnumWindows(cb, 0)
    return out


def usable(hwnd):
    """能不能用来录：可见 + 非最小化 + 在屏幕内 + 尺寸合理（幽灵/最小化窗口一律拒绝）"""
    if not user32.IsWindowVisible(hwnd):
        return False
    if user32.IsIconic(hwnd):
        return False
    l, t, r, b = rect_of(hwnd)
    if l < -10000 or t < -10000:
        return False
    return (r - l) >= 200 and (b - t) >= 150


def rect_of(hwnd):
    r = wt.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(r))
    return r.left, r.top, r.right, r.bottom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="列出可见窗口标题")
    ap.add_argument("--window", default="WorldModel", help="窗口标题关键词（默认 WorldModel=球场）")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "real_match.mp4"))
    ap.add_argument("--scale", type=int, default=1280, help="输出宽度（高度按比例）")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="等窗口出现的秒数（0=立刻；平台还没开时给个大值，如 900）")
    ap.add_argument("--shots", default=None, help="定时存全屏截图的目录（留证据/标定用）")
    ap.add_argument("--shot-every", type=float, default=15.0, help="全屏截图间隔秒")
    a = ap.parse_args()

    if a.list:
        for w in top_windows():
            print(f"  {w['hwnd']:>10}  {w['title']}")
        return 0

    tgt, t_wait = None, time.time() + max(a.wait, 0.0)
    told = False
    while True:
        for w in top_windows():
            if a.window.lower() not in w["title"].lower().replace(" ", ""):
                continue
            if usable(w["hwnd"]):
                tgt = w
                break
            if user32.IsIconic(w["hwnd"]):          # 平台被最小化 → 还原它，否则录不到画面
                user32.ShowWindow(w["hwnd"], 9)     # SW_RESTORE
                if not told:
                    print("  （平台窗口是最小化的，已自动还原以便录制）")
                    told = True
        if tgt or time.time() >= t_wait:
            break
        if int(time.time()) % 10 == 0:
            print(f"  等窗口“{a.window}”出现…（已等 {int(a.wait - (t_wait - time.time()))}s）", flush=True)
        time.sleep(1.0)
    if not tgt:
        print(f"✗ 等不到标题含“{a.window}”的窗口。先启动平台，或用 --list 看标题。")
        return 2
    l, t, r, b = rect_of(tgt["hwnd"])
    w_px, h_px = r - l, b - t
    print(f"目标窗口：{tgt['title']}  ({w_px}x{h_px} @ {l},{t})")

    ff = _find_ffmpeg()
    if not ff:
        print("✗ 没有可用 ffmpeg（pip install imageio-ffmpeg 即可）")
        return 3

    vf = f"scale={a.scale}:-2"
    cmd = [ff, "-y", "-f", "image2pipe", "-vcodec", "mjpeg", "-framerate", str(a.fps),
           "-i", "-", "-vf", vf, "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
           "-preset", "veryfast", a.out]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    print(f"录制 {a.seconds:.0f} 秒 @ {a.fps}fps → {a.out}（Ctrl+C 可提前结束）")
    n = 0
    next_shot = time.time() + 1.0
    next_rect = 0.0
    t_end = time.time() + a.seconds
    try:
        while time.time() < t_end:
            t0 = time.time()
            if time.time() >= next_rect:            # 窗口被拖动/改大小也能跟上
                r2 = rect_of(tgt["hwnd"])
                if not usable(tgt["hwnd"]):
                    print("  窗口消失了，停止录制")
                    break
                l, t, r, b = r2
                next_rect = time.time() + 2.0
            try:
                im = ImageGrab.grab(bbox=(l, t, r, b))
                im.convert("RGB").save(proc.stdin, format="JPEG", quality=88)
                n += 1
                if a.shots and time.time() >= next_shot:
                    os.makedirs(a.shots, exist_ok=True)
                    ImageGrab.grab().save(os.path.join(
                        a.shots, "t%05.1f.png" % (a.seconds - (t_end - time.time()))))
                    next_shot = time.time() + max(a.shot_every, 2.0)
            except Exception as e:                      # 窗口被移动/关闭
                print("  抓帧失败：", e)
                break
            dt = time.time() - t0
            if dt < 1.0 / a.fps:
                time.sleep(1.0 / a.fps - dt)
    except KeyboardInterrupt:
        print("  用户提前结束")
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
        proc.wait(timeout=60)
    size = os.path.getsize(a.out) / 1048576 if os.path.exists(a.out) else 0
    print(f"✓ 完成：{n} 帧，{a.out}（{size:.1f} MB）")
    print("  下一步：python tools\\py\\make_video.py --rlg <真机.rlg> --real "
          + os.path.relpath(a.out, ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
