# -*- coding: utf-8 -*-
"""
run_match.py — 自动跑一场真机验证赛（启动平台 → 前台真点击 Start → 轮询比赛时钟 → 收尾）

用法：
    python tools/py/run_match.py [--run-seconds 200] [--probe-seconds 40] [--recon-only]

重要经验（2026-09-11 第一版踩的坑）：
  · SendMessage(BM_CLICK) 会一直阻塞到 Start 的处理函数返回（整场）→ 必须用 PostMessage 或真点击；
    而且一旦平台弹出**模态对话框**（比如 DLL 加载失败），同步调用会永久卡住、比赛根本不走。
  · 所以本版：SetForegroundWindow + 真鼠标点击 Start，然后**每 5s 轮询**
    「Remaining Time / 比分 / 顶层窗口清单」——时钟不走或冒出模态框立刻能看出来。
  · 全程写全屏截图（PIL.ImageGrab）到 --out-dir，便于事后看清界面/报错。

退出码：0 正常；2 找不到 Start；3 平台没起来；5 探测期时钟没走（比赛没跑起来）
"""
import argparse
import ctypes
import ctypes.wintypes as wt
import os
import subprocess
import sys
import time

user32 = ctypes.windll.user32
gdi32 = ctypes.windll.gdi32
SW_RESTORE = 9
WM_CLOSE = 0x0010
BM_CLICK = 0x00F5
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004


def top_windows(only_visible=True):
    out = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def cb(hwnd, _):
        if not only_visible or user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            b = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, b, n + 1)
            cls = ctypes.create_unicode_buffer(256)
            user32.GetClassNameW(hwnd, cls, 256)
            r = wt.RECT()
            user32.GetWindowRect(hwnd, ctypes.byref(r))
            out.append(dict(hwnd=hwnd, title=b.value, cls=cls.value,
                            rect=(r.left, r.top, r.right, r.bottom)))
        return True

    user32.EnumWindows(cb, 0)
    return out


def children(hwnd):
    out = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def cb(h, _):
        n = user32.GetWindowTextLengthW(h)
        b = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(h, b, n + 1)
        cls = ctypes.create_unicode_buffer(256)
        user32.GetClassNameW(h, cls, 256)
        r = wt.RECT()
        user32.GetWindowRect(h, ctypes.byref(r))
        out.append(dict(hwnd=h, title=b.value.strip(), cls=cls.value,
                        visible=bool(user32.IsWindowVisible(h)),
                        rect=(r.left, r.top, r.right, r.bottom)))
        return True

    user32.EnumChildWindows(hwnd, cb, 0)
    return out


def find_window():
    for w in top_windows():
        if "simuro" in w["title"].lower() or "simuro" in w["cls"].lower():
            return w
    return None


def click_at(x, y):
    user32.SetCursorPos(int(x), int(y))
    time.sleep(0.2)
    user32.mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0)
    time.sleep(0.08)
    user32.mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0)


def find_button(kids, *keywords):
    for k in kids:
        t = (k["title"] or "").lower().strip()
        if k["cls"].lower() == "button" and any(w in t for w in keywords):
            return k
    return None


def grab_window(hwnd, path):
    from PIL import Image
    r = wt.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(r))
    w, h = r.right - r.left, r.bottom - r.top
    if w <= 0 or h <= 0:
        return None
    hdc = user32.GetWindowDC(hwnd)
    mdc = gdi32.CreateCompatibleDC(hdc)
    bmp = gdi32.CreateCompatibleBitmap(hdc, w, h)
    gdi32.SelectObject(mdc, bmp)
    user32.PrintWindow(hwnd, mdc, 1)   # 1 = PW_CLIENTONLY 之外的全窗口

    class BMIH(ctypes.Structure):
        _fields_ = [("biSize", wt.DWORD), ("biWidth", wt.LONG), ("biHeight", wt.LONG),
                    ("biPlanes", wt.WORD), ("biBitCount", wt.WORD), ("biCompression", wt.DWORD),
                    ("biSizeImage", wt.DWORD), ("biXPelsPerMeter", wt.LONG),
                    ("biYPelsPerMeter", wt.LONG), ("biClrUsed", wt.DWORD), ("biClrImportant", wt.DWORD)]

    bi = BMIH()
    bi.biSize = ctypes.sizeof(BMIH)
    bi.biWidth, bi.biHeight = w, -h
    bi.biPlanes, bi.biBitCount, bi.biCompression = 1, 32, 0
    buf = ctypes.create_string_buffer(w * h * 4)
    gdi32.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0)
    Image.frombuffer("RGBA", (w, h), buf, "raw", "BGRA", 0, 1).convert("RGB").save(path)
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(mdc)
    user32.ReleaseDC(hwnd, hdc)
    return path


def grab_screen(path):
    from PIL import ImageGrab
    ImageGrab.grab().save(path)
    return path


def status_line(win_hwnd, out_dir, tag):
    """返回一行状态：时钟 / 比分 / 是否有额外的顶层窗口（模态框）"""
    kids = children(win_hwnd)
    texts = [k["title"] for k in kids if k["title"]]
    clock = next((k["title"] for k in kids if k["title"].isdigit()), "?")
    score = next((k["title"] for k in kids if ":" in k["title"]), "?")
    others = [w["title"] for w in top_windows()
              if w["hwnd"] != win_hwnd and w["title"] and "Program Manager" not in w["title"]]
    print(f"[{tag}] 时钟={clock!r} 比分={score!r} 其它顶层窗口={others} 控件文本={texts[:8]}", flush=True)
    return clock, score, others


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=r"C:\Strategy\SimuroSot5.exe")
    ap.add_argument("--run-seconds", type=int, default=200, help="探测通过后再跑多久")
    ap.add_argument("--probe-seconds", type=int, default=40, help="开赛后探测多久判断比赛是否真的在走")
    ap.add_argument("--recon-only", action="store_true")
    ap.add_argument("--out-dir", default="build")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print("[1] 启动平台:", args.exe, flush=True)
    proc = subprocess.Popen([args.exe], cwd=os.path.dirname(args.exe))

    win = None
    for _ in range(60):
        time.sleep(0.5)
        win = find_window()
        if win:
            break
        if proc.poll() is not None:
            print("❌ 平台进程已退出 code =", proc.returncode, flush=True)
            return 3
    if not win:
        print("❌ 找不到 SimuroSot 窗口；可见顶层窗口：", flush=True)
        for w in top_windows():
            print("   ", w, flush=True)
        return 3

    print("[2] 平台窗口:", win, flush=True)
    grab_window(win["hwnd"], os.path.join(args.out_dir, "platform_00_before_start.png"))
    grab_screen(os.path.join(args.out_dir, "screen_00_before_start.png"))
    status_line(win["hwnd"], args.out_dir, "before-start")

    if args.recon_only:
        print("[recon-only] 平台保持运行，不做点击", flush=True)
        return 0

    kids = children(win["hwnd"])
    if not find_button(kids, "start"):
        print("⚠️ 没找到 Start 按钮", flush=True)
        return 2

    # 平台帧循环依赖窗口可见（docs/07 FAQ：被盖住/最小化会"比赛不走"）→
    # 先把平台挪到左上角并置顶，再前台真点击，最后校验点击是否真的落地。
    HWND_TOPMOST, HWND_NOTOPMOST, SWP_SHOWWINDOW = -1, -2, 0x40
    r = win["rect"]
    w, h = r[2] - r[0], r[3] - r[1]
    user32.SetWindowPos(win["hwnd"], HWND_TOPMOST, 8, 8, w, h, SWP_SHOWWINDOW)
    user32.ShowWindow(win["hwnd"], SW_RESTORE)
    user32.SetForegroundWindow(win["hwnd"])
    time.sleep(0.8)

    kids = children(win["hwnd"])              # 挪窗后 rect 变了，重新取
    btn = find_button(kids, "start")
    r = btn["rect"]
    print(f"[3] 前台真点击 Start @ ({int((r[0]+r[2])/2)},{int((r[1]+r[3])/2)})", flush=True)
    click_at((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
    time.sleep(2.5)

    # 校验是否真的开赛：轮询「时钟」控件（按钮文字不会立刻变，不能据此判失败！）。
    # 2026-09-11 踩坑：真点击其实生效了，误判成"没生效"又 PostMessage 一次 Start
    #   → 平台同时收到两次开赛，几十秒后自己退出且不落 .rlg（白跑一场）。
    ok = False
    for _ in range(10):
        time.sleep(1.0)
        clk = next((k["title"].strip() for k in children(win["hwnd"])
                    if k["title"].strip().isdigit()), None)
        if clk is not None and int(clk) < 300:
            ok = True
            break
    if not ok:
        b = find_button(children(win["hwnd"]), "start")
        if b:
            print("[3b] 时钟未动且 Start 仍在 → 改用 PostMessage(BM_CLICK)", flush=True)
            user32.PostMessageW(b["hwnd"], BM_CLICK, 0, 0)
            time.sleep(2.0)

    # ---- 探测期：时钟必须往下走 ----
    time.sleep(3)
    clocks = []
    t0 = time.time()
    i = 0
    while time.time() - t0 < args.probe_seconds:
        clock, score, others = status_line(win["hwnd"], args.out_dir, f"probe{i}")
        if clock.isdigit():
            clocks.append(int(clock))
        if others:
            grab_screen(os.path.join(args.out_dir, f"screen_probe{i}_modal.png"))
        if i % 2 == 0:
            grab_window(win["hwnd"], os.path.join(args.out_dir, f"platform_probe{i}.png"))
        i += 1
        time.sleep(5)

    moved = len(clocks) >= 2 and clocks[-1] < clocks[0]
    print(f"[4] 探测结果：时钟采样={clocks} → {'✅ 在走' if moved else '❌ 没走'}", flush=True)
    if not moved:
        print("[!] 比赛没跑起来（时钟不动）——截图已存，人工看 screen_probe*_modal.png", flush=True)
        grab_screen(os.path.join(args.out_dir, "screen_fail_fullscreen.png"))
        return 5

    # ---- 正式跑 ----
    t0 = time.time()
    i = 0
    while time.time() - t0 < args.run_seconds:
        time.sleep(15)
        i += 1
        clock, score, others = status_line(win["hwnd"], args.out_dir, f"run{i}")
        if i % 4 == 0:
            grab_window(win["hwnd"], os.path.join(args.out_dir, f"platform_run{i}.png"))

    # ---- 收尾：Pause → Close（PostMessage 异步，不阻塞）----
    for kw in ("pause", "close"):
        b = find_button(children(win["hwnd"]), kw)
        if b:
            user32.PostMessageW(b["hwnd"], BM_CLICK, 0, 0)
            print(f"[5] PostMessage {kw} → {b['title']!r}", flush=True)
            time.sleep(2.5)
    try:      # 把平台窗口放回普通层，别一直置顶
        user32.SetWindowPos(win["hwnd"], -2, 8, 8, 0, 0, 0x0001 | 0x0002 | 0x0040)
    except Exception:
        pass
    time.sleep(5)
    print("[done] 平台进程 code =", proc.poll(), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
