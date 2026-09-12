# -*- coding: utf-8 -*-
"""
stop_match.py — 收尾正在跑的真机比赛：点 Pause → 点 Close → 校验进程退出

用法：
    python tools/py/stop_match.py [--pause-only] [--wait 8]

为什么要单独一个脚本：run_match.py 里点 Start 之后，Start 的处理函数会在平台
单线程消息循环里一直跑到比赛结束，所以「点 Close」不能依赖那个调用返回。
这里用 PostMessage（异步）从不阻塞的进程里发，平台照样能收到（窗口消息循环里
会处理 Pause/Close）。

退出码：0 平台已退出；4 点了 Close 但进程还在（需人工处理）
"""
import argparse
import ctypes
import ctypes.wintypes as wt
import time

user32 = ctypes.windll.user32
BM_CLICK = 0x00F5
PROCS = ("SimuroSot5.exe", "WorldModel.exe")


def find_window():
    found = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            b = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, b, n + 1)
            c = ctypes.create_unicode_buffer(256)
            user32.GetClassNameW(hwnd, c, 256)
            if "simuro" in b.value.lower() or "simuro" in c.value.lower():
                found.append(hwnd)
        return True

    user32.EnumWindows(cb, 0)
    return found[0] if found else None


def child_buttons(hwnd, keyword):
    hits = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def cb(h, _):
        n = user32.GetWindowTextLengthW(h)
        b = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(h, b, n + 1)
        if keyword in b.value.lower():
            hits.append((h, b.value.strip()))
        return True

    user32.EnumChildWindows(hwnd, cb, 0)
    return hits


def alive():
    """平台/世界模型是否还在跑（tasklist，避免依赖 psutil）"""
    import subprocess
    out = []
    r = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        for p in PROCS:
            if line.lower().startswith('"' + p.lower() + '"'):
                out.append(line.split(",")[0].strip('"'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pause-only", action="store_true")
    ap.add_argument("--wait", type=int, default=8, help="点 Close 后等多久再校验")
    args = ap.parse_args()

    hwnd = find_window()
    if not hwnd:
        print("⚠️ 没找到 SimuroSot5 窗口（可能已退出）")
        print("进程:", alive() or "无")
        return 0

    for kw, label in (("pause", "Pause"), ("close", "Close")):
        if args.pause_only and kw == "close":
            break
        hits = child_buttons(hwnd, kw)
        if not hits:
            print(f"⚠️ 没找到 {label} 按钮")
            continue
        h, text = hits[0]
        ok = user32.PostMessageW(h, BM_CLICK, 0, 0)
        print(f"[{label}] PostMessage → {text!r} 返回 {ok}")
        time.sleep(2.5)

    time.sleep(args.wait)
    still = alive()
    print("剩余进程:", still or "无")
    return 4 if still else 0


if __name__ == "__main__":
    raise SystemExit(main())
