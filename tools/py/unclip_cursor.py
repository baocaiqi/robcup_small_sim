# -*- coding: utf-8 -*-
"""
unclip_cursor.py — 解除 SimuroSot5 比赛期间的光标钳制（平台"锁鼠标"）

背景：官方平台 SimuroSot5.exe 跑比赛时会调用 Windows 的 ClipCursor 把
     鼠标光标钳在平台窗口内，导致比赛期间鼠标移不出去，无法边跑比赛
     边在这台电脑上看日志 / 改代码。

原理：光标钳制是"全局状态"，任意进程调用 user32.ClipCursor(NULL) 都能
     解除，且钳制方（平台）只在开赛/停赛时设置、不会每帧重新钳制，
     所以本工具每 50ms 解除一次即可保证鼠标一直自由。

用法：
    python unclip_cursor.py [间隔毫秒]     :: 默认 50ms；开赛前启动
    Ctrl+C 退出

注意事项：
    1. 只解除"鼠标移动范围"，平台本身不受影响。鼠标不进入平台窗口就
       不会碰到比赛——但比赛进行中不要把鼠标点进场地/机器人（可能
       选中/拖拽/误触暂停），这正是"鼠标不进比赛区间"的由来。
    2. 键盘：先点一下别的窗口（或 Alt+Tab）把输入焦点让出去再打字，
       否则按键会送进平台窗口（空格/回车可能是暂停/开球类热键）。
    3. 若 Alt+Tab 切走或最小化后发现比赛变慢/不走，说明平台帧循环依赖
       窗口可见：把平台窗口缩小放屏幕一角保持可见即可（见 docs/07 FAQ）。
    4. 极少数程序会每帧重新钳制；若仍感觉被锁，把间隔改小到 10ms 再试。
"""
import ctypes
import sys
import time

user32 = ctypes.windll.user32  # Windows only（平台本来就是 Windows 程序）


def unclip_once() -> None:
    # ClipCursor(None) → 解除全局光标钳制；返回 0 表示失败，忽略即可
    user32.ClipCursor(None)


def main() -> int:
    try:
        interval_ms = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
    except ValueError:
        print("用法: python unclip_cursor.py [间隔毫秒]  (默认 50ms)")
        return 2
    if interval_ms <= 0:
        interval_ms = 10.0

    print(f"[unclip_cursor] 启动：每 {interval_ms:g}ms 解除一次光标钳制")
    print("[unclip_cursor] 鼠标现在可以自由移出平台窗口；Ctrl+C 退出")
    try:
        while True:
            unclip_once()
            time.sleep(interval_ms / 1000.0)
    except KeyboardInterrupt:
        unclip_once()
        print("\n[unclip_cursor] 已退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
