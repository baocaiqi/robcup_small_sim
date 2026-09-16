# -*- coding: utf-8 -*-
"""list_tracked_demo.py — 用 Python 列仓库里跟踪的"演示/讲解类"文件（避开 PowerShell 编码坑）。

为什么要单独写：PowerShell 读 git 的中文路径会按本地代码页解码成乱码，
按中文关键词匹配全部失效（本次已踩第三次）。Python 用 UTF-8 解析 git 的 -z 输出最可靠。
"""
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
REF = sys.argv[1] if len(sys.argv) > 1 else "HEAD"
out = subprocess.run(["git", "ls-tree", "-r", REF, "--name-only", "-z"],
                     capture_output=True).stdout.decode("utf-8", "replace")
files = [f for f in out.split("\0") if f]
KEYS = ["mp4", "pdf", "html", "docx", "png", "稿", "说明书", "策略介绍", "队伍介绍",
        "视频", "字幕", "配音", "讲解", "提交材料"]
hits = [f for f in files if any(k in f for k in KEYS)]
print(f"=== {REF} 树里共 {len(files)} 个文件，其中演示/讲解类 {len(hits)} 个 ===")
for f in hits:
    print("  ", f)
