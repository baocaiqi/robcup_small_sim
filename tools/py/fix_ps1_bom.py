# -*- coding: utf-8 -*-
"""fix_ps1_bom.py — 给 .ps1 加 UTF-8 BOM（PowerShell 5.1 的坑，踩过两次）。

为什么需要：Windows PowerShell 5.1 读**没有 BOM** 的 .ps1 时按本地代码页（GBK）解码，
脚本里的中文注释会被解成乱码，乱码字节里若凑出引号就会**破坏语法**（报"字符串缺少结束符"）。
而编辑工具保存时常把 BOM 去掉 ⇒ 每次编辑 .ps1 之后都要补一次。

用法：python tools\\py\\fix_ps1_bom.py tools/py/*.ps1
"""
import glob
import sys

sys.stdout.reconfigure(encoding="utf-8")
BOM = b"\xef\xbb\xbf"
args = sys.argv[1:] or ["tools/py/*.ps1", "*.ps1"]
files = []
for pat in args:
    files.extend(glob.glob(pat))
if not files:
    print("没有匹配到 .ps1 文件")
for p in sorted(set(files)):
    data = open(p, "rb").read()
    if data.startswith(BOM):
        print(f"  已是 BOM: {p}")
        continue
    open(p, "wb").write(BOM + data)
    print(f"  ✓ 已加 BOM: {p}")
