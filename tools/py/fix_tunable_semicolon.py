# -*- coding: utf-8 -*-
"""fix_tunable_semicolon.py — 修掉 TUNABLE(name, expr;); 里多出来的分号。

根因：convert_to_tunable.py 只剥掉了行尾注释，没剥行尾 `;`，
于是 `TUNABLE(kX, 50.0;);` —— 分号跑进宏参数里，编译报 "缺少 )"。
（脚本已同步修正，本文件用于修复已经转换过的源码。）
"""
import io
import re
import sys

sys.stdout.reconfigure(encoding="utf-8")

FILES = ["src/shoot.cpp", "src/pass.cpp", "src/defense.cpp",
         "src/roles.cpp", "src/strategy.cpp", "src/motion.cpp"]

# TUNABLE(名字, 表达式;)  →  TUNABLE(名字, 表达式)
PAT = re.compile(r"(TUNABLE\(\s*\w+\s*,\s*)([^;]*?)\s*;\s*(\)\s*;)")


def main():
    total = 0
    for p in FILES:
        s = io.open(p, encoding="utf-8", newline="").read()
        s2, n = PAT.subn(lambda m: m.group(1) + m.group(2) + m.group(3), s)
        if n:
            io.open(p, "w", encoding="utf-8", newline="").write(s2)
        total += n
        print(f"{p:18s} 修 {n:3d} 处")
    print("合计", total)


if __name__ == "__main__":
    main()
