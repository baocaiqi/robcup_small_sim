# -*- coding: utf-8 -*-
"""convert_to_tunable.py — 把策略源码顶层的数值常量转成 TUNABLE(...) 可注入旋钮。

只转**文件顶层**（第 0 列）的 `constexpr/const double|float|int NAME = EXPR;`：
  · 函数内部的 static 局部常量不转（要到首次调用才初始化，会盖掉注入值）
  · bool 开关不转（是开关不是旋钮）
  · 几何结构常量不转（route.cpp 的网格、kInf、kLegacyRange 等）
  · 一行多个声明（kMinX = 12.0, kMaxX = 208.0）拆成两行 TUNABLE

默认值原样保留 ⇒ 不注入时行为与改动前一致（回归闸门由 sim_bench 50 局对比保证）。
"""
import io
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8")

MODS = {
    "shoot.cpp": "shoot.",
    "pass.cpp": "pass.",
    "defense.cpp": "defense.",
    "roles.cpp": "roles.",
    "strategy.cpp": "strategy.",
    "motion.cpp": "motion.",
}
# 明确排除：几何/结构量、明显不是"旋钮"的
EXCLUDE = {"kLegacyRange", "kInf", "kCell", "kW", "kH", "kN", "kHalfDiag",
           "kWallMargin", "kMaxExpand"}

TOP = re.compile(r"^(static\s+)?const(?:expr)?\s+(double|float|int)\s+(.+)$")
INC_ANCHOR = re.compile(r'^#include\s+"simuro5/')


def split_decls(s):
    """按顶层逗号切 'a = 1.0, b = 2.0'；忽略括号/尖括号内的逗号。"""
    out, depth, cur = [], 0, ""
    for ch in s:
        if ch in "([{<":
            depth += 1
        elif ch in ")]}>":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur)
    return out


def convert(path, prefix):
    text = io.open(path, encoding="utf-8", newline="").read()
    lines = text.splitlines(keepends=True)
    out, n_conv, skipped = [], 0, []

    anchor = max((i for i, l in enumerate(lines) if INC_ANCHOR.match(l)), default=None)
    inserted = False
    for i, line in enumerate(lines):
        if anchor is not None and i == anchor + 1 and not inserted:
            out.append('#define TUNABLE_PREFIX "%s"\n' % prefix)
            out.append('#include "simuro5/tunable.hpp"\n')
            inserted = True

        body = line.rstrip("\r\n")
        eol = line[len(body):] or "\n"
        m = TOP.match(body)
        if not m:
            out.append(line)
            continue
        decls_text = m.group(3)
        # 拆掉行尾注释（TUNABLE 里保留 expr 就够了，注释单独跟到最后）
        comment = ""
        ci = decls_text.find("//")
        if ci >= 0:
            comment = decls_text[ci:]
            decls_text = decls_text[:ci]
        decls = [d.strip() for d in split_decls(decls_text) if d.strip()]
        pairs = []
        ok = True
        for d in decls:
            mm = re.match(r"^(\w+)\s*=\s*(.+)$", d)
            if not mm:
                ok = False
                break
            name, expr = mm.group(1), mm.group(2).strip()
            expr = expr.rstrip().rstrip(";").strip()      # 剥掉行尾分号（否则会跑进宏参数）
            if name in EXCLUDE or expr in ("true", "false"):
                ok = False
                break
            pairs.append((name, expr))
        if not ok or not pairs:
            skipped.append((i + 1, body.strip()[:70]))
            out.append(line)
            continue
        for k, (name, expr) in enumerate(pairs):
            tail = ("  " + comment) if (k == len(pairs) - 1 and comment) else ""
            out.append("TUNABLE(%s, %s);%s%s" % (name, expr, tail, eol))
            n_conv += 1
    io.open(path, "w", encoding="utf-8", newline="").write("".join(out))
    return n_conv, skipped


def main():
    root = "src"
    total = 0
    for f, prefix in MODS.items():
        p = os.path.join(root, f)
        n, skipped = convert(p, prefix)
        total += n
        print(f"{f:16s} 转换 {n:3d} 个常量")
        for ln, txt in skipped:
            print(f"    跳过 {f}:{ln}  {txt}")
    print(f"合计转换 {total} 个")


if __name__ == "__main__":
    main()
