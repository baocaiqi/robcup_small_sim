# -*- coding: utf-8 -*-
"""
dead_code_scan.py — 扫"没被调用的函数"（保守版：只报不删）

用法：
    python tools/py/dead_code_scan.py            # 全仓扫描（src/include/tools）
    python tools/py/dead_code_scan.py --json out.json

做法（不依赖第三方工具，纯文本 + 括号配对）：
  1. 收集 src/*.cpp、tools/**_/*.cpp、include/**/*.hpp 里**顶格**定义的函数/内联函数
     （本库风格：函数定义都从第 0 列开始）；
  2. 用括号配对切出函数体，并把注释/字符串剔掉再配对（避免花括号误判）；
  3. 在全仓里数这个名字被引用的次数，区分：
       0 次          → **完全没人用**（可删候选）
       只在测试里用  → offline_test / sim_bench 用（要问：是能力保留还是该删）
       其它          → 在用
  4. 平台导出接口（RunStrategy/SetFormerRobots/SetLaterRobots/SetBall/Set*TeamName）永远保留。

提醒：本工具只看"名字出现次数"，同名重载/宏/注释里的同名会干扰 → 删除前必须人工确认。
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC_GLOBS = ["src", "include", "tools"]
TEST_FILES = ("offline_test.cpp", "sim_bench.cpp")
KEEP = {"RunStrategy", "SetFormerRobots", "SetLaterRobots", "SetBall",
        "SetBlueTeamName", "SetYellowTeamName", "main", "DllMain"}

DEF_RE = re.compile(r"^(?:static\s+|inline\s+|constexpr\s+|virtual\s+)*"
                    r"[A-Za-z_][\w:<>,&*\s]*?[\s*&]+([A-Za-z_]\w*)\s*\(")


def strip_comments_strings(text):
    """把注释与字符串字面量替换成空格（保留换行，便于行号），用于括号配对"""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                if text[i] == "\n":
                    out.append("\n")
                i += 1
            i += 2
        elif c in "\"'":
            q = c
            i += 1
            while i < n and text[i] != q:
                if text[i] == "\\":
                    i += 1
                if text[i] == "\n":
                    out.append("\n")
                i += 1
            i += 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def collect_files():
    files = []
    for g in SRC_GLOBS:
        base = os.path.join(ROOT, g)
        for dirpath, _dirs, names in os.walk(base):
            if "build" in dirpath.replace("\\", "/").split("/"):
                continue
            for f in names:
                if f.endswith((".cpp", ".hpp", ".h")):
                    files.append(os.path.join(dirpath, f))
    return sorted(set(files))


def find_defs(path, clean):
    """返回 [(name, start_line, end_line)]（1-based 行号）"""
    lines = clean.splitlines()
    text = clean
    offsets, acc = [], 0
    for ln in lines:
        offsets.append(acc)
        acc += len(ln) + 1
    defs = []
    for idx, ln in enumerate(lines):
        m = DEF_RE.match(ln)
        if not m:
            continue
        name = m.group(1)
        if name in ("if", "for", "while", "switch", "return", "else"):
            continue
        # 找到函数体的第一个 '{'（只看本行 + 之后几行，且最多 5 行内）
        j = idx
        found = -1
        while j < min(idx + 6, len(lines)):
            for k, ch in enumerate(lines[j]):
                if ch == "{":
                    found = j
                    break
                if ch == ";":      # 声明，不是定义
                    found = -2
                    break
            if found != -1:
                break
            j += 1
        if found < 0:
            continue
        # 从 found 行起做括号配对
        start = offsets[found] + lines[found].index("{")
        depth, i = 0, start
        while i < len(text):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        if depth != 0:
            continue
        end_line = text.count("\n", 0, i) + 1
        defs.append((name, idx + 1, end_line))
    return defs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    files = collect_files()
    raw = {f: open(f, encoding="utf-8", errors="replace").read() for f in files}
    clean = {f: strip_comments_strings(raw[f]) for f in files}

    defs = []
    for f in files:
        if not f.endswith((".cpp", ".hpp")):
            continue
        for (name, s, e) in find_defs(f, clean[f]):
            defs.append((name, f, s, e))

    rows = []
    for (name, f, s, e) in defs:
        if name in KEEP:
            rows.append((name, f, s, e, -1, "平台接口/入口（永远保留）"))
            continue
        total = 0
        in_test = 0
        for g in files:
            for m in re.finditer(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"(?![A-Za-z0-9_])",
                                 clean[g]):
                line = clean[g].count("\n", 0, m.start()) + 1
                if g == f and s <= line <= e:
                    continue          # 自己定义处不算
                total += 1
                if os.path.basename(g) in TEST_FILES:
                    in_test += 1
        if total == 0:
            kind = "⚠️ 完全没人用（可删候选）"
        elif total == in_test:
            kind = "只有测试用（能力保留？）"
        else:
            kind = "在用"
        rows.append((name, f, s, e, total, kind))

    rows.sort(key=lambda r: (r[4] if r[4] >= 0 else 999, r[1]))
    print(f"扫描 {len(files)} 个文件，共 {len(rows)} 个函数定义\n")
    print(f"{'引用':>5}  {'函数':<38}{'文件':<26}{'行':>5}  {'结论'}")
    for (name, f, s, e, total, kind) in rows:
        rel = os.path.relpath(f, ROOT)
        print(f"{total if total >= 0 else '—':>5}  {name:<38}{rel:<26}{s:>5}  ({e-s+1} 行)  {kind}")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump([{"name": n, "file": os.path.relpath(f, ROOT), "start": s, "end": e,
                        "refs": t, "kind": k} for (n, f, s, e, t, k) in rows], fp,
                      ensure_ascii=False, indent=1)
        print(f"\n已写 {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
