# -*- coding: utf-8 -*-
"""apply_params.py — 把搜索出来的参数写回源码的 TUNABLE(...) 默认值。

为什么要有这一步：比赛时平台加载的是**裸 DLL**，DLL 不能读参数文件
（这是红线：无外部依赖）。所以搜索阶段用"注入"验证方向，
**最终必须把最优值写回源码常量、重新编译**，源码才是唯一真相。

用法：
    python tools\\py\\apply_params.py docs\\work\\best_combined.txt --dry-run   # 只看会改什么
    python tools\\py\\apply_params.py docs\\work\\best_combined.txt             # 真改
    python tools\\py\\apply_params.py --revert                                 # 从 git 还原这 6 个文件

安全措施：
  · 默认先打印"旧 → 新"清单并要求 --yes 才写（--dry-run 只打印）
  · 只动 TUNABLE(...) 行，不碰其他代码
  · 改完提示重新编译 + 跑 offline_test（不自动做，避免误部署）
"""
import argparse
import io
import os
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8")

PREFIX_TO_FILE = {
    "shoot.": "src/shoot.cpp",
    "pass.": "src/pass.cpp",
    "defense.": "src/defense.cpp",
    "roles.": "src/roles.cpp",
    "strategy.": "src/strategy.cpp",
    "motion.": "src/motion.cpp",
    # 仿真物理参数（定标产出）也写回源码：这样"重跑搜索"时仿真就是定标过的，
    # 不用每次手动带 --params；sim_bench 与平台 DLL 共用同一份注册表，互不影响。
    "sim.": os.path.join("tools", "sim_bench", "sim_bench.cpp"),
}


def fmt(v):
    """整数值写成整数（避免 kMaxShootPushes 3 → 3.0 这种别扭写法）。"""
    return str(int(round(v))) if abs(v - round(v)) < 1e-9 else ("%.6g" % v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("params", nargs="?", help="参数文件（name value 每行一条）")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true", help="确认写入")
    ap.add_argument("--revert", action="store_true", help="从 git 还原 6 个源码文件")
    a = ap.parse_args()

    if a.revert:
        files = sorted(set(PREFIX_TO_FILE.values()))
        r = subprocess.run(["git", "checkout", "--"] + files, capture_output=True, text=True)
        print("✓ 已从 git 还原:", " ".join(files) if r.returncode == 0 else r.stderr)
        return 0

    if not a.params:
        print("✗ 需要参数文件（或 --revert）")
        return 2
    want = {}
    with open(a.params, encoding="utf-8") as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line:
                continue
            p = line.split()
            if len(p) == 2:
                want[p[0]] = float(p[1])
    print(f"参数文件 {a.params}: {len(want)} 条")

    # 按文件分组
    by_file = {}
    for name, v in want.items():
        for pre, path in PREFIX_TO_FILE.items():
            if name.startswith(pre):
                by_file.setdefault(path, []).append((name, v))
                break
        else:
            print(f"  ⚠️ 无法归类（前缀不认识）: {name}")

    total_changed, missing = 0, []
    for path, items in sorted(by_file.items()):
        s = io.open(path, encoding="utf-8", newline="").read()
        diffs = []
        for name, v in items:
            # 参数文件里是带前缀的名字（shoot.kMinOpen），源码里只有裸名（TUNABLE(kMinOpen, ...)）
            bare = name.split(".", 1)[1] if "." in name else name
            pat = re.compile(r"(TUNABLE\(\s*" + re.escape(bare) + r"\s*,\s*)([^)]*?)(\s*\)\s*;)")

            def repl(m, _v=v):
                old = m.group(2).strip()
                new = fmt(_v)
                if abs(float(old) - _v) > 1e-9:
                    diffs.append((name, old, new))
                return m.group(1) + new + m.group(3)
            s2, n = pat.subn(repl, s)
            if n == 0:
                missing.append(name)
            else:
                s = s2
        print(f"\n{path}: 命中 {len(items) - len([m for m in missing if m.startswith(path.split('/')[-1].split('.')[0] + '.')])} 条")
        for name, old, new in diffs:
            print(f"    {name:34s} {old:>10s} → {new}")
        total_changed += len(diffs)
        if diffs and not a.dry_run:
            if not a.yes:
                print("    （未写入：加 --yes 确认）")
                continue
            io.open(path, "w", encoding="utf-8", newline="").write(s)
            print(f"    ✓ 已写回（{len(diffs)} 处）")

    if missing:
        print("\n⚠️ 源码里没找到这些 TUNABLE（检查拼写/是否未转换）:")
        for m in missing:
            print("   ", m)
    print(f"\n合计需要改动 {total_changed} 处" + ("（dry-run，未写入）" if a.dry_run else ""))
    if total_changed and not a.dry_run:
        print("下一步：cmake --build build --config Release && build\\Release\\offline_test.exe")
    return 0


if __name__ == "__main__":
    sys.exit(main())
