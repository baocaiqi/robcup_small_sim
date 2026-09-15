# -*- coding: utf-8 -*-
"""collect_workspace.py — 把平台目录（C:\\Strategy）里的东西**收进本工作区**，让工作区自包含。

为什么需要：平台的 DLL、真机日志 `.rlg`、黑匣子 CSV、官方 demo 源码都在 `C:\\Strategy`，
本工作区只放了"我们自己的代码和产出"。一旦要打包/交接/复盘，散在两处很容易漏。
这个脚本把该留的都同步到工作区（**平台那一份不动**，因为平台必须从它自己的目录加载）。

红线遵守：**官方 demo 代码只放进 `external/`，而 `external/` 在 .gitignore 里** ——
即"放在工作区"但**不进 git 仓库**，避免资格审核查重风险（AGENTS.md 原创红线第 1 条）。

用法：
    python tools\\py\\collect_workspace.py            # 先看要同步什么（不复制）
    python tools\\py\\collect_workspace.py --apply     # 真同步（只补新增/变化的文件）
    python tools\\py\\collect_workspace.py --apply --skip-logs   # 不搬 255MB 的 .rlg
"""
import argparse
import hashlib
import os
import shutil
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STRAT = r"C:\Strategy"

# (说明, 源, 目标, 是否目录, 是否大件)
JOBS = [
    ("真机比赛日志 .rlg",       os.path.join(STRAT, "*.rlg"),            "logs/rlg",              "glob", True),
    ("官方比赛日志 SimuroSot5.log", os.path.join(STRAT, "*.log"),         "logs",                  "glob", False),
    ("黑匣子 CSV",             os.path.join(STRAT, "*blackbox*.csv"),   "logs",                  "glob", True),
    ("官方原始 DLL",            os.path.join(STRAT, "backup_official"),  "external/official_dll", "dir",  False),
    ("两用干净包（蓝/黄 DLL）",   os.path.join(STRAT, "hnnu_build"),       "external/hnnu_build",   "dir",  False),
    ("平台当前部署的 DLL",       os.path.join(STRAT, "Strategy4Blue.dll"), "external/deployed",    "file", False),
    ("平台当前部署的 DLL",       os.path.join(STRAT, "Strategy4Yellow.dll"), "external/deployed",  "file", False),
    ("历史备份 DLL",            os.path.join(STRAT, "backup_20260912"),  "external/backups",      "dir",  False),
    ("官方 demo 源码（不进 git）", os.path.join(STRAT, ".tmp", "demo_yellow"), "external/demo_source", "dir", False),
]


def sha1(p, blocks=1 << 20):
    h = hashlib.sha1()
    with open(p, "rb") as f:
        while True:
            b = f.read(blocks)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def same(src, dst):
    """已一致就跳过（先比大小，再比哈希）。"""
    if not os.path.exists(dst):
        return False
    if os.path.getsize(src) != os.path.getsize(dst):
        return False
    return sha1(src) == sha1(dst)


def collect(desc, src, dst, kind, apply_, is_big, skip_logs):
    if is_big and skip_logs:
        return 0, 0, 0, "跳过（--skip-logs）"
    if kind == "glob":
        import glob
        files = sorted(glob.glob(src))
    elif kind == "file":
        files = [src] if os.path.exists(src) else []
    else:
        files = [os.path.join(r, f) for r, _, fs in os.walk(src) for f in fs] if os.path.isdir(src) else []
    if not files:
        return 0, 0, 0, "源不存在"
    os.makedirs(os.path.join(WS, dst), exist_ok=True)
    copied = skipped = 0
    total = 0
    for f in files:
        rel = os.path.relpath(f, src if kind != "glob" else os.path.dirname(src))
        target = os.path.join(WS, dst, os.path.basename(f) if kind == "glob" else rel)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        if same(f, target):
            skipped += 1
            continue
        if apply_:
            shutil.copy2(f, target)
        copied += 1
        total += os.path.getsize(f)
    return copied, skipped, total, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="真复制（默认只预览）")
    ap.add_argument("--skip-logs", action="store_true", help="跳过 .rlg / 黑匣子 CSV 这类大件")
    a = ap.parse_args()
    print(f"工作区: {WS}")
    print(f"平台目录: {STRAT}\n")
    print(f"{'项目':<26}{'新增/更新':>10}{'已一致':>8}{'体积':>12}  说明")
    grand = 0
    for desc, src, dst, kind, big in JOBS:
        c, s, tot, note = collect(desc, src, dst, kind, a.apply, big, a.skip_logs)
        grand += tot
        print(f"{desc:<26}{c:>10}{s:>8}{(f'{tot/1048576:.1f} MB' if tot else '-'):>12}  {note}"
              f"{'' if a.apply else '（预览）'}")
    print(f"\n合计{'复制' if a.apply else '需复制'} {grand/1048576:.1f} MB")
    if not a.apply:
        print("→ 加 --apply 真同步")
    else:
        print("\n提示：external/ 与 logs/ 已在 .gitignore 中（官方代码与真机数据不入库）")
        print("     platform 自己的目录 C:\\Strategy 保持不动（平台必须从那里加载）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
