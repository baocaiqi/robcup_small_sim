# -*- coding: utf-8 -*-
"""
switch_gk_experiment.py — 在守门员验证实验之间切换现役蓝队 DLL（一次一条命令）

用法：
    python tools/py/switch_gk_experiment.py --status              # 看现役是哪个变体
    python tools/py/switch_gk_experiment.py A                     # 只校验（不部署）
    python tools/py/switch_gk_experiment.py A --deploy            # 备份现役 → 覆盖 C:\\Strategy

实验变体（由 tools/py/patch_gk_experiments.py 生成，见 build\\gk_experiments\\）：
    base  当前工作树的原样（= 9/11 上机版，SHA1 D80176AA…）
    A     角区守卫对门将放开（只留死球期）
    B1    门将扑救/清球改 TM_PASS（不套制动包线）
    B2    门将专用制动加速度 kGkBrakeAccel=900
    C     第 47 轮远射档回退开关 kFarShotEnabled=false

安全约束：平台（SimuroSot5/WorldModel）在跑时拒绝部署；部署前校验 i386 + /MT + 导出 5 个
         与官方一致；部署时把现役 DLL 备份到 C:\\Strategy\\backup_<日期>\\。
"""
import argparse
import ctypes
import ctypes.wintypes as wt
import datetime
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pe_exports
import pe_imports

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXP_DIR = os.path.join(ROOT, "build", "gk_experiments")
DEPLOY = r"C:\Strategy\Strategy4Blue.dll"
OFFICIAL = r"C:\Strategy\backup_official\Strategy4Blue.dll"
NAMES = ["base", "A", "B1", "B2", "C", "wall", "ownfix", "wallfast"]


def variant_path(name):
    return os.path.join(EXP_DIR, f"Strategy4Blue_{name}.dll")


def sha1(path):
    import hashlib
    with open(path, "rb") as fp:
        return hashlib.sha1(fp.read()).hexdigest()


def platform_running():
    user32 = ctypes.windll.user32
    found = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            b = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, b, n + 1)
            if "simuro" in b.value.lower():
                found.append(b.value)
        return True

    user32.EnumWindows(cb, 0)
    return found


def check(path):
    """校验 i386 + 无动态 VC 运行时 + 导出与官方一致"""
    m, imports = pe_imports.parse(path)
    bad = [n for n in imports if any(k in n.upper() for k in ("VCRUNTIME", "MSVCP", "UCRTBASE", "API-MS-WIN-CRT"))]
    _, exps = pe_exports.exports(path)
    ok_arch = (m == 0x14C)
    ok_exp = (len(exps) == 5)
    print(f"  架构 i386: {'✅' if ok_arch else '❌ ' + hex(m)} | 动态运行库: "
          f"{'✅ 无' if not bad else '❌ ' + str(bad)} | 导出 {len(exps)} 个: {'✅' if ok_exp else '❌'}")
    return ok_arch and not bad and ok_exp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("exp", nargs="?", choices=NAMES)
    ap.add_argument("--deploy", action="store_true")
    ap.add_argument("--status", action="store_true")
    args = ap.parse_args()

    if not os.path.isdir(EXP_DIR):
        print(f"❌ 没有 {EXP_DIR}：先用 tools/py/patch_gk_experiments.py 生成变体")
        return 2

    cur = sha1(DEPLOY) if os.path.isfile(DEPLOY) else None
    if args.status or not args.exp:
        print(f"现役 {DEPLOY}: {cur[:12] if cur else '（不存在）'}")
        for n in NAMES:
            p = variant_path(n)
            if os.path.isfile(p):
                same = " ← 现役就是它" if cur and sha1(p) == cur else ""
                print(f"  {n:<5} {sha1(p)[:12]}  {os.path.getsize(p):>7} B{same}")
        return 0

    p = variant_path(args.exp)
    if not os.path.isfile(p):
        print(f"❌ 找不到 {p}")
        return 2
    print(f"变体 {args.exp}: {p}")
    if not check(p):
        print("❌ 校验未通过，拒绝部署")
        return 3
    if not args.deploy:
        print("（只校验；加 --deploy 才会覆盖现役 DLL）")
        return 0

    run = platform_running()
    if run:
        print(f"❌ 平台在运行（{run}），先关掉再部署")
        return 4
    bak_dir = os.path.join(r"C:\Strategy", "backup_" + datetime.date.today().strftime("%Y%m%d"))
    os.makedirs(bak_dir, exist_ok=True)
    if cur:
        ts = datetime.datetime.now().strftime("%H%M%S")
        bak = os.path.join(bak_dir, f"Strategy4Blue_before_{args.exp}_{ts}_{cur[:8]}.dll")
        shutil.copy2(DEPLOY, bak)
        print(f"已备份现役 → {bak}")
    shutil.copy2(p, DEPLOY)
    print(f"✅ 已部署 {args.exp} → {DEPLOY}（SHA1 {sha1(DEPLOY)[:12]}）")
    print("   跑完一场后：python tools\\py\\goalie_diag.py \"C:\\Strategy\\<新>.rlg\""
          " + python tools\\py\\verify_demo_discipline.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
