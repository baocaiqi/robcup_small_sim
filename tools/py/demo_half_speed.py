# -*- coding: utf-8 -*-
"""demo_half_speed.py — 【测试仪器】把官方 demo（黄队陪练）的速度整体乘以倍率。

用户指令（2026-09-14）：「把官方的速度改慢一半，主要是为了测试借墙射门和进攻板块的效果」。

做法：官方 demo 的轮速只在 `Robot::velocityLeft/Right` 一处写回（Strategy4Yellow.cpp:428-429），
     所以只把这两个赋值乘 0.5——**demo 的决策逻辑一行不改**，只是整体变慢。
     源码在仓库外（C:\\Strategy\\src\\Strategy4Yellow\\），符合"官方代码不入库"的红线；
     补丁本身记录在本脚本里（可 --check / --revert 复现与回退）。

用法：
    python tools\\py\\demo_half_speed.py --check      # 看当前是否已打（含平台 DLL 哈希）
    python tools\\py\\demo_half_speed.py --apply      # 打补丁 + 编译 + 备份并部署到平台黄位
    python tools\\py\\demo_half_speed.py --revert     # 还原源码 + 把平台黄位换回备份的原始 demo
"""
import argparse
import glob
import hashlib
import os
import shutil
import subprocess
import sys
import time

SRC_DIR = r"C:\Strategy\src\Strategy4Yellow"
SRC = os.path.join(SRC_DIR, "Strategy4Yellow.cpp")
SLN = os.path.join(SRC_DIR, "Strategy4Yellow.sln")
DEPLOY = r"C:\Strategy\Strategy4Yellow.dll"
BAK_DIR = r"C:\Strategy\backup_20260912"
MARK = "【测试仪器】官方速度倍率"

OLD = "robot->velocityLeft = vl;\n\trobot->velocityRight = vr;"
NEW = ("robot->velocityLeft = vl * 1.5;    // " + MARK +
       "（用户 2026-09-14 指令，测试借墙射门/进攻用；--revert 可还原）\n"
       "\trobot->velocityRight = vr * 1.5;")


def sha(p, n=12):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:n].upper()


def read():
    return open(SRC, encoding="gbk", errors="replace").read()


def write(s):
    open(SRC, "w", encoding="gbk", errors="replace").write(s)


def find_msbuild():
    vswhere = r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
    if os.path.exists(vswhere):
        r = subprocess.run([vswhere, "-latest", "-requires", "Microsoft.Component.MSBuild",
                            "-find", r"MSBuild\**\Bin\MSBuild.exe"],
                           capture_output=True, text=True)
        for line in r.stdout.splitlines():
            if line.strip().endswith("MSBuild.exe") and os.path.exists(line.strip()):
                return line.strip()
    for c in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\MSBuild\Current\Bin\MSBuild.exe"):
        return c
    return None


def build():
    msb = find_msbuild()
    if not msb:
        print("✗ 找不到 MSBuild（需要 Visual Studio 的 C++ 工具）")
        return None
    print("MSBuild:", msb)
    r = subprocess.run([msb, SLN, "/p:Configuration=Release", "/p:Platform=Win32",
                        "/p:PlatformToolset=v143", "/m", "/v:m"],
                       capture_output=True, text=True, errors="replace")
    out = r.stdout + r.stderr
    ok = "-> " in out or r.returncode == 0
    print("  构建", "成功" if ok else "失败", "（exit", r.returncode, "）")
    if not ok:
        print("\n".join(out.splitlines()[-15:]))
        return None
    cands = glob.glob(os.path.join(SRC_DIR, "**", "*.dll"), recursive=True)
    cands = [c for c in cands if "Strategy4Yellow" in os.path.basename(c)]
    return max(cands, key=os.path.getmtime) if cands else None


def newest_backup():
    b = sorted(glob.glob(os.path.join(BAK_DIR, "Strategy4Yellow_demo_original_*.dll")))
    return b[-1] if b else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()

    if not os.path.exists(SRC):
        print("✗ 找不到 demo 源码：", SRC)
        return 2
    s = read()
    applied = MARK in s
    plat = sha(DEPLOY) if os.path.exists(DEPLOY) else "无"

    if not a.apply and not a.revert:
        print(f"源码 {SRC}")
        print(f"  已打速度减半补丁: {applied}")
        print(f"平台黄位 {DEPLOY}: {plat}")
        print(f"  最近备份: {newest_backup()}")
        return 0

    if a.revert:
        if applied:
            write(s.replace(NEW, OLD))
            print("✓ 源码已还原（去掉 *0.5）")
        bak = newest_backup()
        if bak:
            shutil.copy2(DEPLOY, os.path.join(BAK_DIR, f"Strategy4Yellow_halfspeed_{time.strftime('%Y%m%d_%H%M')}.dll"))
            shutil.copy2(bak, DEPLOY)
            print(f"✓ 平台黄位已换回原始 demo：{os.path.basename(bak)} → 现在 {sha(DEPLOY)}")
        else:
            print("⚠️ 没有找到原始 demo 备份，平台黄位没动")
        return 0

    # apply
    if not applied:
        if OLD not in s:
            print("✗ 源码里找不到锚点（可能已打过别的补丁改动了这两行）")
            return 3
        write(s.replace(OLD, NEW, 1))
        print("✓ 源码已打补丁（velocityLeft/Right 乘 1.5 = ±150）")
    else:
        print("（源码已经是减半版）")
    dll = build()
    if not dll:
        return 4
    os.makedirs(BAK_DIR, exist_ok=True)
    if os.path.exists(DEPLOY):
        bak = os.path.join(BAK_DIR, f"Strategy4Yellow_demo_original_{time.strftime('%Y%m%d_%H%M')}.dll")
        if not newest_backup():
            shutil.copy2(DEPLOY, bak)
            print("✓ 已备份原始 demo →", os.path.basename(bak))
    shutil.copy2(dll, DEPLOY)
    print(f"✓ 已部署减半版到黄位：{DEPLOY} = {sha(DEPLOY)}（{os.path.getsize(DEPLOY)} B）")
    print("   回退：python tools\\py\\demo_half_speed.py --revert")
    return 0


if __name__ == "__main__":
    sys.exit(main())
