# -*- coding: utf-8 -*-
"""demo_half_speed.py — 【测试仪器】把官方 demo（黄队陪练）的轮速整体乘以倍率。

用户指令变更史：
  2026-09-14「把官方的速度改慢一半」→ 倍率 0.5
  2026-09-15「官方的速度恢复到150」→ 倍率 1.5（官方原速上限 ±100，×1.5 = ±150）

为什么要改这个文件（历史教训，别再用 replace(...,1)）：
  官方 demo 有**两处**把轮速写回机器人的地方，都在 C:\\Strategy\\src\\Strategy4Yellow\\Strategy4Yellow.cpp：
    · 约 432 行 Velocity()     —— Position()/Angle() 等辅助动作走这里
    · 约 766 行 NearBound2()   —— Attack2()/Defend() 主攻防走这里
  旧版脚本只替换第一处（count=1），导致 2026-09-15 部署出去的黄队是
  「主攻防半速 + 辅助动作 1.5 倍」的混合速度，两边都没达到 ±150。
  现在改成：以**原始 demo 源码**（ORIG）为基准，把所有轮速赋值处一并替换，
  再写回 SRC —— 结果确定、可重复，不会叠加也不会漏。

源码在仓库外（C:\\Strategy\\src\\Strategy4Yellow\\），符合「官方代码不入库」的红线；
补丁内容记录在本脚本里（可 --check / --apply / --revert 复现与回退）。

用法：
    python tools\\py\\demo_half_speed.py --check      # 看当前源码两处是否都是 ×1.5、平台 DLL 哈希
    python tools\\py\\demo_half_speed.py --apply      # 打补丁 + 编译 + 备份并部署到平台黄位
    python tools\\py\\demo_half_speed.py --revert     # 还原源码 + 把平台黄位换回备份的原始 demo
"""
import argparse
import glob
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time

SRC_DIR = r"C:\Strategy\src\Strategy4Yellow"
SRC = os.path.join(SRC_DIR, "Strategy4Yellow.cpp")
SLN = os.path.join(SRC_DIR, "Strategy4Yellow.sln")
DEPLOY = r"C:\Strategy\Strategy4Yellow.dll"
BAK_DIR = r"C:\Strategy\backup_20260912"
ORIG = r"C:\Strategy\.tmp\demo_yellow\Strategy4Yellow.cpp"   # 原始 demo 源码（未打任何补丁）
OFFICIAL_DLL = r"C:\Strategy\backup_official\Strategy4Yellow.dll"

SCALE = 1.5            # 倍率：±100 → ±150
MARK = "【测试仪器】官方速度倍率"

# 原始两行 → 打补丁后的两行（两处赋值处原文完全相同，用 replace-all 一把替换）
OLD = "robot->velocityLeft = vl;\n\trobot->velocityRight = vr;"
NEW = ("robot->velocityLeft = vl * %g;    // " % SCALE + MARK +
       "（用户 2026-09-15 指令，测试借墙射门/进攻用；--revert 可还原）\n"
       "\trobot->velocityRight = vr * %g;" % SCALE)

# 回退用：把任意倍率的写法还原成 = vl; / = vr;（并去掉注入的注释），ORIG 缺失时的兜底
RE_L = re.compile(r"robot->velocityLeft = vl(?:\s*\*\s*[\d.]+)?;\s*(?://[^\n]*)?")
RE_R = re.compile(r"robot->velocityRight = vr(?:\s*\*\s*[\d.]+)?;\s*(?://[^\n]*)?")


def sha(p, n=12):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:n].upper()


def read(p=SRC):
    return open(p, encoding="gbk", errors="replace").read()


def write(s):
    open(SRC, "w", encoding="gbk", errors="replace", newline="").write(s)


def scaled_sites(s):
    """返回 [(行号, 倍率文本)] —— 所有写了 * 倍率 的轮速赋值处。"""
    out = []
    for i, line in enumerate(s.splitlines(), 1):
        m = re.search(r"robot->velocity(?:Left|Right) = v[lr]", line)
        if m:
            k = re.search(r"\*\s*([\d.]+)", line)
            out.append((i, k.group(1) if k else "1.0(未打)"))
    return out


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
    """最近的**有效**原始 demo 备份。

    历史上这里踩过坑：backup_20260912 里的 ..._demo_original_20260915_0812.dll 只有 15,360 B
    （既不是官方原件 64,512 B，也不是我们 /MT 重建的 94,720 B），是坏文件；
    照它回退会把一个加载不了的 DLL 推到黄位。所以这里加一道大小过滤：
    只认 ≥40 KB 的备份；找不到就返回 None，由调用方改用「源码重建」或官方原件。
    """
    b = [p for p in sorted(glob.glob(os.path.join(BAK_DIR, "Strategy4Yellow_demo_original_*.dll")))
         if os.path.getsize(p) >= 40 * 1024]
    return b[-1] if b else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只查看状态（等同于不带参数）")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()

    if not os.path.exists(SRC):
        print("✗ 找不到 demo 源码：", SRC)
        return 2
    s = read()
    sites = scaled_sites(s)
    plat = sha(DEPLOY) if os.path.exists(DEPLOY) else "无"
    off = sha(OFFICIAL_DLL) if os.path.exists(OFFICIAL_DLL) else "无"

    if not a.apply and not a.revert:
        print(f"源码 {SRC}")
        print(f"  轮速赋值处（应 4 处 = 2 函数 × 左右轮，均为 x{SCALE:g}）: {sites}")
        print(f"  倍率一致: {len(sites) == 4 and all(k == ('%g' % SCALE) for _, k in sites)}")
        print(f"平台黄位 {DEPLOY}: {plat}（{os.path.getsize(DEPLOY) if os.path.exists(DEPLOY) else 0} B）")
        print(f"  官方原始 demo: {off}（{os.path.getsize(OFFICIAL_DLL) if os.path.exists(OFFICIAL_DLL) else 0} B）")
        print(f"  最近备份: {newest_backup()}")
        return 0

    if a.revert:
        if os.path.exists(ORIG):
            write(read(ORIG))
            print("✓ 源码已还原为原始 demo（从 %s 覆盖）" % ORIG)
        else:
            write(RE_R.sub("robot->velocityRight = vr;", RE_L.sub("robot->velocityLeft = vl;", s)))
            print("✓ 源码已还原（正则去掉倍率与注释；未找到原件副本）")
        # 回退优先「按原始源码重建」——产物大小/工具链和平时部署的一致，最稳；
        # 重建失败才退而用官方原件 DLL（2010 老工具链编的 64 KB 版本）。
        shutil.copy2(DEPLOY, os.path.join(BAK_DIR, f"Strategy4Yellow_scaled_{time.strftime('%Y%m%d_%H%M')}.dll"))
        dll = build()
        if dll:
            shutil.copy2(dll, DEPLOY)
            print(f"✓ 平台黄位已换回原始 demo（源码重建）：现在 {sha(DEPLOY)}（{os.path.getsize(DEPLOY)} B）")
        elif os.path.exists(OFFICIAL_DLL):
            shutil.copy2(OFFICIAL_DLL, DEPLOY)
            print(f"✓ 平台黄位已换回官方原件 DLL：现在 {sha(DEPLOY)}（{os.path.getsize(DEPLOY)} B）")
        else:
            print("⚠️ 还原源码了，但重建失败且没有官方原件，平台黄位没动")
        return 0

    # apply：以 ORIG 为基准重写全部轮速赋值处（幂等，绝不叠加）
    base = read(ORIG) if os.path.exists(ORIG) else s
    if OLD not in base:
        # 兜底：原件不在，就在当前内容上把 = vl; / = vr; 归一化后统一乘倍率
        base = RE_R.sub("robot->velocityRight = vr;", RE_L.sub("robot->velocityLeft = vl;", base))
        if OLD not in base:
            print("✗ 源码里找不到轮速赋值锚点（demo 源码被改得不一样了）")
            return 3
    n = base.count(OLD)
    write(base.replace(OLD, NEW))
    print(f"✓ 源码已打补丁：{n} 处轮速赋值均乘 {SCALE:g}（= ±150）")
    print("  ", scaled_sites(read()))

    dll = build()
    if not dll:
        return 4
    os.makedirs(BAK_DIR, exist_ok=True)
    if os.path.exists(DEPLOY) and not newest_backup():
        bak = os.path.join(BAK_DIR, f"Strategy4Yellow_demo_original_{time.strftime('%Y%m%d_%H%M')}.dll")
        shutil.copy2(DEPLOY, bak)
        print("✓ 已备份当前黄位 →", os.path.basename(bak))
    shutil.copy2(dll, DEPLOY)
    print(f"✓ 已部署 x{SCALE:g} 版到黄位：{DEPLOY} = {sha(DEPLOY)}（{os.path.getsize(DEPLOY)} B）")
    print("   回退：python tools\\py\\demo_half_speed.py --revert")
    return 0


if __name__ == "__main__":
    sys.exit(main())
