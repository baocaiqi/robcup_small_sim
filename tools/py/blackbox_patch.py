"""黑匣子 / 门将轨迹 诊断开关工具（2026-10-05 重写）

## 为什么重写

原来这个脚本靠**改写源码**来插黑匣子：把 `include/simuro5/blackbox.hpp` 写出来、
往 `src/dll_blue.cpp` 里插 `bb::*` 调用。副作用是——**这个"临时仪器"变成了常驻**：
`blackbox.hpp` 进了 git，`dll_blue.cpp` 的调用点一直在，于是**比赛版 DLL 无条件 40Hz 写盘**。
2026-10-05 实测 `C:\\Strategy\\goalie_trace.csv` 已涨到 **1.09 GB**、`hnnu_blackbox.csv` **105.9 MB**。

现在改成**编译期开关**（沿用仓库已有的 `branch_trace.hpp` / `SIMURO5_BRANCH_TRACE` 范式）：
  · 源码里的记录器调用点**永远保留**，但整体受 `SIMURO5_HNNU_TRACE` 保护；
  · 由 CMake 选项 `HNNU_TRACE` 控制，**默认 OFF**（比赛版）；
  · 关掉时 `bb::kEnabled == false`，每个函数第一行就返回 —— 零 I/O、零行为影响，
    且编译器会把路径字符串一起消除（已用 `grep -a` 在 DLL 里验证过）。

因此本脚本不再改写源码，只**驱动那个 CMake 选项 + 编译 + 部署**。

## 用法

  python tools\\py\\blackbox_patch.py --check             # 看当前开关状态 / CSV 多大
  python tools\\py\\blackbox_patch.py --apply             # 打开诊断 + 编译
  python tools\\py\\blackbox_patch.py --apply --deploy    # 再部署到平台蓝位（自动备份）
  python tools\\py\\blackbox_patch.py --revert            # 关掉诊断 + 编译 + 部署干净版
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

# Windows 控制台默认是 GBK，脚本里的 ✓/✗ 会抛 UnicodeEncodeError（实测 2026-09-14）
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "src", "dll_blue.cpp")
HDR = os.path.join(ROOT, "include", "simuro5", "blackbox.hpp")
CACHE = os.path.join(ROOT, "build", "CMakeCache.txt")
DEPLOY = r"C:\Strategy\Strategy4Blue.dll"
CSV = r"C:\Strategy\hnnu_blackbox.csv"
GK_CSV = r"C:\Strategy\goalie_trace.csv"

# ⚠️ PATH 里的 cmake 可能是 MinGW 版（本机实测是 winlibs 4.1.0），
#    那个编出来平台不认。优先用 VS 自带的 cmake，与 build_msvc_local.ps1 保持一致。
VS_CMAKE = (r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
            r"\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe")

MARK = "simuro5/blackbox.hpp"          # dll_blue.cpp 里调用点的判据
GATE = "SIMURO5_HNNU_TRACE"            # blackbox.hpp 里编译开关的判据


def cmake_exe():
    if os.path.exists(VS_CMAKE):
        return VS_CMAKE
    return shutil.which("cmake") or "cmake"


def read(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read().replace("\r\n", "\n")


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n} B"
        n /= 1024.0


def file_info(p):
    if not os.path.exists(p):
        return "（还没有）"
    st = os.stat(p)
    ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(st.st_mtime))
    return f"{human(st.st_size)}  mtime={ts}"


# ---------------------------------------------------------------- 状态

def source_ready():
    """源码里的调用点是否还在（在 ⇒ 开关一打开就能生效）。"""
    if not os.path.exists(HDR):
        return False, "include/simuro5/blackbox.hpp 不存在"
    if GATE not in read(HDR):
        return False, "blackbox.hpp 里没有 {0} 开关（是不是被旧版覆盖了？）".format(GATE)
    if MARK not in read(SRC):
        return False, "src/dll_blue.cpp 里没有 #include \"simuro5/blackbox.hpp\""
    return True, "源码就绪"


def cmake_trace_on():
    """读 CMakeCache 判断当前构建是否编译了诊断。返回 None = 还没配置过。"""
    if not os.path.exists(CACHE):
        return None
    for line in read(CACHE).splitlines():
        if line.startswith("HNNU_TRACE:"):
            return line.strip().endswith("=ON")
    return None


# ---------------------------------------------------------------- 动作

def configure(trace_on):
    flag = "ON" if trace_on else "OFF"
    r = subprocess.run([cmake_exe(), "-S", ".", "-B", "build", f"-DHNNU_TRACE={flag}"],
                       cwd=ROOT, capture_output=True, text=True, errors="replace")
    if r.returncode != 0:
        print((r.stdout + r.stderr)[-1200:])
        return False
    print(f"✓ 已配置 HNNU_TRACE={flag}")
    return True


def build():
    r = subprocess.run([cmake_exe(), "--build", "build", "--config", "Release",
                        "--target", "Strategy4Blue"],
                       cwd=ROOT, capture_output=True, text=True, errors="replace")
    ok = r.returncode == 0
    if not ok:
        print((r.stdout + r.stderr)[-1500:])
    return ok


def dll_src():
    return os.path.join(ROOT, "build", "bin", "Release", "Strategy4Blue.dll")


def deploy(tag):
    os.makedirs(r"C:\Strategy\backup_20260912", exist_ok=True)
    bak = rf"C:\Strategy\backup_20260912\Strategy4Blue_{tag}_{time.strftime('%Y%m%d_%H%M')}.dll"
    if os.path.exists(DEPLOY):
        shutil.copy2(DEPLOY, bak)
    shutil.copy2(dll_src(), DEPLOY)
    print(f"✓ 已部署到平台（{os.path.getsize(dll_src())}B，备份 {os.path.basename(bak)}）")
    print("   ⚠️ 必须重启平台（或重新加载队伍）才生效")


# ---------------------------------------------------------------- 命令

def cmd_check():
    ready, why = source_ready()
    on = cmake_trace_on()
    print(f"源码     : {'✓ ' + why if ready else '✗ ' + why}")
    if on is None:
        print("构建开关 : ? 还没配置过 build/（先跑一次 build_msvc_local.ps1）")
    else:
        print(f"构建开关 : HNNU_TRACE={'ON（诊断已编译进 DLL）' if on else 'OFF（比赛版，正常）'}")
    if os.path.exists(dll_src()) and on is not None:
        has = b"hnnu_blackbox.csv" in open(dll_src(), "rb").read()
        verdict = "一致" if has == on else "⚠️ 不一致！需要重新 configure + build"
        print(f"产物验证 : DLL 里{'有' if has else '没有'}诊断字符串，与开关{verdict}")
    print(f"黑匣子   : {CSV} {file_info(CSV)}")
    print(f"门将轨迹 : {GK_CSV} {file_info(GK_CSV)}")
    return 0


def cmd_apply(do_deploy):
    ready, why = source_ready()
    if not ready:
        print(f"✗ {why}")
        return 1
    if not configure(True):
        return 1
    if not build():
        print("✗ 构建失败")
        return 1
    print(f"✓ 诊断已编译进 DLL: {dll_src()}")
    if do_deploy:
        deploy("before_blackbox")
        print(f"  跑完把 {CSV} + 两场 rlg 交给我")
    else:
        print(f"  部署命令：copy \"{dll_src()}\" \"{DEPLOY}\"")
    return 0


def cmd_revert(do_deploy):
    """关掉诊断；默认顺手部署一个干净 DLL（保持旧行为）。"""
    if not configure(False):
        return 1
    if not build():
        print("✗ 重建失败")
        return 1
    print("✓ 已重建为比赛版（无诊断）")
    if do_deploy:
        deploy("after_blackbox")
    return 0


def cmd_redeploy():
    """诊断已打开时，按当前源码重建并重新部署（源码可能又改了别的东西）。"""
    ready, why = source_ready()
    if not ready:
        print(f"✗ {why}")
        return 1
    if cmake_trace_on() is not True:
        print("构建开关不是 ON → 先把诊断打开")
        return cmd_apply(True)
    if not build():
        print("✗ 构建失败")
        return 1
    deploy("before_redeploy")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="打开诊断开关并编译")
    ap.add_argument("--revert", action="store_true", help="关掉诊断开关并编译为比赛版")
    ap.add_argument("--check", action="store_true", help="看当前开关状态")
    ap.add_argument("--deploy", action="store_true", help="同时覆盖到 C:\\Strategy（自动备份）")
    ap.add_argument("--redeploy", action="store_true", help="已打开诊断时按当前源码重建并重新部署")
    a = ap.parse_args()
    if a.apply:
        return cmd_apply(a.deploy)
    if a.revert:
        return cmd_revert(a.deploy)
    if a.redeploy:
        return cmd_redeploy()
    return cmd_check()


if __name__ == "__main__":
    sys.exit(main())
