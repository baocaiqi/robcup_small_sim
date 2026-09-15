# -*- coding: utf-8 -*-
"""promote_params.py — 把验收通过的参数"扶正"：写回源码 → 编译 → 冒烟测试 →（可选）部署。

为什么要有这一步：比赛时平台加载的是**裸 DLL**，DLL 不读参数文件。
所以搜索阶段用"注入"验证方向，最终必须把最优值写回源码常量、重新编译、再部署。
这条流水线手工做容易漏步骤（尤其"忘了跑 offline_test"和"忘了备份旧 DLL"）。

用法：
    python tools\\py\\promote_params.py docs\\work\\cal_best_combined.txt            # 只看会改什么
    python tools\\py\\promote_params.py docs\\work\\cal_best_combined.txt --yes      # 真写+编译+测试
    python tools\\py\\promote_params.py docs\\work\\cal_best_combined.txt --yes --deploy   # 再部署到平台
"""
import argparse
import os
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def run(cmd, title, timeout=1800):
    print(f"\n=== {title} ===")
    r = subprocess.run(cmd, shell=True, cwd=ROOT, capture_output=True, text=True, errors="replace",
                       timeout=timeout)
    out = (r.stdout or "") + (r.stderr or "")
    tail = [l for l in out.splitlines() if l.strip()][-8:]
    for l in tail:
        print("   ", l)
    return r.returncode, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("params", help="验收通过的战略参数文件（可多个，逗号分隔）")
    ap.add_argument("--yes", action="store_true", help="真正写入源码（否则只 dry-run）")
    ap.add_argument("--deploy", action="store_true", help="编译后部署到平台蓝位（自动备份旧 DLL）")
    ap.add_argument("--skip-test", action="store_true")
    a = ap.parse_args()

    files = a.params.split(",")
    for f in files:
        if not os.path.exists(os.path.join(ROOT, f)):
            print(f"✗ 找不到参数文件: {f}")
            return 2

    # ① 写回源码（先 dry-run 给人看清单）
    for f in files:
        rc, out = run(f'python tools\\py\\apply_params.py "{f}" --dry-run', f"预览改动 {f}")
        if rc != 0:
            return 3
    if not a.yes:
        print("\n（未写入：加 --yes 真写）")
        return 0
    for f in files:
        rc, out = run(f'python tools\\py\\apply_params.py "{f}" --yes', f"写回源码 {f}")
        if rc != 0:
            return 3

    # ② 编译
    rc, out = run("cmake --build build --config Release", "编译（含 offline_test 与两个 DLL）", timeout=3600)
    if rc != 0:
        print("✗ 编译失败，未部署")
        return 4

    # ③ 冒烟测试（红线：必须 ALL TESTS PASSED 才算过）
    if not a.skip_test:
        rc, out = run("build\\Release\\offline_test.exe", "冒烟测试 offline_test")
        if "ALL TESTS PASSED" not in out:
            print("✗ offline_test 未通过，未部署（按团队铁律，测试不过不许上机）")
            return 5
        print("   ✓ ALL TESTS PASSED")

    # ④ 改了什么（便于写文档）
    run("git diff --stat src include", "本次改动的文件")

    # ⑤ 部署（可选）
    if a.deploy:
        rc, out = run("python tools\\py\\blackbox_patch.py --redeploy", "部署到平台蓝位（自动备份）")
        if rc != 0:
            return 6
        print("   ⚠️ 部署后必须重启平台（或重新加载队伍）才生效")
    else:
        print("\n（未部署：加 --deploy；或手动 copy build\\bin\\Release\\Strategy4Blue.dll C:\\Strategy\\）")

    print("\n回退命令：")
    print("  python tools\\py\\apply_params.py --revert      # 源码回退")
    print("  copy C:\\Strategy\\backup_20260912\\<最新备份> C:\\Strategy\\Strategy4Blue.dll")
    return 0


if __name__ == "__main__":
    sys.exit(main())
