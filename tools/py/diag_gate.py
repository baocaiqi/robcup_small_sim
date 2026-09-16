# -*- coding: utf-8 -*-
"""diag_gate.py — 定位"哪个参数在范围边界上会挂哪项单元测试"。

背景：定标仿真上的防守组搜索 **182 个候选全被测试闸门否决**（可行域为空），
需要知道是哪个参数/哪项行为在拦。做法：在**当前源码参数**（已扶正的进攻组）基础上，
一次只把一个防守参数推到它的搜索范围边界，跑一遍 offline_test，记下失败项。

用法：python tools\\py\\diag_gate.py
"""
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, os.path.join("tools", "py"))
import tune_es as T

GATE = os.path.join("build", "Release", "offline_test.exe")
TMP = os.path.join("build", "scratch", "diag.txt")
os.makedirs(os.path.dirname(TMP), exist_ok=True)


def gate(pfile=None):
    cmd = [GATE] + (["--params", pfile] if pfile else [])
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    out = r.stdout or ""
    fails = [l.strip() for l in out.splitlines() if l.strip().startswith("FAIL")]
    return ("ALL TESTS PASSED" in out), fails


base, _ = T.load_baseline_params()
ok, fails = gate()
print(f"当前源码参数（已扶正进攻组）: {'ALL TESTS PASSED' if ok else fails[:1]}")

print(f"\n{'参数':<34}{'边界':>6}{'取值':>11}  首个失败项")
bad = {}
for name in T.GROUPS["defense"]:
    d = base[name]
    lo, hi = T.RANGES.get(name, (d * 0.75, d * 1.25))
    lo = max(lo, d - abs(d) * 0.6 - 0.5)
    hi = min(hi, d + abs(d) * 0.6 + 0.5)
    for tag, v in (("下界", lo), ("上界", hi)):
        with open(TMP, "w", encoding="ascii") as f:
            f.write("%s %.6g\n" % (name, v))
        ok, fails = gate(TMP)
        if not ok:
            print(f"{name:<34}{tag:>6}{v:>11.4g}  {fails[0] if fails else '（未知失败）'}")
            bad.setdefault(name, []).append(tag)
print(f"\n=== 结论：防守组 {len(T.GROUPS['defense'])} 个参数里，{len(bad)} 个在边界上会挂测试 ===")
for k, v in bad.items():
    print(f"   {k}: {'/'.join(v)}")
if not bad:
    print("   （单个参数在边界上都不挂 ⇒ 可行域为空是**多参数叠加**造成的，需要成对排查）")
