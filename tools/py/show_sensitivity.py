# -*- coding: utf-8 -*-
"""show_sensitivity.py — 打印灵敏度筛查结果摘要（死旋钮 / 有效旋钮排行）。"""
import json
import sys

sys.stdout.reconfigure(encoding="utf-8")
path = sys.argv[1] if len(sys.argv) > 1 else "docs/work/param_sensitivity.json"
d = json.load(open(path, encoding="utf-8"))
b, live, dead = d["baseline"], d["live"], d["dead"]
print(f"基线: net={b['net']:.3f} shots={b['shots']:.1f} gaf={b['gaf']:.2f} fb={b['fb']:.1f}")
print(f"已筛查: 有效 {len(live)}，死 {len(dead)}")
if dead:
    print("\n死旋钮（改了完全没影响，下轮搜索排除）:")
    for r in dead:
        print(f"   {r['name']:34s} {r['default']:>8.4g} → {r['loud']:>8.4g}")
if live:
    print("\n有效旋钮（按 |Δnet| 排序）:")
    for r in sorted(live, key=lambda x: -abs(x["delta"]["net"]))[:20]:
        dd = r["delta"]
        print(f"   {r['name']:34s} {r['default']:>8.4g} → {r['loud']:>8.4g}   "
              f"Δnet={dd['net']:+.2f} Δshots={dd['shots']:+.0f} Δgaf={dd['gaf']:+.2f} Δfb={dd['fb']:+.1f}")
