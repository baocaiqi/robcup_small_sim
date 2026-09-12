# -*- coding: utf-8 -*-
"""
pe_exports.py — 列出 PE 的导出函数名（校核「导出符号与官方一致」，队伍铁律 #1）

用法：
    python tools/py/pe_exports.py a.dll [b.dll ...]
两个以上文件时自动做集合对照（差异会标出来）。
"""
import struct
import sys


def exports(path):
    d = open(path, "rb").read()
    e_lfanew = struct.unpack_from("<I", d, 0x3C)[0]
    assert d[e_lfanew:e_lfanew + 4] == b"PE\0\0", "not a PE"
    machine = struct.unpack_from("<H", d, e_lfanew + 4)[0]
    nsec = struct.unpack_from("<H", d, e_lfanew + 6)[0]
    optsz = struct.unpack_from("<H", d, e_lfanew + 20)[0]
    opt = e_lfanew + 24
    magic = struct.unpack_from("<H", d, opt)[0]
    dd = opt + (96 if magic == 0x10B else 112)          # PE32 / PE32+
    exp_rva, exp_size = struct.unpack_from("<II", d, dd)
    sec = opt + optsz
    sections = []
    for i in range(nsec):
        off = sec + i * 40
        name = d[off:off + 8].rstrip(b"\0").decode("latin1")
        va, rawsz, raw = struct.unpack_from("<III", d, off + 12)[0], struct.unpack_from("<I", d, off + 16)[0], struct.unpack_from("<I", d, off + 20)[0]
        sections.append((va, rawsz, raw))

    def rva2off(rva):
        for va, rawsz, raw in sections:
            if va <= rva < va + max(rawsz, 1):
                return raw + (rva - va)
        return None

    if exp_rva == 0:
        return machine, []
    eo = rva2off(exp_rva)
    nnames = struct.unpack_from("<I", d, eo + 24)[0]
    names_rva = struct.unpack_from("<I", d, eo + 32)[0]
    no = rva2off(names_rva)
    out = []
    for i in range(nnames):
        nr = struct.unpack_from("<I", d, no + i * 4)[0]
        o = rva2off(nr)
        end = d.index(b"\0", o)
        out.append(d[o:end].decode("latin1"))
    return machine, sorted(out)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    res = {}
    for p in sys.argv[1:]:
        m, names = exports(p)
        res[p] = names
        arch = {0x14C: "i386", 0x8664: "x64"}.get(m, hex(m))
        print(f"===== {p}  [{arch}]  导出 {len(names)} 个 =====")
        for n in names:
            print("   ", n)
    if len(res) > 1:
        paths = list(res)
        base = set(res[paths[0]])
        print("\n-- 与第一个文件对照 --")
        ok = True
        for p in paths[1:]:
            s = set(res[p])
            if s != base:
                ok = False
                print(f"  ❌ {p}: 缺少 {sorted(base - s)}  多出 {sorted(s - base)}")
            else:
                print(f"  ✅ {p}: 导出集合一致（{len(s)} 个）")
        return 0 if ok else 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
