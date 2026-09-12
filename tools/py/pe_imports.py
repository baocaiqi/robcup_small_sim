# -*- coding: utf-8 -*-
"""
pe_imports.py — 列出 PE 的导入 DLL（用来查「DLL 加载失败」类问题）

用法：
    python tools/py/pe_imports.py a.dll [b.dll ...]

判据（docs/02 红线）：策略 DLL 必须**静态链接运行库(/MT)**，导入表里
    不应出现 VCRUNTIME*.dll / MSVCP*.dll / ucrtbase.dll / api-ms-win-crt-*.dll，
    否则在没装对应 VC 运行时的机器（比赛现场低配机）上 LoadLibrary 失败 → 机器人不动。

参考：官方 demo 用 VS2010(/MT) 编译，导入表只有 msvcr100 之类老运行时或完全没有。
"""
import struct
import sys


def parse(path):
    d = open(path, "rb").read()
    e = struct.unpack_from("<I", d, 0x3C)[0]
    assert d[e:e + 4] == b"PE\0\0"
    machine = struct.unpack_from("<H", d, e + 4)[0]
    nsec = struct.unpack_from("<H", d, e + 6)[0]
    optsz = struct.unpack_from("<H", d, e + 20)[0]
    opt = e + 24
    magic = struct.unpack_from("<H", d, opt)[0]
    dd = opt + (96 if magic == 0x10B else 112)
    imp_rva = struct.unpack_from("<I", d, dd + 8)[0]
    sec = opt + optsz
    sections = []
    for i in range(nsec):
        off = sec + i * 40
        va = struct.unpack_from("<I", d, off + 12)[0]
        vsz = struct.unpack_from("<I", d, off + 8)[0]
        raw = struct.unpack_from("<I", d, off + 20)[0]
        sections.append((va, vsz, raw))

    def r2o(rva):
        for va, vsz, raw in sections:
            if va <= rva < va + max(vsz, 1):
                return raw + (rva - va)
        return None

    names = []
    if imp_rva:
        o = r2o(imp_rva)
        i = 0
        while True:
            ent = d[o + i * 20: o + (i + 1) * 20]
            if len(ent) < 20 or ent == b"\0" * 20:
                break
            namerva = struct.unpack_from("<I", ent, 12)[0]
            if namerva == 0:
                break
            no = r2o(namerva)
            end = d.index(b"\0", no)
            names.append(d[no:end].decode("latin1"))
            i += 1
    return machine, names


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    bad = 0
    for p in sys.argv[1:]:
        m, names = parse(p)
        arch = {0x14C: "i386", 0x8664: "x64"}.get(m, hex(m))
        risky = [n for n in names if any(k in n.upper() for k in
                 ("VCRUNTIME", "MSVCP", "UCRTBASE", "API-MS-WIN-CRT", "MSVCR1", "MSVCP1"))]
        print(f"===== {p}  [{arch}] 导入 {len(names)} 个 =====")
        for n in names:
            flag = "  ⚠️ 动态运行库" if n in risky else ""
            print(f"    {n}{flag}")
        if risky:
            bad += 1
            print(f"  ❌ 依赖动态 VC 运行时：{risky} → 现场机器可能 LoadLibrary 失败（应改 /MT）")
        else:
            print("  ✅ 无动态 VC 运行时依赖（/MT 口径）")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
