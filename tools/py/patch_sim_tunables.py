# -*- coding: utf-8 -*-
"""patch_sim_tunables.py — 给 sim_bench.cpp 打"仿真保真度"补丁（可重复执行、幂等）。

用户 2026-09-16 选定「定标包」（①常量可注入 + ②延迟/死区）。
补丁内容（7 处）：
  1. 加 `#define TUNABLE_PREFIX "sim."` + include tunable.hpp
  2. 常量区：11 个物理常量 → TUNABLE(...)（kDt / 门宽保持 constexpr）
  3. 球衰减改**两档**（真机低速段几乎不减速 0.9999，高速段 0.992~0.994）
  4. 撞墙切向摩擦**分轴向**（真机 x 墙几乎无切向损失 0.99，y 墙 0.78）
  5. 推球动量两系数 0.3/0.7 从硬编码变旋钮
  6. 机器人指令**延迟 + 轮速死区**（真机 40Hz 下指令至少晚 1 帧生效）
  7. 脚本对手速度从写死的 80/30/50/40 变旋钮（实测根因：对手慢一半不是物理）

**关键设计**：所有新旋钮的默认值 = 现在的硬编码值
（kBallDecaySlow=0.985、kDecayVref=0、kActDelay=0、kWheelDead=0、kPushKeep=0.3、
 kPushGain=0.7、对手 80/30/50/40、kWallFricX/Y=0.81）
⇒ **打完后不注入任何参数时，50 局仿真结果必须与打补丁前逐位一致（366:55）** —— 回归闸门。

用法：
    python tools\\py\\patch_sim_tunables.py            # 打补丁（先备份到 build/sim_backup）
    python tools\\py\\patch_sim_tunables.py --check     # 只检查是否已打
    python tools\\py\\patch_sim_tunables.py --revert    # 从备份还原
"""
import argparse
import io
import os
import shutil
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

SRC = os.path.join("tools", "sim_bench", "sim_bench.cpp")
BAK = os.path.join("build", "sim_backup", "sim_bench.cpp.orig")

MARK = '#define TUNABLE_PREFIX "sim."'

NEW_CONSTS = '''// ==================== 物理常量（可注入旋钮，默认值=原来的硬编码值） ====================
// 生活化比喻：这些原来是焊死的螺母，现在换成带刻度的旋钮——
//   默认仍拧在原来的位置，所以"不注入参数"时行为与打补丁前逐位一致。
//   定标脚本（tools/py/calibrate_sim.py）用真机 132 场 rlg 反推它们该拧到哪。
static constexpr double kDt = 1.0 / 40.0;                 // 40Hz（平台定死，不可调）
static constexpr double kGoalLo = 70.0, kGoalHi = 110.0;  // 门宽（规则，不可调）
TUNABLE(kSpeed, 0.9);            // 轮速→cm/s 缩放（真机实测我方 p99≈154cm/s，现在只有 90）
TUNABLE(kWheelBase, 10.0);       // 轮距 cm（决定转向灵敏度）
TUNABLE(kAccel, 300.0);          // 轮速最大加速度 cm/s²（真机 p90≈0.209cm/帧² ⇒ 量级正确）
TUNABLE(kBallDecay, 0.985);      // 球**高速段**每帧衰减（真机实测 0.992~0.994）
TUNABLE(kBallDecaySlow, 0.985);  // 球**低速段**每帧衰减（真机实测 ≈0.9999 几乎不减速）
TUNABLE(kDecayVref, 0.0);        // 速度分档阈值 cm/帧；0 = 关闭两档（默认与旧行为一致）
TUNABLE(kWallRest, 0.66);        // 撞墙法向恢复（真机新测法实测 0.451/0.449）
TUNABLE(kWallFricX, 0.81);       // x 墙切向保持（真机实测 ≈0.99 几乎无损失）
TUNABLE(kWallFricY, 0.81);       // y 墙切向保持（真机实测 0.78~0.84）
TUNABLE(kContact, 5.5);          // 球-机器人最小分离 cm（防球嵌进机器人身体）
TUNABLE(kCarryR, 9.0);           // 携带区半径 cm（略大于策略"球后 8cm 推球点"）
TUNABLE(kCarryArc, 40.0);        // 携带区前向半弧（度）
TUNABLE(kDeflect, 0.45);         // 守门员挡球反弹恢复系数
TUNABLE(kRobotR, 6.0);           // 机器人-机器人最小间距 cm
// —— 新增：真机有、仿真原先没有的两个执行环节 ——
TUNABLE(kActDelay, 0.0);         // 指令生效延迟帧数（真机至少 1 帧；0=旧行为）
TUNABLE(kWheelDead, 0.0);        // 轮速死区：小于此值的轮速命令推不动（0=旧行为）
// —— 推球动量（原来硬编码 0.3 / 0.7） ——
TUNABLE(kPushKeep, 0.3);         // 推球时保留旧球速的比例
TUNABLE(kPushGain, 0.7);         // 推球时机器人速度注入的比例
// —— 脚本对手速度（原来写死 80/30/50/40；实测"对手慢一半"的根因就在这里） ——
TUNABLE(oppChaseSpeed, 80.0);      // 追击手（远球）
TUNABLE(oppChaseNearSpeed, 30.0);  // 追击手（近球 20cm 内）
TUNABLE(oppSupportSpeed, 50.0);    // 协防
TUNABLE(oppFormationSpeed, 40.0);  // 阵型站位
TUNABLE(oppGkSpeed, 30.0);         // 门将横向 cm/s
'''

EDITS = [
    # (说明, 原文, 新文, 期望命中次数)
    ("包含 TUNABLE_PREFIX",
     '#include "simuro5/tunable.hpp"',
     MARK + '\n#include "simuro5/tunable.hpp"', 1),

    ("球衰减改两档",
     "    s.bvx *= kBallDecay; s.bvy *= kBallDecay;",
     "    // 两档衰减：真机实测低速段几乎不减速(0.9999)，高速段 0.992~0.994\n"
     "    { double bspd = std::hypot(s.bvx, s.bvy);\n"
     "      double dec = (kDecayVref > 0.0 && bspd < kDecayVref) ? kBallDecaySlow : kBallDecay;\n"
     "      s.bvx *= dec; s.bvy *= dec; }", 1),

    ("x 墙用 kWallFricX（左）",
     "else { s.bx = -s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFric; }",
     "else { s.bx = -s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFricX; }", 1),
    ("x 墙用 kWallFricX（右）",
     "else { s.bx = 440 - s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFric; }",
     "else { s.bx = 440 - s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFricX; }", 1),
    ("y 墙用 kWallFricY（下+上）",
     "if (s.by < 0) { s.by = -s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFric; }\n"
     "    if (s.by > 180) { s.by = 360 - s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFric; }",
     "if (s.by < 0) { s.by = -s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFricY; }\n"
     "    if (s.by > 180) { s.by = 360 - s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFricY; }", 1),

    ("推球动量两系数",
     "            s.bvx = s.bvx * 0.3 + rvx * 0.7;\n"
     "            s.bvy = s.bvy * 0.3 + rvy * 0.7;",
     "            s.bvx = s.bvx * kPushKeep + rvx * kPushGain;\n"
     "            s.bvy = s.bvy * kPushKeep + rvy * kPushGain;", 1),

    ("指令延迟 + 轮速死区",
     "        double maxdv = kAccel * kDt;\n"
     "        double nvl = r.vl, nvr = r.vr;",
     "        // 指令延迟：真机上本帧算出的轮速，要过 kActDelay 帧才生效\n"
     "        int D = (int)(kActDelay + 0.5); if (D > 3) D = 3; if (D < 0) D = 0;\n"
     "        r.hist_l[r.hidx] = r.vl; r.hist_r[r.hidx] = r.vr;\n"
     "        // D=0 → 读本帧刚写入那格（无延迟，保持旧行为）；D=1 → 读上一帧的命令\n"
     "        int rd = ((r.hidx - D) % 4 + 4) % 4;\n"
     "        r.hidx = (r.hidx + 1) & 3;\n"
     "        double cmd_l = r.hist_l[rd];\n"
     "        double cmd_r = r.hist_r[rd];\n"
     "        // 轮速死区：真机小轮速推不动（0 = 关闭，保持旧行为）\n"
     "        if (std::fabs(cmd_l) < kWheelDead) cmd_l = 0;\n"
     "        if (std::fabs(cmd_r) < kWheelDead) cmd_r = 0;\n"
     "        double maxdv = kAccel * kDt;\n"
     "        double nvl = cmd_l, nvr = cmd_r;", 1),

    ("对手门将横向速度",
     "    double maxdy = 30.0 * strength / 40.0;       // 30*strength cm/s 横向限速",
     "    double maxdy = oppGkSpeed * strength / 40.0;  // oppGkSpeed*strength cm/s 横向限速", 1),
    ("对手追击手速度",
     "            double spd_near = 30.0 + 25.0 * (strength - 1.0);\n"
     "            double spd = (db < 20.0) ? spd_near : (80.0 * (0.6 + 0.4 * strength));",
     "            double spd_near = oppChaseNearSpeed * (0.6 + 0.4 * strength);\n"
     "            double spd = (db < 20.0) ? spd_near : (oppChaseSpeed * (0.6 + 0.4 * strength));", 1),
    ("对手协防速度",
     "            drive(R[i], mx, my, 50);",
     "            drive(R[i], mx, my, oppSupportSpeed);", 1),
    ("对手阵型速度",
     "            drive(R[i], sx, sy, 40);",
     "            drive(R[i], sx, sy, oppFormationSpeed);", 1),

    ("SimRobot 加延迟环形缓冲",
     "struct SimRobot { double x=0, y=0, rot=0, vl=0, vr=0, pl=0, pr=0; };",
     "struct SimRobot {\n"
     "    double x=0, y=0, rot=0, vl=0, vr=0, pl=0, pr=0;\n"
     "    double hist_l[4]={0,0,0,0}, hist_r[4]={0,0,0,0};  // 指令延迟缓冲（见 kActDelay）\n"
     "    int hidx=0;\n"
     "};", 1),
]


def read(p):
    return io.open(p, encoding="utf-8", newline="").read()


def write(p, s):
    io.open(p, "w", encoding="utf-8", newline="").write(s)


def replace_const_block(s, eol):
    """常量区整段替换（从"简化物理常量"标题到 kGoalLo 行）。"""
    lines = s.splitlines(keepends=True)
    i0 = next((k for k, l in enumerate(lines) if "简化物理常量" in l or "物理常量（可注入旋钮" in l), None)
    i1 = next((k for k, l in enumerate(lines) if kGoalLo_is(l)), None)
    if i0 is None or i1 is None:
        return s, False
    return "".join(lines[:i0]) + NEW_CONSTS.replace("\n", eol) + "".join(lines[i1 + 1:]), True


def kGoalLo_is(line):
    return "kGoalLo" in line and "constexpr" in line


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    s = read(SRC)
    eol = "\r\n" if "\r\n" in s else "\n"      # 锚点/新文本都要跟随文件换行风格
    print(f"源码换行风格: {'CRLF' if eol == chr(13) + chr(10) else 'LF'}")

    if a.check:
        done = MARK in s
        print(f"补丁状态: {'已打' if done else '未打'}")
        for name, _, _, _ in EDITS:
            print(f"   {name}: 见正文替换检查")
        print("  TUNABLE 数量:", s.count("TUNABLE("))
        return 0

    if a.revert:
        if not os.path.exists(BAK):
            print("✗ 没有备份可还原:", BAK); return 2
        shutil.copy2(BAK, SRC)
        print("✓ 已还原:", SRC)
        return 0

    if MARK in s:
        print("（补丁已存在，跳过替换；如需重打请先 --revert）")
        return 0
    os.makedirs(os.path.dirname(BAK), exist_ok=True)
    shutil.copy2(SRC, BAK)
    print("✓ 已备份 →", BAK)

    s, ok = replace_const_block(s, eol)
    print(("✓ " if ok else "✗ ") + "常量区替换")
    if not ok:
        return 3
    bad = 0
    for name, old0, new0, want in EDITS:
        old = old0.replace("\n", eol)      # 锚点必须跟随文件的换行风格（否则多行锚点匹配不上）
        new = new0.replace("\n", eol)
        n = s.count(old)
        if n != want:
            print(f"✗ {name}: 命中 {n} 次（期望 {want}）—— 锚点不匹配，请人工检查")
            bad += 1
            continue
        s = s.replace(old, new)
        print(f"✓ {name}")
    if bad:
        print("有锚点没匹配上，**未写入文件**（避免写出半成品）")
        return 4
    write(SRC, s)
    print(f"✓ 已写入 {SRC}（TUNABLE 数 {s.count('TUNABLE(')}）")
    print("下一步：编译 → 回归闸门（50 局必须还是 366:55）→ 跑 calibrate_sim.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
