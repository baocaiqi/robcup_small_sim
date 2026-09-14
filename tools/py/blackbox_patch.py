"""黑匣子变体工具（仅测试用；比赛版绝对不带）—— 2026-09-14 用户指令"开始"

目的：回答 5 个只能靠**平台原始字段**才能答的问题（rlg 里 gs/whos 恒 0，答不了）：
  ① 踢球人"站球后"检查多严（我们写 49.4，平台留 43.5）
  ② 守门员贴门线有没有硬约束
  ③ 执行期有没有隐藏计时
  ④ "禁区 4 人"的禁区尺寸（纯数据分析，本工具顺带提供帧数据）
  ⑤ 执行期 gameState / whosBall 到底是什么值、是不是只在摆位回调里给真信息

做法（纯观察，**不改任何决策逻辑**）：往 dll_blue.cpp 里插一个 header-only 记录器，
  每帧写一行 F 行、每次摆位回调写一行 P/B 行 → C:\\Strategy\\hnnu_blackbox.csv

用法：
  python tools/py/blackbox_patch.py --apply          # 打补丁 + 构建变体 DLL
  python tools/py/blackbox_patch.py --apply --deploy # 再覆盖到 C:\\Strategy（自动备份）
  python tools/py/blackbox_patch.py --check          # 看当前是否已打
  python tools/py/blackbox_patch.py --revert         # 还原源码 + 重建 + 还原平台 DLL
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

# Windows 控制台默认是 GBK，脚本里的 ✓/✗ 会抛 UnicodeEncodeError（实测 2026-09-14）
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "src", "dll_blue.cpp")
HDR = os.path.join(ROOT, "include", "simuro5", "blackbox.hpp")
BAK_DIR = os.path.join(ROOT, "build", "blackbox_backup")
VAR_DIR = os.path.join(ROOT, "build", "blackbox")
DEPLOY = r"C:\Strategy\Strategy4Blue.dll"
CSV = r"C:\Strategy\hnnu_blackbox.csv"
MARK = "simuro5/blackbox.hpp"          # 打补丁的判据

HEADER = r'''// blackbox.hpp — 临时测量仪器（仅测试用，比赛版不带）
// 每帧写一行 F 行 + 每次摆位回调写 P/B 行到 C:\\Strategy\\hnnu_blackbox.csv
// 绝不影响决策：文件打不开就静默放弃，最多试 3 次。
#ifndef SIMURO5_BLACKBOX_HPP
#define SIMURO5_BLACKBOX_HPP
#include <cstdio>
#include "simuro5/simuro_interface.hpp"

namespace simuro5 { namespace bb {
struct State { std::FILE *fp = nullptr; long frame = 0; int fails = 0; };
inline State &st() { static State s; return s; }        // 全程序唯一实例（inline + 局部 static）
inline std::FILE *fp() {
    State &s = st();
    if (!s.fp && s.fails < 3) {
        s.fp = std::fopen("C:\\Strategy\\hnnu_blackbox.csv", "a");
        if (s.fp) std::fprintf(s.fp, "# ---- new session ----\n");
        else ++s.fails;
    }
    return s.fp;
}
// F 行：平台原始字段 + 我们的判断
inline void frame(long gs, long whos, double bx, double by, double bvx, double bvy,
                  int we_have_ball, int in_penalty, double gk_x, double active_x) {
    std::FILE *f = fp(); if (!f) return;
    State &s = st(); ++s.frame;
    std::fprintf(f, "F,%ld,%ld,%ld,%.2f,%.2f,%.3f,%.3f,%d,%d,%.2f,%.2f\n",
                 s.frame, gs, whos, bx, by, bvx, bvy, we_have_ball, in_penalty, gk_x, active_x);
    if ((s.frame % 40) == 0) std::fflush(f);
}
// P 行：我们**写进去**的摆位（在 formation_* 之后调用，记的就是我们写出的值）
inline void placement(const char *tag, long gs, const Robot *r, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "P,%ld,%s,%ld", st().frame, tag, gs);
    for (int i = 0; i < n; ++i) std::fprintf(f, ",%.2f,%.2f", r[i].pos.x, r[i].pos.y);
    std::fprintf(f, "\n"); std::fflush(f);
}
inline void setball(long gs, double x, double y) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "B,%ld,%.2f,%.2f,%ld\n", st().frame, x, y, gs); std::fflush(f);
}
// R 行：我方 5 台的 (x,y,rot,role) —— 供"抓屏叠加"工具把画面里的色块对回编号
//   role 在 wm.role[] 里（固定分工：0=门将 1=主攻 2=助攻 3=中场 4=后卫，见 roles.hpp）
inline void robots(long is_blue, const double *xs, const double *ys, const double *rots,
                   const int *roles, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "R,%ld,%ld", st().frame, is_blue);
    for (int i = 0; i < n; ++i)
        std::fprintf(f, ",%.2f,%.2f,%.1f,%d", xs[i], ys[i], rots[i], roles[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
// O 行：对手 5 台的 (x,y,rot) —— 只为了叠加时多几个参照点（提高标定精度）
inline void opponents(const double *xs, const double *ys, const double *rots, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "O,%ld", st().frame);
    for (int i = 0; i < n; ++i) std::fprintf(f, ",%.2f,%.2f,%.1f", xs[i], ys[i], rots[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
}}  // namespace simuro5::bb
#endif
'''

INS = {
    '#include "simuro5/formation.hpp"':
        '#include "simuro5/formation.hpp"\n#include "simuro5/blackbox.hpp"   // 黑匣子（临时）',
    "    formation_former(g_ctx, gameState, robots);":
        "    formation_former(g_ctx, gameState, robots);\n"
        "    bb::placement(\"former\", (long)gameState, robots, PLAYERS_PER_SIDE);",
    "    formation_later(g_ctx, gameState, formerRobots, ball, laterRobots);":
        "    bb::setball((long)gameState, ball.x, ball.y);\n"
        "    formation_later(g_ctx, gameState, formerRobots, ball, laterRobots);\n"
        "    bb::placement(\"later\", (long)gameState, laterRobots, PLAYERS_PER_SIDE);",
    "    formation_set_ball(g_ctx, gameState, pBall);":
        "    formation_set_ball(g_ctx, gameState, pBall);\n"
        "    bb::setball((long)gameState, pBall->x, pBall->y);",
    "    g_strategy.run(g_wm);":
        "    g_strategy.run(g_wm);\n"
        "    bb::frame((long)pEnv->gameState, (long)pEnv->whosBall,\n"
        "              pEnv->currentBall.pos.x, pEnv->currentBall.pos.y,\n"
        "              g_wm.ball.vx, g_wm.ball.vy, g_wm.we_have_ball ? 1 : 0,\n"
        "              g_wm.in_penalty_exec ? 1 : 0, g_wm.home[0].x, g_wm.home[1].x);\n"
        "    {   // 抓屏叠加用：我方 5 台坐标+角色、对手 5 台坐标\n"
        "        double hx[PLAYERS_PER_SIDE], hy[PLAYERS_PER_SIDE], hr[PLAYERS_PER_SIDE];\n"
        "        double ox[PLAYERS_PER_SIDE], oy[PLAYERS_PER_SIDE], orr[PLAYERS_PER_SIDE];\n"
        "        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {\n"
        "            hx[i] = g_wm.home[i].x; hy[i] = g_wm.home[i].y; hr[i] = g_wm.home[i].rot;\n"
        "            ox[i] = g_wm.opp[i].x;  oy[i] = g_wm.opp[i].y;  orr[i] = g_wm.opp[i].rot;\n"
        "        }\n"
        "        bb::robots(g_ctx.is_blue ? 1 : 0, hx, hy, hr, g_wm.role, PLAYERS_PER_SIDE);\n"
        "        bb::opponents(ox, oy, orr, PLAYERS_PER_SIDE);\n"
        "    }",
}


def read(p):
    with open(p, encoding="utf-8", newline="") as f:
        return f.read().replace("\r\n", "\n")


def write(p, s):
    with open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def applied():
    return os.path.exists(HDR) and MARK in read(SRC)


def build():
    r = subprocess.run(["cmake", "--build", "build", "--config", "Release", "--target", "Strategy4Blue"],
                       cwd=ROOT, capture_output=True, text=True, errors="replace")
    ok = "Strategy4Blue.vcxproj ->" in (r.stdout + r.stderr) or r.returncode == 0
    if not ok:
        print((r.stdout + r.stderr)[-1500:])
    return ok


def dll_src():
    return os.path.join(ROOT, "build", "bin", "Release", "Strategy4Blue.dll")


def cmd_apply(deploy):
    if applied():
        print("已经打过补丁（--revert 可还原）")
        return 0
    os.makedirs(BAK_DIR, exist_ok=True)
    os.makedirs(VAR_DIR, exist_ok=True)
    shutil.copy2(SRC, os.path.join(BAK_DIR, f"dll_blue.cpp.{time.strftime('%Y%m%d_%H%M%S')}"))
    write(HDR, HEADER)
    s = read(SRC)
    for a, b in INS.items():
        if s.count(a) != 1:
            print(f"✗ 锚点不唯一({s.count(a)}): {a[:50]}")
            return 1
        s = s.replace(a, b)
    write(SRC, s)
    print("✓ 源码已打补丁（dll_blue.cpp + 新增 blackbox.hpp）")
    if not build():
        print("✗ 构建失败")
        return 1
    var = os.path.join(VAR_DIR, "Strategy4Blue_blackbox.dll")
    shutil.copy2(dll_src(), var)
    print(f"✓ 变体 DLL: {var}")
    if deploy:
        os.makedirs(r"C:\Strategy\backup_20260912", exist_ok=True)
        bak = rf"C:\Strategy\backup_20260912\Strategy4Blue_before_blackbox_{time.strftime('%Y%m%d_%H%M')}.dll"
        shutil.copy2(DEPLOY, bak)
        shutil.copy2(var, DEPLOY)
        print(f"✓ 已部署到平台（备份 {os.path.basename(bak)}）")
        print(f"  跑完把 {CSV} + 两场 rlg 交给我")
    else:
        print(f"  部署命令：copy \"{var}\" \"{DEPLOY}\"")
    return 0


def cmd_revert():
    baks = sorted(f for f in os.listdir(BAK_DIR)) if os.path.isdir(BAK_DIR) else []
    if not baks:
        print("没有备份可还原")
        return 1
    shutil.copy2(os.path.join(BAK_DIR, baks[-1]), SRC)
    if os.path.exists(HDR):
        os.remove(HDR)
    print(f"✓ 源码已还原（用 {baks[-1]}）")
    if not build():
        print("✗ 重建失败")
        return 1
    bak = rf"C:\Strategy\backup_20260912\Strategy4Blue_after_blackbox_{time.strftime('%Y%m%d_%H%M')}.dll"
    shutil.copy2(DEPLOY, bak)
    shutil.copy2(dll_src(), DEPLOY)
    print("✓ 平台 DLL 已还原为正常版")
    return 0


def cmd_redeploy():
    """已打补丁时：按**当前源码**重建变体 DLL 并重新部署（源码可能又改了别的东西）。

    为什么需要它：黑匣子是常驻仪器，改完策略（如借墙射门）后要让它带上新代码；
      原来只能 `--revert` 再 `--apply --deploy`（两次全量构建），现在一条命令。
    """
    if not applied():
        print("还没打补丁 → 走 --apply --deploy")
        return cmd_apply(True)
    if not build():
        print("✗ 构建失败")
        return 1
    os.makedirs(VAR_DIR, exist_ok=True)
    var = os.path.join(VAR_DIR, "Strategy4Blue_blackbox.dll")
    shutil.copy2(dll_src(), var)
    os.makedirs(r"C:\Strategy\backup_20260912", exist_ok=True)
    bak = rf"C:\Strategy\backup_20260912\Strategy4Blue_before_redeploy_{time.strftime('%Y%m%d_%H%M')}.dll"
    shutil.copy2(DEPLOY, bak)
    shutil.copy2(var, DEPLOY)
    print(f"✓ 已按当前源码重建并部署（{os.path.getsize(var)}B，备份 {os.path.basename(bak)}）")
    return 0


def cmd_check():
    print("已打补丁" if applied() else "未打补丁")
    print(f"  备份: {len(os.listdir(BAK_DIR)) if os.path.isdir(BAK_DIR) else 0} 个")
    print(f"  CSV : {CSV} {'存在' if os.path.exists(CSV) else '还没有'}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--deploy", action="store_true")
    ap.add_argument("--redeploy", action="store_true", help="已打补丁时按当前源码重建并重新部署")
    a = ap.parse_args()
    if a.apply:
        return cmd_apply(a.deploy)
    if a.revert:
        return cmd_revert()
    if a.redeploy:
        return cmd_redeploy()
    return cmd_check()


if __name__ == "__main__":
    sys.exit(main())
