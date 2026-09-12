# -*- coding: utf-8 -*-
"""
patch_gk_experiments.py — 守门员「慢半拍」最小验证实验的补丁工具（应用/回滚）

背景（真机定位，2026-09-11）：
  第 44 轮 `motion::position()` 制动包线（默认 TM_STOP）只在最后 ~7.6cm 起作用，
  但门将近距扑救(roles.cpp:394)/清球(380) 全走它 → 14cm 扑救从 ~0.2s 变 ~0.34s，
  而扑球分支的触发线是 kMaxTTA=15 帧(0.375s) → 末段顶到生死线（"慢半拍"）。
  第 49 轮 `run_goalie` 新增推球守卫(roles.cpp:162)：死球期 **或球在四角 35cm 黄区内**
  一律不碰球 → 球在角上时门将看着不动（实测球在角区时门将静止占比 47% → 59~100%）。

三个实验（一次只上一个，用户手动各跑一场，用 goalie_diag.py 出对照）：
  A  角区守卫对门将放开：只保留"死球期不许推"（1 行）
  B1 门将扑救(394)+清球(380) 改 TM_PASS（经过型，不套包线）（2 行）
  B2 门将单独用更高制动加速度 kGkBrakeAccel=900（motion 加可选参数 + 两处调用）
  C  第 47 轮远射档回退开关 kFarShotEnabled=false（1 行，测"锅是不是在进攻端"）

用法：
    python tools/py/patch_gk_experiments.py --check              # 看当前状态（是否已打某个实验）
    python tools/py/patch_gk_experiments.py --apply A            # 应用（自动备份）
    python tools/py/patch_gk_experiments.py --revert             # 回滚到最近一次备份
    python tools/py/patch_gk_experiments.py --list               # 列出实验内容

约定：应用前把涉及文件完整备份到 build/gk_patch_backup/<时间戳>/（含 SHA1 清单）；
      回滚用最近一次备份原子恢复。补丁只在工作树里，**不会**碰 C:\Strategy 的 DLL。
"""
import argparse
import hashlib
import os
import shutil
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BACKUP_ROOT = os.path.join(ROOT, "build", "gk_patch_backup")

ROLES = "src/roles.cpp"
MOTION_CPP = "src/motion.cpp"
MOTION_HPP = "include/simuro5/motion.hpp"
SHOOT_CPP = "src/shoot.cpp"

EXPS = {
    "A": [
        (ROLES,
         "    if (!push_allowed(wm)) {\n",
         "    if (wm.game_state != PM_PlayOn) {   // 实验A：角区对门将放开（只留死球期）\n"),
    ],
    "B1": [
        (ROLES,
         "        motion::position(r, clear_x, clear_y);\n",
         "        motion::position(r, clear_x, clear_y, motion::TM_PASS);   // 实验B1：清球不套包线\n"),
        (ROLES,
         "        motion::position(r, aim_x, aim_y);\n",
         "        motion::position(r, aim_x, aim_y, motion::TM_PASS);   // 实验B1：扑救不套包线\n"),
    ],
    "B2": [
        (MOTION_HPP,
         "void position(RobotState &r, double tx, double ty, TargetMode mode = TM_STOP);\n",
         "void position(RobotState &r, double tx, double ty, TargetMode mode = TM_STOP,\n"
         "              double brake_accel = kBrakeAccel);   // 实验B2：可按调用点覆盖制动加速度\n"),
        (MOTION_CPP,
         "void position(RobotState &r, double tx, double ty, TargetMode mode) {\n",
         "void position(RobotState &r, double tx, double ty, TargetMode mode, double brake_accel) {\n"),
        (MOTION_CPP,
         "        double v_allow = std::sqrt(2.0 * kBrakeAccel * std::max(0.0, de - kStopEps));\n",
         "        double v_allow = std::sqrt(2.0 * brake_accel * std::max(0.0, de - kStopEps));   // 实验B2\n"),
        (ROLES,
         "    const double kLateral   = 15.0;\n",
         "    const double kLateral   = 15.0;\n"
         "    const double kGkBrakeAccel = 900.0;   // 实验B2：门将专用制动加速度（默认 400）\n"),
        (ROLES,
         "        motion::position(r, clear_x, clear_y);\n",
         "        motion::position(r, clear_x, clear_y, motion::TM_STOP, kGkBrakeAccel);   // 实验B2\n"),
        (ROLES,
         "        motion::position(r, aim_x, aim_y);\n",
         "        motion::position(r, aim_x, aim_y, motion::TM_STOP, kGkBrakeAccel);   // 实验B2\n"),
    ],
    "C": [
        (SHOOT_CPP,
         "constexpr bool   kFarShotEnabled = true;\n",
         "constexpr bool   kFarShotEnabled = false;   // 实验C：回退第47轮远射档\n"),
    ],
}

MARKERS = {"A": "实验A", "B1": "实验B1", "B2": "实验B2", "C": "实验C"}


def read_text(path):
    with open(path, "r", encoding="utf-8", newline="") as fp:
        return fp.read()


def write_text(path, text):
    tmp = path + ".pgtmp"
    with open(tmp, "w", encoding="utf-8", newline="") as fp:
        fp.write(text)
    os.replace(tmp, path)


def sha1(path):
    with open(path, "rb") as fp:
        return hashlib.sha1(fp.read()).hexdigest()


def applied_exps():
    out = []
    for name, marker in MARKERS.items():
        for rel, _old, _new in EXPS[name]:
            p = os.path.join(ROOT, rel)
            if marker in read_text(p):
                out.append(name)
                break
    return out


def latest_backup():
    if not os.path.isdir(BACKUP_ROOT):
        return None
    dirs = [os.path.join(BACKUP_ROOT, d) for d in os.listdir(BACKUP_ROOT)]
    dirs = [d for d in dirs if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


def cmd_check():
    cur = applied_exps()
    print("当前已打实验:", cur or "无（工作树是原版）")
    b = latest_backup()
    print("最近备份:", b or "无")
    if b:
        man = os.path.join(b, "MANIFEST.txt")
        if os.path.isfile(man):
            print(read_text(man).strip())
    return 0


def cmd_list():
    for name, edits in EXPS.items():
        print(f"== 实验 {name} ==")
        for rel, old, new in edits:
            print(f"   {rel}: {old.strip()[:70]}")
            print(f"      → {new.strip()[:70]}")
    return 0


def cmd_apply(name):
    if name not in EXPS:
        print(f"未知实验 {name}，可选 {list(EXPS)}")
        return 2
    cur = applied_exps()
    if cur:
        print(f"⚠️ 工作树已打过实验 {cur}，先 --revert 再应用")
        return 3
    edits = EXPS[name]
    files = sorted({rel for rel, _o, _n in edits})
    ts = time.strftime("%Y%m%d_%H%M%S") + "_" + name
    bdir = os.path.join(BACKUP_ROOT, ts)
    os.makedirs(bdir, exist_ok=True)
    lines = []
    for rel in files:
        src = os.path.join(ROOT, rel)
        dst = os.path.join(bdir, rel.replace("/", "__"))
        shutil.copy2(src, dst)
        lines.append(f"{sha1(src)}  {rel}")
    with open(os.path.join(bdir, "MANIFEST.txt"), "w", encoding="utf-8") as fp:
        fp.write(f"实验 {name} @ {ts}\n" + "\n".join(lines) + "\n")

    # 逐文件做替换（注意 EOL：文件可能是 LF 或 CRLF）
    by_file = {}
    for rel, old, new in edits:
        by_file.setdefault(rel, []).append((old, new))
    for rel, pairs in by_file.items():
        path = os.path.join(ROOT, rel)
        text = read_text(path)
        eol = "\r\n" if "\r\n" in text else "\n"
        for old, new in pairs:
            o = old.replace("\n", eol)
            n = new.replace("\n", eol)
            cnt = text.count(o)
            if cnt != 1:
                print(f"❌ {rel}: 锚点匹配 {cnt} 次（应为 1）→ 放弃，未改任何文件")
                return 4
            text = text.replace(o, n)
        write_text(path, text)
        print(f"✅ 已改 {rel}")
    print(f"备份目录：{bdir}")
    return 0


def cmd_revert():
    b = latest_backup()
    if not b:
        print("没有备份可回滚")
        return 2
    n = 0
    for f in os.listdir(b):
        if f.endswith(".txt"):
            continue
        rel = f.replace("__", "/")
        dst = os.path.join(ROOT, rel)
        if os.path.isfile(dst):
            shutil.copy2(os.path.join(b, f), dst)
            print(f"↩️  恢复 {rel}")
            n += 1
    print(f"从 {b} 回滚了 {n} 个文件")
    return 0


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--list", action="store_true")
    g.add_argument("--revert", action="store_true")
    g.add_argument("--apply", metavar="EXP", help="A | B1 | B2 | C")
    args = ap.parse_args()
    if args.check:
        return cmd_check()
    if args.list:
        return cmd_list()
    if args.revert:
        return cmd_revert()
    return cmd_apply(args.apply)


if __name__ == "__main__":
    sys.exit(main())
