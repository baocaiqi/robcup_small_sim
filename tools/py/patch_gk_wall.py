# -*- coding: utf-8 -*-
"""
patch_gk_wall.py — 门将「墙边来球封近角 + 反射轨迹预测」补丁（docs/21 的 P0+P1）

用法：
    python tools/py/patch_gk_wall.py --check
    python tools/py/patch_gk_wall.py --apply r49off       # 关闭第 49 轮推球守卫（回到"能赢版"的行为）
    python tools/py/patch_gk_wall.py --apply p0p1         # 新门将逻辑
    python tools/py/patch_gk_wall.py --revert             # 回滚最近一次备份

两个补丁内容：

**r49off**（等价于回退第 49 轮的行为；用于出"38E8389E + 新门将"的包）
  · `push_allowed()` → 恒 true（原来：死球期或球在角区 → false）
  · `prep_point_ok()` → 恒 true
  （关掉这两个入口，第 49 轮所有接线点都不会触发；helper 本体与调用点保留，便于随时开回来。
   与真·38E8389E 的残留差异：角区救球块里 `deep = in_no_push_zone(...)` 的 35cm 口径仍在，
   会让 ACTIVE 在角落 35cm 内不救球——影响面小，真机对照时留意。）

**p0p1**（docs/21）
  · P0：门将改用**带边墙反射**的轨迹预测 `predict_y_at_x_reflect`（与后卫断球点同一把尺子），
        `y_at_goal` 与站位线拦截点 `iy` 都用反射版 → 墙边来球不再算错侧。
  · P1：新增"墙边来球 → 堵对应门框内侧"分支：球贴边墙(|y|<25 / >155)且朝门且距门<120cm，
        或反射落点落在门框附近外侧(|y_at_goal−90| ∈ [20,45]) → 贴门线 3cm、站**近门柱内侧 4cm**
        （按来球侧选 76 / 104），优先于近距扑球分支。
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
MOTION_HPP = "include/simuro5/motion.hpp"
MOTION_CPP = "src/motion.cpp"

EXPS = {
    # ---- 关闭第 49 轮守卫（行为等价于昨夜连胜版）----
    "r49off": [
        (ROLES,
         "    if (wm.game_state != PM_PlayOn) return false;             // 死球/摆位/重启期\n"
         "    return !in_no_push_zone(wm.ball.x, wm.ball.y);            // 球未贴角\n",
         "    (void)wm;\n"
         "    return true;   // 实验 r49off：本轮关闭推球守卫（回到能赢版行为）\n"),
        (ROLES,
         "bool prep_point_ok(double px, double py) { return !in_no_push_zone(px, py); }\n",
         "bool prep_point_ok(double px, double py) { (void)px; (void)py; return true; }   // 实验 r49off\n"),
    ],
    # ---- P0：反射预测 ----
    "p0p1": [
        (ROLES,
         "    bool heading_goal = predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal);\n",
         "    // 实验P0（docs/21）：改带边墙反射的预测——球贴墙滚/将要撞墙时，直线外推会把落点\n"
         "    //   算到界外或错侧，导致 on_target 判错、门将站错边（与后卫断球点同一把尺子）。\n"
         "    bool heading_goal = predict_y_at_x_reflect(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal);\n"),
        (ROLES,
         "    if (!predict_y_at_x(bx, by, vx, vy, out_x, iy)) {\n",
         "    if (!predict_y_at_x_reflect(bx, by, vx, vy, out_x, iy)) {   // 实验P0：同上，反射版\n"),
        # ---- P1：墙边来球 → 封近角（变量插在决策链之前）----
        (ROLES,
         "    clamp_goalie_area(ctx, out_x, iy);\n",
         "    clamp_goalie_area(ctx, out_x, iy);\n"
         "\n"
         "    // —— 实验P1（docs/21）：墙边来球 → 堵对应门框内侧 ——\n"
         "    //   触发①：球贴边墙且朝我方门且距门 <120cm；\n"
         "    //   触发②：反射预测落点擦着门框外侧（|y_at_goal-90| ∈ [20,45]）。\n"
         "    //   动作：贴门线 3cm、站近门柱内侧 4cm（按来球侧选 76 / 104），\n"
         "    //   放在近距扑球之前——墙边球常常因算不到落点而错过所有出击分支。\n"
         "    const double kWallSideDist = 25.0, kPostMargin = 14.0;   // 76/104 = 90∓14\n"
         "    bool wall_side = (by < kWallSideDist || by > 180.0 - kWallSideDist) &&\n"
         "                     danger > 0.0 && ctx.dist_our_goal(bx) < 120.0;\n"
         "    bool near_post_risk = std::fabs(y_at_goal - 90.0) >= 20.0 &&\n"
         "                          std::fabs(y_at_goal - 90.0) <= 45.0;\n"
         "    bool wall_post = wall_side || near_post_risk;\n"
         "    double post_ref = wall_side ? by : y_at_goal;            // 按来球侧选门柱\n"
         "    double post_y = (post_ref < 90.0) ? 90.0 - kPostMargin : 90.0 + kPostMargin;\n"),
        # ---- P1：插入分支 ----
        (ROLES,
         "    } else if (on_target && tta < kMaxTTA && db < kMaxReach && danger > kMinSpeed) {\n",
         "    } else if (wall_post) {                                  // 实验P1：墙边来球封近角\n"
         "        motion::position(r, ctx.our_goal_x() + ctx.attack_dir() * 3.0, post_y);\n"
         "    } else if (on_target && tta < kMaxTTA && db < kMaxReach && danger > kMinSpeed) {\n"),
    ],
    # ---- P4：贴门线时禁止"直线穿球"（治乌龙球，docs/21 §8）----
    "p4": [
        (ROLES,
         "        double px, py;\n"
         "        if (dbg < 25.0 && aligned) {\n",
         "        double px, py;\n"
         "        // —— 实验P4（docs/21 §8）：贴门线时禁止\"直线穿球\" ——\n"
         "        //   球距门 <15cm 且门将在球的外侧时，原逻辑会直奔\"球的门侧 8cm\"，\n"
         "        //   路径穿过球 → 把球顶进自家门（真机 09:08 场 4 个丢球全中此招）。\n"
         "        //   改为：横move到球侧 22cm 的场侧点，下一帧再从门侧绕过去推穿。\n"
         "        {\n"
         "            double gside = (ctx.our_goal_x() > bx) ? 1.0 : -1.0;   // 球门相对球的方位\n"
         "            bool at_line = ctx.dist_our_goal(bx) < 15.0;\n"
         "            bool outside = (r.x - bx) * gside < 0.0;               // 门将在球的外侧\n"
         "            if (at_line && outside) {\n"
         "                double side = (r.y >= by) ? 1.0 : -1.0;            // 往自己那侧绕，少掉头\n"
         "                px = bx - gside * 10.0;                            // 场侧 10cm，绝不越过球\n"
         "                py = clamp(by + side * 22.0, 74.0, 106.0);\n"
         "                clamp_goalie_area(ctx, px, py);\n"
         "                motion::position(r, px, py, motion::TM_STOP);\n"
         "                return;\n"
         "            }\n"
         "        }\n"
         "        if (dbg < 25.0 && aligned) {\n"),
    ],
    # ---- P5：墙边封球提速（提前触发 + 赶路不刹车 + 制动假想减速度 400→600）----
    "p5": [
        (MOTION_HPP,
         "constexpr double kBrakeAccel = 400.0;",
         "constexpr double kBrakeAccel = 600.0;   // 实验P5：400→600（真机实测物理 p95=658，仍在能力内）"),
        (ROLES,
         "    if (clearing) {\n"
         "        motion::position(r, clear_x, clear_y);\n"
         "    } else if (has_support) {\n",
         "    if (clearing) {\n"
         "        motion::position(r, clear_x, clear_y);\n"
         "    } else if ((ctx.dist_our_goal(bx) < 160.0) &&                    // 实验P5：墙边来球提前封\n"
         "               (by < 30.0 || by > 150.0) &&\n"
         "               (vx * (ctx.our_goal_x() - bx) > 0.0)) {\n"
         "        // 提前到\"球进我方半场+贴边墙+朝门滚\"就出发；目标 = 带墙反射的预测落点（夹在门框内）；\n"
         "        //   跑动用 TM_PASS 赶路（不刹车），最后 15cm 才 TM_STOP（防过冲打转）。\n"
         "        double ty = clamp(heading_goal ? y_at_goal : by, 74.0, 106.0);\n"
         "        double tx = ctx.our_goal_x() + ctx.attack_dir() * 3.0;\n"
         "        double dd = std::hypot(tx - r.x, ty - r.y);\n"
         "        motion::position(r, tx, ty, (dd > 15.0) ? motion::TM_PASS : motion::TM_STOP);\n"
         "    } else if (has_support) {\n"),
    ],
}
MARKERS = {"r49off": "实验 r49off", "p0p1": "实验P0", "p4": "实验P4", "p5": "实验P5"}


def read_text(p):
    with open(p, "r", encoding="utf-8", newline="") as f:
        return f.read()


def write_text(p, t):
    tmp = p + ".pgtmp"
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(t)
    os.replace(tmp, p)


def sha1(p):
    with open(p, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def applied():
    t = read_text(os.path.join(ROOT, ROLES))
    return [n for n, m in MARKERS.items() if m in t]


def latest_backup():
    if not os.path.isdir(BACKUP_ROOT):
        return None
    ds = [os.path.join(BACKUP_ROOT, d) for d in os.listdir(BACKUP_ROOT)
          if os.path.isdir(os.path.join(BACKUP_ROOT, d))]
    return max(ds, key=os.path.getmtime) if ds else None


def apply(name):
    cur = applied()
    if name in cur:
        print(f"⚠️ {name} 已经打过了")
        return 1
    edits = EXPS[name]
    files = sorted({rel for rel, _o, _n in edits})
    ts = time.strftime("%Y%m%d_%H%M%S") + "_" + name
    bdir = os.path.join(BACKUP_ROOT, ts)
    os.makedirs(bdir, exist_ok=True)
    man = []
    for rel in files:
        src = os.path.join(ROOT, rel)
        shutil.copy2(src, os.path.join(bdir, rel.replace("/", "__")))
        man.append(f"{sha1(src)}  {rel}")
    with open(os.path.join(bdir, "MANIFEST.txt"), "w", encoding="utf-8") as f:
        f.write(f"{name} @ {ts}\n" + "\n".join(man) + "\n")
    # 按文件分组替换（2026-09-12 修：原实现写死只改 roles.cpp，motion.hpp 的锚点永远匹配不到）
    by_file = {}
    for rel, old, new in edits:
        by_file.setdefault(rel, []).append((old, new))
    for rel, pairs in by_file.items():
        path = os.path.join(ROOT, rel)
        text = read_text(path)
        eol = "\r\n" if "\r\n" in text else "\n"
        for old, new in pairs:
            o, n = old.replace("\n", eol), new.replace("\n", eol)
            if text.count(o) != 1:
                print(f"❌ {rel}: 锚点匹配 {text.count(o)} 次（应为 1）→ 放弃（已改的不会写回）")
                return 2
            text = text.replace(o, n)
        write_text(path, text)
        print(f"✅ 已改 {rel}")
    print(f"✅ 已应用 {name}（备份 {bdir}）")
    return 0


def revert():
    b = latest_backup()
    if not b:
        print("没有备份")
        return 1
    for f in os.listdir(b):
        if f.endswith(".txt"):
            continue
        rel = f.replace("__", "/")
        dst = os.path.join(ROOT, rel)
        if os.path.isfile(dst):
            shutil.copy2(os.path.join(b, f), dst)
            print(f"↩️  恢复 {rel}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", metavar="NAME", choices=list(EXPS))
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    if a.check:
        print("已应用:", applied() or "无")
        print("最近备份:", latest_backup() or "无")
        return 0
    if a.revert:
        return revert()
    return apply(a.apply)


if __name__ == "__main__":
    sys.exit(main())
