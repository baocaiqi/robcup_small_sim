# -*- coding: utf-8 -*-
"""一次性修复 roles.cpp run_goalie 的三个防守问题（UTF-16 + CRLF，必须按 UTF-16 读写）。
① 门将动态深度 80cm → 40cm（别出那么远留空门）
② 门线站位 3cm → 6cm（留出刹车/惯性余量，防冲进门）
③ 解围分支：门侧/场侧分情况，场侧横向绕到门侧（旧弧线绕行算错方向→把球顶进门）
"""
import io

P = r'e:/robcup/robcup_small_sim/src/roles.cpp'
text = io.open(P, encoding='utf-16').read()

def rep(old, new, name):
    global text
    n = text.count(old)
    assert n == 1, f'[{name}] 期望 1 处，实际 {n} 处'
    text = text.replace(old, new)
    print(f'OK [{name}]')

# ① 点球锁定：门线前 3cm → 6cm
rep('ctx.our_goal_x() + ctx.attack_dir() * 3.0, 90.0',
    'ctx.our_goal_x() + ctx.attack_dir() * 6.0, 90.0', 'penalty standoff')

# ② 门线站位深度：3cm → 6cm
rep('(std::fabs(bx - ctx.our_goal_x()) < 15.0 ? 3.0 : kGuardDist)',
    '(std::fabs(bx - ctx.our_goal_x()) < 15.0 ? 6.0 : kGuardDist)', 'guard standoff')

# ③ 近距扑球瞄准：门线前 3cm → 6cm
rep('double aim_x = ctx.our_goal_x() + ctx.attack_dir() * 3.0;',
    'double aim_x = ctx.our_goal_x() + ctx.attack_dir() * 6.0;', 'dive aim standoff')

# ④ 动态出击深度：80cm → 40cm
rep('double depth = kGuardDist + frac * (80.0 - kGuardDist);',
    'double depth = kGuardDist + frac * (40.0 - kGuardDist);', 'depth cap')

# ⑤ 解围分支：门侧/场侧分情况（修乌龙）
old_arc = '''        clear_x = bx - dirx * kPushDist;                // 推球点：球后方（门侧）
        clear_y = by - diry * kPushDist;

        if (ctx.dist_our_goal(bx) < ctx.dist_our_goal(r.x)) {   // 球夹在门将和门之间 → 弧线绕
            double nx = -diry, ny = dirx;
            double side = (r.x - bx) * nx + (r.y - by) * ny;
            double s = (side >= 0.0) ? 1.0 : -1.0;
            clear_x += s * nx * kLateral;
            clear_y += s * ny * kLateral;
        }'''
new_arc = '''        double s = (r.x - bx) * (-ctx.attack_dir());   // >0 门侧(球与门线之间)；<0 场侧
        if (s >= 0.0) {
            // 门侧：直接穿球沿清球方向顶出去
            clear_x = bx - dirx * kPushDist;                // 推球点：球后方（门侧）
            clear_y = by - diry * kPushDist;
        } else {
            // 场侧：不能直线穿球(会把球顶进门)。横向绕到门侧贴位，
            //   下一帧门将已到门侧即转入上分支再把球顶出去。
            double side = (by >= 90.0) ? -1.0 : 1.0;        // 朝中线一侧绕
            clear_x = bx - ctx.attack_dir() * (kPushDist + 4.0);   // 门侧贴位
            clear_y = clamp(by + side * (kLateral + 5.0), 74.0, 106.0);
        }'''
rep(old_arc, new_arc, 'clear branch split')

io.open(P, 'w', encoding='utf-16', newline='').write(text)
print('ALL DONE')
