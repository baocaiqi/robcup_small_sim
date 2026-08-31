# -*- coding: utf-8 -*-
"""一次性修复 roles.cpp run_goalie 死球解围分支的乌龙 bug。
roles.cpp 是 UTF-16 LE + CRLF，必须按 UTF-16 读写，切勿用普通文本编辑。
"""
import io

P = r'e:/robcup/robcup_small_sim/src/roles.cpp'

text = io.open(P, encoding='utf-16').read()
text = text.replace('\r\n', '\n').replace('\r', '\n')
lines = text.split('\n')

# 定位旧分支：锚点是唯一的 if 行
anchor = 'if (std::hypot(vx, vy) < 2.0'
start = None
for i, l in enumerate(lines):
    if anchor in l:
        start = i
        break
assert start is not None, 'anchor not found'
# 结构校验：上方 3 行注释、下方第 6 行是收尾 }
assert lines[start - 3].strip().startswith('//'), 'comment block mismatch: %r' % lines[start - 3]
assert lines[start + 6].strip() == '}', 'closing brace mismatch: %r' % lines[start + 6]

new_block = [
    '    // 死球解围：球停在我方门前 → 门将把它往场地方向顶出去（远离己门），避免门球死循环。',
    '    //   旧版固定站到球远侧(bx + attack_dir*14)，隐含假设门将在【门侧】才能穿过球推出；',
    '    //   一旦门将在【场侧】(球夹在门将与门线之间)，它只会越退越远、眼睁睁看球滚进门',
    '    //   → 乌龙（demo 战 13 球里 11 球是它）。修正：按门将相对球的位置分两种情况。',
    '    if (std::hypot(vx, vy) < 2.0 && ctx.dist_our_goal(bx) < 40.0 && db < kClearDist) {',
    '        double ad = ctx.attack_dir();',
    '        double s = (r.x - bx) * (-ad);   // >0 门将在门侧(球与门线之间)；<0 门将在场侧',
    '        double px, py;',
    '        if (s >= 0.0) {',
    '            // 门将在门侧：直接穿球往场地方向顶出去（原有正确分支）',
    '            px = bx + ad * (kPushDist + 6.0);',
    '            py = clamp(by, 78.0, 102.0);',
    '        } else {',
    '            // 门将在场侧：不能直线穿过球（会把球顶进门）。横向绕到球侧方→先到门侧贴位，',
    '            //   下一帧门将已到门侧即转入上分支，再把球顶出去。',
    '            double side = (by >= 90.0) ? -1.0 : 1.0;   // 朝中线一侧绕（上半→下，下半→上）',
    '            px = bx - ad * (kPushDist + 4.0);          // 门侧贴位',
    '            py = clamp(by + side * kLateral, 74.0, 106.0);',
    '        }',
    '        clamp_goalie_area(ctx, px, py);',
    '        motion::position(r, px, py);',
    '        return;',
    '    }',
]

# 替换 [start-3, start+7) = 旧注释 3 行 + 旧 if 块 7 行
lines[start - 3:start + 7] = new_block

out = '\r\n'.join(lines)
io.open(P, 'w', encoding='utf-16', newline='').write(out)

print('OK: replaced lines %d..%d (%d -> %d lines)' % (start - 3 + 1, start + 7, 10, len(new_block)))
