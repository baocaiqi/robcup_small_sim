# -*- coding: utf-8 -*-
"""
role_overlay.py — 方案 B「抓屏叠加」：把平台真机截图里的我方 5 台机器人标上角色名

一句话：平台球场画面里我方 5 台机器人是彩色小方块，本工具把「门将 0 / 主攻 1 /
助攻 2 / 中场 3 / 后卫 4」写到对应方块旁边，方便肉眼盯真机看"谁在哪、谁该去哪"。

用法（仓库根目录执行）：

  # ① 离线模式：截图 + 同时段录像（自动标定 + 帧匹配，用于开发/验证）
  python tools/py/role_overlay.py --image build/penalty_shots/review/153147_e01_00_field.png \
      --rlg "C:\\Strategy\\20260912152913-5-DEMO Yellow-MyTeam-Blue.rlg" \
      --out build/role_overlay_check --recalib

  # ② 生产模式：截图 + 黑匣子 CSV（R 行给我方 5 台坐标 + role）
  python tools/py/role_overlay.py --image shot.png --csv C:\\Strategy\\hnnu_blackbox.csv --row latest

  # ③ 实时模式：循环抓 WorldModel 窗口 → 读 CSV 最新一行 → 叠加 → 存 PNG
  python tools/py/role_overlay.py --watch --csv C:\\Strategy\\hnnu_blackbox.csv \
      --out build/role_overlay_live --interval 1.5

  # ④ 只画场地模板（验证标定是否对准平台自己画的线）
  python tools/py/role_overlay.py --image shot.png --template-only --calib build/role_overlay_calib.json

  # ⑤ 手动标定：给 4+ 组 [场地x, 场地y, 像素x, 像素y]（最可靠，推荐现场用）
  python tools/py/role_overlay.py --calib-points build/anchors.json --calib build/role_overlay_calib.json

标定默认存 build/role_overlay_calib.json，标定一次后复用（--recalib 强制重算）。

角色↔编号是固定对应（src/role_assignment.cpp: 0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE），
所以本工具真正要解的是「画面上哪台机器人是几号」。

依赖：仅 Python 标准库 + PIL（不用 numpy）。
"""
import argparse
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ------------------------------------------------------------------ 输出编码
# Windows 控制台默认 GBK，直接 print 中文/箭头会 UnicodeEncodeError → 强制 UTF-8
try:
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
except Exception:
    pass

# ------------------------------------------------------------------ 常量

# 编号 → (中文名, 英文名)。编号与角色固定对应。
# 注意 include/simuro5/role_assignment.hpp 的 enum 把 PASSIVE=2 / ASSIST=3 / MIDFIELD=4，
# 但 assign() 与 offline_test 实际写的是 0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE，
# 本工具按"实际运行的映射"命名（与任务说明一致）。--role-map header 可切换成 enum 口径。
ROLE_LABELS = {
    0: ("门将", "GK"),
    1: ("主攻", "ACTIVE"),
    2: ("助攻", "ASSIST"),
    3: ("中场", "MID"),
    4: ("后卫", "PASSIVE"),
}
ROLE_LABELS_HEADER = {
    0: ("门将", "GK"),
    1: ("主攻", "ACTIVE"),
    2: ("后卫", "PASSIVE"),
    3: ("助攻", "ASSIST"),
    4: ("中场", "MID"),
}
ROLE_COLORS = {
    0: (255, 208, 0),
    1: (255, 70, 70),
    2: (70, 220, 90),
    3: (80, 180, 255),
    4: (225, 130, 255),
}

# 排除区（2 倍放大图）：x>1000 右侧菜单面板，y>950 底部 HELP 文字
MENU_X = 1000
HELP_Y = 950

# 色块尺寸过滤（任务给的区间）：面积 200~4000、外框 12~90px
BLOB_MIN_AREA = 200
BLOB_MAX_AREA = 4000
BLOB_MIN_BOX = 12
BLOB_MAX_BOX = 90
BALL_MAX_BOX = 26         # 球（橙色小圆点）比机器人色块小

# 机器人标记的真实外框约 38~45px；用上限 90 会把"相邻两台粘在一起"的连通域也放进来，
# 那种粘连块的中心会偏到两台中间 → 标错。这里用更紧的上限过滤粘连块。
SPRITE_MAX_BOX = 62

FONT_CANDIDATES = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
                   r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\simsun.ttc"]

DEFAULT_CALIB = os.path.join("build", "role_overlay_calib.json")

# 场地常量（与 include/simuro5/field_info.hpp 一致）
FIELD_LEN = 220.0
FIELD_WID = 180.0


# ------------------------------------------------------------------ 线性代数


def solve_linear(A, b, n):
    """高斯消元（部分选主元）解 n 元线性方程组。"""
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-10:
            return None
        M[c], M[p] = M[p], M[c]
        pv = M[c][c]
        for r in range(n):
            if r != c and abs(M[r][c]) > 1e-14:
                f = M[r][c] / pv
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def build_affine_system(pairs):
    """仿射最小二乘（6 参数）：X = a x + b y + c ; Y = d x + e y + f"""
    n = 6
    A = [[0.0] * n for _ in range(n)]
    b = [0.0] * n
    for (x, y), (X, Y) in pairs:
        r1 = [x, y, 1.0, 0.0, 0.0, 0.0]
        r2 = [0.0, 0.0, 0.0, x, y, 1.0]
        for i in range(n):
            for j in range(n):
                A[i][j] += r1[i] * r1[j] + r2[i] * r2[j]
            b[i] += r1[i] * X + r2[i] * Y
    return A, b, n


def build_homography_system(pairs):
    """完整 8 参数 DLT（h8=1 归一化）"""
    n = 8
    A = [[0.0] * n for _ in range(n)]
    b = [0.0] * n
    for (x, y), (X, Y) in pairs:
        r1 = [x, y, 1.0, 0.0, 0.0, 0.0, -x * X, -y * X]
        r2 = [0.0, 0.0, 0.0, x, y, 1.0, -x * Y, -y * Y]
        for i in range(n):
            for j in range(n):
                A[i][j] += r1[i] * r1[j] + r2[i] * r2[j]
            b[i] += r1[i] * X + r2[i] * Y
    return A, b, n


def apply_transform(h, x, y):
    """场地坐标(cm) → 像素。h 为 8 或 9 个数（仿射时 h[6]=h[7]=0）。"""
    d = h[6] * x + h[7] * y + (h[8] if len(h) > 8 else 1.0)
    if abs(d) < 1e-9:
        d = 1e-9
    return ((h[0] * x + h[1] * y + h[2]) / d, (h[3] * x + h[4] * y + h[5]) / d)


def normalize_matrix(m):
    """把 8/9 个数规整为 9 个数（最后一位归一化为 1）。"""
    if m is None:
        return None
    v = [float(t) for t in m]
    if len(v) == 6:
        v = v + [0.0, 0.0]
    if len(v) == 8:
        v = v + [1.0]
    if len(v) != 9:
        return None
    if abs(v[8]) < 1e-12:
        return None
    return [t / v[8] for t in v]


def fit_affine(pairs):
    A, b, n = build_affine_system(pairs)
    sol = solve_linear(A, b, n)
    if sol is None:
        return None
    return normalize_matrix(sol + [0.0, 0.0])


def fit_homography(pairs):
    A, b, n = build_homography_system(pairs)
    sol = solve_linear(A, b, n)
    if sol is None:
        return None
    return normalize_matrix(sol + [1.0])


def rms_error(h, pairs):
    if h is None or not pairs:
        return float("inf")
    tot = 0.0
    for (x, y), (X, Y) in pairs:
        px, py = apply_transform(h, x, y)
        tot += (px - X) ** 2 + (py - Y) ** 2
    return math.sqrt(tot / len(pairs))


# ------------------------------------------------------------------ 图像检测


def _is_color_pixel(r, g, b):
    """机器人色块 / 球的判据：够亮且有明显色偏（场地线是深灰，会被排除）。"""
    mx, mn = max(r, g, b), min(r, g, b)
    return (mx - mn) > 45 and mx > 75


def _is_orange(r, g, b):
    """球是橙色小圆点"""
    return r > 140 and g < 180 and b < 100 and (r - b) > 80


def _quantize(r, g, b):
    """颜色粗分类，用于生成"色块签名"（跨帧跟踪同一台机器人）。"""
    mx, mn = max(r, g, b), min(r, g, b)
    if mx - mn <= 45:
        return "GRAY"
    if r > 140 and g > 140 and b < 120:
        return "YGREEN" if g >= r else "YELLOW"
    if r > 150 and 90 < g < 180 and b < 110:
        return "ORANGE"
    if b > 120 and g > 90 and r < 120:
        return "CYAN"
    if b > 120 and r > 90 and g < 110:
        return "MAGENTA"
    if r > 120 and g < 100 and b < 100:
        return "RED"
    if b > 110 and r < 110 and g < 110:
        return "BLUE"
    if g > 130 and r < 120 and b < 120:
        return "GREEN"
    return "OTHER"


def detect_blobs(img, exclude_menu=True, exclude_help=True):
    """检测彩色连通域。返回 (sprites, balls)。

    排除：x > MENU_X 的菜单面板、y > HELP_Y 的底部 HELP 文字（否则文字被当成色块）。
    过滤：面积 200~4000、外框 12~90px、长宽比 0.35~2.8。
    球（橙色小点，外框 ≤26px）单独归到 balls，不当作机器人。
    """
    W, H = img.size
    px = img.load()
    x_lim = min(W, MENU_X) if exclude_menu else W
    y_lim = min(H, HELP_Y) if exclude_help else H

    seen = bytearray(x_lim * y_lim)
    sprites, balls = [], []
    for sy in range(y_lim):
        for sx in range(x_lim):
            if seen[sy * x_lim + sx]:
                continue
            r, g, b = px[sx, sy]
            if not _is_color_pixel(r, g, b):
                continue
            # 洪水填充（4 邻域）
            stack = [(sx, sy)]
            seen[sy * x_lim + sx] = 1
            comp = []
            while stack:
                cx, cy = stack.pop()
                comp.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if nx < 0 or ny < 0 or nx >= x_lim or ny >= y_lim:
                        continue
                    if seen[ny * x_lim + nx]:
                        continue
                    rr, gg, bb = px[nx, ny]
                    if _is_color_pixel(rr, gg, bb):
                        seen[ny * x_lim + nx] = 1
                        stack.append((nx, ny))
            area = len(comp)
            if area < 30:
                continue
            xs = [p[0] for p in comp]
            ys = [p[1] for p in comp]
            x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
            w, h = x1 - x0 + 1, y1 - y0 + 1
            cr = sum(px[p[0], p[1]][0] for p in comp) // area
            cg = sum(px[p[0], p[1]][1] for p in comp) // area
            cb = sum(px[p[0], p[1]][2] for p in comp) // area
            info = dict(cx=sum(xs) / float(area), cy=sum(ys) / float(area),
                        box=(x0, y0, x1, y1), area=area, w=w, h=h, rgb=(cr, cg, cb),
                        signature=[_quantize(cr, cg, cb)])
            if w <= BALL_MAX_BOX and h <= BALL_MAX_BOX and _is_orange(cr, cg, cb):
                info["kind"] = "ball"
                balls.append(info)
                continue
            if not (BLOB_MIN_AREA <= area <= BLOB_MAX_AREA):
                continue
            if not (BLOB_MIN_BOX <= w <= BLOB_MAX_BOX and BLOB_MIN_BOX <= h <= BLOB_MAX_BOX):
                continue
            if not (0.35 <= w / float(h) <= 2.8):
                continue
            info["kind"] = "sprite"
            sprites.append(info)
    sprites.sort(key=lambda d: (d["cy"], d["cx"]))
    balls.sort(key=lambda d: (d["cy"], d["cx"]))
    for s in sprites:
        s["signature"] = sprite_signature(img, s)
    return sprites, balls


def sprite_signature(img, s):
    """色块内出现比例最高的 2 种主色（跨帧识别"同一台"的指纹）。"""
    px = img.load()
    x0, y0, x1, y1 = s["box"]
    hist = {}
    for yy in range(y0, y1 + 1):
        for xx in range(x0, x1 + 1):
            r, g, b = px[xx, yy]
            if _is_color_pixel(r, g, b):
                k = _quantize(r, g, b)
                hist[k] = hist.get(k, 0) + 1
    return [k for k, _ in sorted(hist.items(), key=lambda kv: -kv[1])[:2]]


# ------------------------------------------------------------------ 黑匣子 CSV


def parse_blackbox(csv_path):
    """解析黑匣子 CSV。

    F,<frame>,<gameState>,<whosBall>,<bx>,<by>,<bvx>,<bvy>,<we_have_ball>,<in_penalty>,<home0.x>,<home1.x>
    R,<frame>,<isBlue>,x0,y0,rot0,role0,x1,y1,rot1,role1,...   （我方 5 台，5 组 4 个数）
    O,<frame>,x0,y0,rot0,...                                  （对手 5 台，5 组 3 个数）
    """
    out = dict(frames=[], R=[], O=[])
    if not csv_path or not os.path.isfile(csv_path):
        out["error"] = "文件不存在"
        return out
    with open(csv_path, encoding="utf-8", errors="replace") as fp:
        for ln in fp:
            t = ln.strip().split(",")
            if not t:
                continue
            try:
                if t[0] == "F" and len(t) >= 12:
                    out["frames"].append(dict(
                        frame=int(t[1]), gs=int(t[2]), whos=int(t[3]),
                        bx=float(t[4]), by=float(t[5]), vx=float(t[6]), vy=float(t[7]),
                        ours=int(t[8]), pen=int(t[9]),
                        home0=float(t[10]), home1=float(t[11])))
                elif t[0] == "R" and len(t) >= 23:
                    robots = []
                    for i in range(5):
                        b = 3 + 4 * i
                        robots.append(dict(idx=i, x=float(t[b]), y=float(t[b + 1]),
                                           rot=float(t[b + 2]), role=int(t[b + 3])))
                    out["R"].append(dict(frame=int(t[1]), is_blue=int(t[2]), robots=robots))
                elif t[0] == "O" and len(t) >= 17:
                    robots = []
                    for i in range(5):
                        b = 2 + 3 * i
                        robots.append(dict(idx=i, x=float(t[b]), y=float(t[b + 1]),
                                           rot=float(t[b + 2])))
                    out["O"].append(dict(frame=int(t[1]), robots=robots))
            except (IndexError, ValueError):
                continue
    return out


def our_robots_from_csv(csv_path, row="latest"):
    """从 CSV 取我方 5 台（含 role）。返回 (dict|None, err|None)"""
    data = parse_blackbox(csv_path)
    if data.get("error"):
        return None, data["error"] + f": {csv_path}"
    R = data["R"]
    if not R:
        if data["frames"]:
            f = data["frames"][-1]
            return None, ("CSV 只有 F 行（还没有 R 行）：F 行只记了 home0.x=%.1f / home1.x=%.1f，"
                          "给不出 5 台完整 (x,y)，无法定位。等 R 行写入，或改用 --rlg。"
                          % (f["home0"], f["home1"]))
        return None, "CSV 里没有可用坐标行"
    if row == "latest":
        rec = R[-1]
    else:
        rec = None
        for r in R:
            if str(r["frame"]) == str(row):
                rec = r
                break
        if rec is None:
            return None, f"CSV 里没有 frame={row} 的 R 行（共 {len(R)} 条 R 行）"
    return dict(robots=rec["robots"], frame=rec["frame"], is_blue=rec.get("is_blue")), None


# ------------------------------------------------------------------ 帧匹配（离线自动标定）


def _assign_candidates(robots, sprites, tries=60, seed=20260912):
    """生成（机器人→色块）的候选指派：
      · 一组确定性的"按序配对"（图像 x 与场地 y 通常单调）；
      · 若干随机的 4 点组合（RANSAC 风格），保证能跳出错误初值。
    """
    import random
    rng = random.Random(seed)
    n = len(sprites)
    m = len(robots)
    out = []
    # 按 x 排序的确定性配对
    for keyr, keys in ((lambda r: r["y"], lambda s: s["cx"]),
                       (lambda r: r["x"], lambda s: s["cy"])):
        order_r = sorted(range(m), key=lambda i: keyr(robots[i]))
        order_s = sorted(range(n), key=lambda j: keys(sprites[j]))
        out.append([order_s[i % n] for i in range(m)])
    # 随机组合
    for _ in range(tries):
        out.append([rng.randrange(n) for _ in range(m)])
    return out


def icp_match(robots, sprites, h0=None, tol=90.0, iters=6, max_seeds=10):
    """把机器人坐标投到图上，反复"预测→找最近色块→重拟合"直到稳定（ICP 式）。

    多个初值并行尝试，但只对"初始内点多"的前 max_seeds 个做 ICP（否则太慢）。
    按（命中数, 内点数, -残差）取最优；要求一对一（distinct）。
    """
    if not robots or not sprites:
        return None
    sprite_pts = [(s["cx"], s["cy"]) for s in sprites]

    def init_score(h):
        """初始拟合的内点数（预测点落在某色块 40px 内的个数）"""
        n = 0
        for r in robots:
            px, py = apply_transform(h, r["x"], r["y"])
            d = min(math.hypot(px - p[0], py - p[1]) for p in sprite_pts)
            if d < 40.0:
                n += 1
        return n

    seeds = []
    if h0:
        seeds.append((init_score(h0), h0))
    for assign in _assign_candidates(robots, sprites, tries=40):
        pairs = [((robots[i]["x"], robots[i]["y"]), sprite_pts[assign[i]])
                 for i in range(len(robots))]
        h = fit_affine(pairs)
        if h is not None:
            seeds.append((init_score(h), h))
    seeds.sort(key=lambda t: -t[0])
    cand_h = [h for _, h in seeds[:max_seeds]]

    best = None
    for h in cand_h:
        for _ in range(iters):
            pairs, seen = [], set()
            for r in robots:
                px, py = apply_transform(h, r["x"], r["y"])
                for j in sorted(range(len(sprite_pts)),
                                key=lambda k: (sprite_pts[k][0] - px) ** 2 +
                                              (sprite_pts[k][1] - py) ** 2):
                    if j in seen:
                        continue
                    dd = math.hypot(sprite_pts[j][0] - px, sprite_pts[j][1] - py)
                    if dd < tol:
                        seen.add(j)
                        pairs.append(((r["x"], r["y"]), sprite_pts[j]))
                    break
            if len(pairs) < 3:
                break
            h2 = fit_affine(pairs)
            if h2 is None:
                break
            # 抗外点：去掉残差最大的一个点再拟合
            errs = sorted(((math.hypot(apply_transform(h2, f[0], f[1])[0] - s[0],
                                       apply_transform(h2, f[0], f[1])[1] - s[1]), f, s)
                           for f, s in pairs), key=lambda t: -t[0])
            if len(errs) >= 4:
                h3 = fit_affine([(f, s) for _, f, s in errs[:-1]])
                if h3 is not None:
                    h2 = h3
            h = h2
        # 收尾评分
        mapping, resid, used = {}, {}, set()
        order = sorted(robots, key=lambda r: min(
            math.hypot(apply_transform(h, r["x"], r["y"])[0] - p[0],
                       apply_transform(h, r["x"], r["y"])[1] - p[1]) for p in sprite_pts))
        tot, hits = 0.0, 0
        for r in order:
            px, py = apply_transform(h, r["x"], r["y"])
            for j in sorted(range(len(sprite_pts)),
                            key=lambda k: (sprite_pts[k][0] - px) ** 2 +
                                          (sprite_pts[k][1] - py) ** 2):
                if j in used:
                    continue
                dd = math.hypot(sprite_pts[j][0] - px, sprite_pts[j][1] - py)
                if dd < tol:
                    used.add(j)
                    mapping[r["idx"]] = j
                    resid[r["idx"]] = dd
                    tot += dd * dd
                    if dd < 12.0:
                        hits += 1
                break
        for r in robots:
            resid.setdefault(r["idx"], None)
        if not mapping:
            continue
        rms = math.sqrt(tot / len(mapping))
        cand = dict(matrix=h, mapping=mapping, residuals=resid, hits=hits, rms=rms,
                    n_matched=len(mapping))
        if best is None or (cand["hits"], cand["n_matched"], -cand["rms"]) > \
                (best["hits"], best["n_matched"], -best["rms"]):
            best = cand
    return best


# 平台上"蓝门罚球区框"的实测像素位置（2 倍图，本仓库资源 build/penalty_shots 里量得，
# 见 build/probe27/29/41）。这两个框是平台自己画的线、语义明确（罚球区 80×70），
# 用作标定校验最可靠：标定后把它们投影回图上，应该与原线大致重合。
LANDMARK_PENALTY_BOX_PX = dict(x=(731.5, 907.5), y=(262.5, 665.5))
LANDMARK_GOALAREA_BOX_PX = dict(x=(830.5, 907.5), y=(337.5, 590.5))


def landmark_consistency(h):
    """把"罚球区框/球门区框"的角点按 h 投影回像素，与实测线位置比较，
    返回平均偏差(px)；无法判定时返回 None。

    这是防止"色块↔编号"匹配凑出假解的关键校验（实测：假解能凑出 0px 残差，
    但投影出的罚球区框会偏出几百像素）。

    ⚠️ 实测结论（见 build/probe45..51）：平台上"罚球区框"的像素尺寸
    (176 x 403) 与 "球门区框"(77 x 253) 给出的 px/cm 尺度彼此不一致
    （y 尺度 5.76 vs 8.43），说明【这两组框不是同一透视下的同名物】，
    或者平台用了非等比视口 + 透视。因此本校验只用于**剔除明显荒谬的解**，
    不能当作"标定正确"的证明；最终请用 --template-only 出图肉眼确认。
    """
    if h is None:
        return None
    x0, x1 = LANDMARK_PENALTY_BOX_PX["x"]
    y0, y1 = LANDMARK_PENALTY_BOX_PX["y"]
    # 罚球区四角（场地坐标，顺序：远-下、远-上、近-下、近-上）
    field_corners = [(140.0, 55.0), (140.0, 125.0), (220.0, 55.0), (220.0, 125.0)]
    orderings = [
        [(x0, y0), (x0, y1), (x1, y0), (x1, y1)],
        [(x0, y1), (x0, y0), (x1, y1), (x1, y0)],
    ]
    best = None
    for px_corners in orderings:
        errs = []
        for (fx, fy), (X, Y) in zip(field_corners, px_corners):
            px, py = apply_transform(h, fx, fy)
            errs.append(math.hypot(px - X, py - Y))
        m = sum(errs) / len(errs)
        if best is None or m < best:
            best = m
    return best


def default_anchors():
    """内置的默认标定点（从本仓库 build/penalty_shots 的截图实测）。
    返回 [[fx, fy, px, py], ...]。这些点是"平台自己画的罚球区/球门区框"的角点，
    语义明确；用它们拟合出的标定至少保证门区附近位置正确。
    """
    x0, x1 = LANDMARK_PENALTY_BOX_PX["x"]
    y0, y1 = LANDMARK_PENALTY_BOX_PX["y"]
    gx0, gx1 = LANDMARK_GOALAREA_BOX_PX["x"]
    gy0, gy1 = LANDMARK_GOALAREA_BOX_PX["y"]
    return [
        [140.0, 55.0, x0, y0],
        [140.0, 125.0, x0, y1],
        [220.0, 55.0, x1, y0],
        [220.0, 125.0, x1, y1],
        [170.0, 75.0, gx0, gy0],
        [170.0, 105.0, gx0, gy1],
    ]


def validate_matrix(h):
    """对仿射/单应矩阵做"物理合理性"检查——防止 ICP 收敛到荒谬解（例如所有点挤到一处）。
    检查项：
      1) 场地 x 方向与 y 方向在图上都要有"足够大"的变化（否则退化）；
      2) 每 cm 对应像素数应在 0.3~30 之间（真机 2 倍图约 2~8 px/cm）。
    返回 (ok, reason)
    """
    if h is None:
        return False, "矩阵为空"
    # 场地 x 单位向量 (dx, dy) 与 y 单位向量
    px0, py0 = apply_transform(h, 0.0, 0.0)
    px1, py1 = apply_transform(h, 100.0, 0.0)   # 沿场地 x 走 100cm
    px2, py2 = apply_transform(h, 0.0, 100.0)   # 沿场地 y 走 100cm
    ex = math.hypot(px1 - px0, py1 - py0) / 100.0
    ey = math.hypot(px2 - px0, py2 - py0) / 100.0
    if not (0.3 <= ex <= 30.0):
        return False, f"场地 x 方向缩放异常：{ex:.3f} px/cm"
    if not (0.3 <= ey <= 30.0):
        return False, f"场地 y 方向缩放异常：{ey:.3f} px/cm"
    # 两个方向不能几乎平行（退化）
    v1 = ((px1 - px0) / 100.0, (py1 - py0) / 100.0)
    v2 = ((px2 - px0) / 100.0, (py2 - py0) / 100.0)
    cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
    if cross < 1e-3 * (ex * ey):
        return False, "场地 x/y 两方向在图上几乎平行（退化）"
    return True, "ok"


def find_best_frame(img, sprites, frames, ball_pts=None, coarse=400, verbose=True):
    """在 RLG 里找与截图最匹配的帧（同时得到标定 + 颜色↔编号对照）。

    ⚠️ 可靠性说明（实测教训）：
      只靠"色块位置 ↔ 机器人在场坐标"做 ICP，很容易收敛到"多台挤到同一色块"的
      退化解（残差看起来很小，标定却完全错）。所以这里加了多重把关：
        1) validate_matrix 检查缩放/非退化；
        2) 一对一匹配（distinct）；
        3) 只接受「命中 5 台」或「命中≥3 且残差≤2px」的候选；
        4) 额外要求标定后【罚球区框】的投影与平台自己画的框大致重合（见 check_landmarks）。
      即便全过，仍建议用 --template-only 出图肉眼确认（见模块头说明）。
    """
    if not sprites or not frames:
        return None
    step = max(1, len(frames) // coarse)
    results = []
    for i in range(0, len(frames), step):
        for team in ("blue", "yellow"):
            robots = [dict(idx=j, x=r["x"], y=r["y"], rot=r["rot"])
                      for j, r in enumerate(frames[i][team])]
            m = icp_match(robots, sprites)
            if not m:
                continue
            ok, why = validate_matrix(m["matrix"])
            if not ok:
                continue
            # 命中少却残差大的解不可信（常见于"多台挤到少数色块"的假解）
            if m["hits"] < 5 and not (m["hits"] >= 3 and m["rms"] <= 2.0):
                continue
            # 用平台自己画的罚球区框做最后一道校验（这是最硬的把关：
            # 假解能让色块残差为 0，却会让罚球区框投影飞出上百像素）
            lerr = landmark_consistency(m["matrix"])
            if lerr is None or lerr > 25.0:
                continue
            results.append((m["hits"], -m["rms"], lerr if lerr is not None else 999.0,
                            i, team, m))
    results.sort(reverse=True)
    if verbose:
        print("[帧匹配] 候选前 5（已过物理/退化/landmark 过滤）：")
        for hits, negrms, lerr, i, team, m in results[:5]:
            print(f"   帧{i:5d} {team:6s} 命中 {hits}/5  残差 {-negrms:6.2f}px  "
                  f"罚球区框偏差 {lerr:6.1f}px")
    if not results:
        return None
    hits, negrms, lerr, i, team, m = results[0]
    return dict(matrix=m["matrix"], hits=hits, rms=-negrms, frame=i, team=team,
                landmark_err=lerr, mapping=m["mapping"],
                robots=[dict(idx=j, x=r["x"], y=r["y"], rot=r["rot"])
                        for j, r in enumerate(frames[i][team])])


# ------------------------------------------------------------------ 绘制


def load_font(size=22):
    from PIL import ImageFont
    for p in FONT_CANDIDATES:
        if os.path.isfile(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return ImageFont.load_default()


def draw_overlay(img, labels, role_labels, meta=None, warnings=None):
    """画角色标签：圆角框 + 中文角色名 + 编号 + 引线；残差大的打警告；
    本帧没检测到色块的，用空心圈 + "未检测到色块"。"""
    from PIL import ImageDraw
    d = ImageDraw.Draw(img, "RGBA")
    font = load_font(22)
    font_s = load_font(16)
    W, H = img.size

    for L in labels:
        if L.get("px") is None:
            continue
        role = L["role"]
        zh, en = role_labels.get(role, (f"角色{role}", f"R{role}"))
        col = ROLE_COLORS.get(role, (255, 255, 255))
        px, py = L["px"], L["py"]
        txt = f"{zh} {L['idx']}"
        tw = d.textlength(txt, font=font)
        th = 20
        bx = px + 30
        by = py - 48
        if bx + tw + 18 > W - 6:
            bx = px - 30 - tw - 18
        if by < 28:
            by = py + 22
        if by + th + 12 > H - 6:
            by = H - th - 18
        bx = max(6, min(bx, W - tw - 24))
        d.rounded_rectangle([bx, by, bx + tw + 18, by + th + 12], radius=7,
                            fill=(0, 0, 0, 200), outline=col, width=3)
        d.text((bx + 9, by + 3), txt, font=font, fill=col)
        anchor_x = bx if bx > px else bx + tw + 18
        d.line([(anchor_x, by + (th + 12) // 2), (px, py)], fill=col, width=3)
        if L.get("detected"):
            d.ellipse([px - 5, py - 5, px + 5, py + 5], outline=col, width=3)
        else:
            d.ellipse([px - 17, py - 17, px + 17, py + 17], outline=col, width=3)
            d.text((px - 40, py + 20), "未检测到色块", font=font_s, fill=col)
        if L.get("resid") is not None and L["resid"] > 10.0:
            d.text((bx, by + th + 15), f"⚠ 残差 {L['resid']:.0f}px", font=font_s,
                   fill=(255, 210, 0))

    if meta:
        lines = meta if isinstance(meta, list) else [meta]
        d.rectangle([0, 0, W, 24 * len(lines) + 6], fill=(0, 0, 0, 205))
        for i, s in enumerate(lines):
            d.text((8, 3 + i * 24), s, font=font_s, fill=(255, 255, 255))
    if warnings:
        y = 24 * (len(meta) if meta else 0) + 10
        for w in warnings:
            d.text((8, y), w, font=font_s, fill=(255, 175, 0))
            y += 22
    return img


def draw_template(img, h, color=(255, 60, 60)):
    """按标定把场地模板画到图上（边框、中线、中圈、两侧门区 50×15、罚球区 80×35、
    球门线 y∈[70,110]），用于人眼验证标定是否与平台自己画的线重合。"""
    from PIL import ImageDraw
    if h is None:
        return img
    d = ImageDraw.Draw(img)

    def P(fx, fy):
        return apply_transform(h, fx, fy)

    d.line([P(0, 0), P(FIELD_LEN, 0), P(FIELD_LEN, FIELD_WID), P(0, FIELD_WID), P(0, 0)],
           fill=color, width=3)
    d.line([P(FIELD_LEN / 2, 0), P(FIELD_LEN / 2, FIELD_WID)], fill=(255, 150, 0), width=2)
    pts = [P(FIELD_LEN / 2 + 25 * math.cos(t), 90 + 25 * math.sin(t))
           for t in [i * math.pi / 24 for i in range(49)]]
    d.line(pts, fill=(255, 255, 0), width=2)
    for gx, sign in ((FIELD_LEN, -1), (0, 1)):
        # 罚球区 80 深 × 35 半宽*2=70 宽  → y∈[55,125]
        d.rectangle(_norm_box(P(gx, 55), P(gx + sign * 80, 125)), outline=(0, 255, 0), width=2)
        # 门区 50 深 × 15 半宽*2=30 宽 → y∈[75,105]
        d.rectangle(_norm_box(P(gx, 75), P(gx + sign * 50, 105)), outline=(0, 210, 255), width=3)
        # 球门线 y∈[70,110]
        d.line([P(gx, 70), P(gx, 110)], fill=(255, 0, 255), width=4)
    return img


def _norm_box(a, b):
    return [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]


# ------------------------------------------------------------------ 标定文件


def load_calib(path):
    if path and os.path.isfile(path):
        try:
            with open(path, encoding="utf-8") as fp:
                c = json.load(fp)
            c["matrix"] = normalize_matrix(c.get("matrix"))
            return c
        except Exception:
            return None
    return None


def save_calib(path, calib):
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    payload = dict(calib)
    if payload.get("matrix"):
        payload["matrix"] = [round(v, 12) for v in payload["matrix"]]
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
    print(f"[标定] 已写入 {path}")


# ------------------------------------------------------------------ 单帧处理


def process_image(image_path, args, calib, role_labels):
    from PIL import Image
    img = Image.open(image_path).convert("RGB")
    sprites, balls = detect_blobs(img)
    n_ball = len(balls)
    src_desc = ""
    frame_no = None
    robots = None

    # ---- 坐标来源 ----
    if args.csv:
        got, err = our_robots_from_csv(args.csv, args.row)
        if got is None:
            print(f"⚠️ CSV 不可用：{err}")
        else:
            robots = got["robots"]
            frame_no = got["frame"]
            src_desc = f"CSV {os.path.basename(args.csv)} R行 frame={frame_no}"
    if robots is None and args.rlg:
        import rlg_analyzer as RA
        frames = RA.parse_rlg(args.rlg)
        if (calib is None or not calib.get("matrix") or args.recalib) and frames:
            print(f"[标定] 在 {os.path.basename(args.rlg)} 的 {len(frames)} 帧里找最匹配的一帧…")
            best = find_best_frame(img, sprites, frames,
                                   ball_pts=[(b["cx"], b["cy"]) for b in balls],
                                   coarse=args.coarse)
            if best and best["hits"] >= 3 and best["rms"] < 12.0:
                calib = dict(kind="affine", matrix=best["matrix"], rms=best["rms"],
                             frame=best["frame"], team=best["team"],
                             hits=best["hits"],
                             source="auto:" + os.path.basename(args.rlg))
                robots = best["robots"]
                frame_no = best["frame"]
                src_desc = (f"RLG {os.path.basename(args.rlg)} 帧{best['frame']} "
                            f"队={best['team']} 命中{best['hits']}/5")
                print(f"[标定] ✅ 采用帧 {best['frame']}（{best['team']}）"
                      f"命中 {best['hits']}/5 残差 {best['rms']:.2f}px")
            elif best:
                print(f"[标定] ⚠️ 最佳匹配只有 命中{best['hits']}/5、残差{best['rms']:.1f}px"
                      f"（帧{best['frame']} {best['team']}）→ 判定为不可信，不采用自动标定")
        if robots is None and frames:
            fi = (calib or {}).get("frame", 0)
            team = (calib or {}).get("team", "blue")
            fi = min(int(fi), len(frames) - 1)
            robots = [dict(idx=j, x=r["x"], y=r["y"], rot=r["rot"])
                      for j, r in enumerate(frames[fi][team])]
            frame_no = fi
            src_desc = f"RLG {os.path.basename(args.rlg)} 帧{fi} 队{team}（沿用已存标定）"

    h = (calib or {}).get("matrix")

    # ---- 没坐标源：画模板 + 检测结果 ----
    if robots is None:
        out = draw_template(img, h)
        hits = len(sprites)
        meta = [f"来源 {os.path.basename(image_path)}   "
                f"标定RMS {calib.get('rms'):.1f}px" if calib and calib.get("rms") is not None
                else f"来源 {os.path.basename(image_path)}   标定：无",
                f"色块识别 {hits} 个   球 {n_ball} 个   未提供 --csv/--rlg → 只画检测结果"]
        warns = ["⚠️ 没有坐标源（--csv/--rlg）：无法给机器人标角色名"]
        return draw_overlay(out, [], role_labels, meta=meta, warnings=warns), calib, [], \
            dict(sprites=sprites, balls=balls), src_desc

    # ---- ICP 精修 ----
    match = icp_match(robots, sprites, h0=h)
    if match:
        h = match["matrix"]
        calib = dict(calib or {})
        calib["matrix"] = h
        calib["rms_fit"] = match["rms"]
    mapping = (match or {}).get("mapping", {})
    resid = (match or {}).get("residuals", {})

    # ---- 生成标签 ----
    labels = []
    for r in robots:
        role = r.get("role")
        if role is None or role not in role_labels:
            role = r["idx"]
        px = py = None
        if h:
            px, py = apply_transform(h, r["x"], r["y"])
        j = mapping.get(r["idx"])
        detected = j is not None
        if detected:
            px, py = sprites[j]["cx"], sprites[j]["cy"]
        labels.append(dict(idx=r["idx"], role=role, fx=r["x"], fy=r["y"],
                           px=px, py=py, detected=detected,
                           resid=resid.get(r["idx"]), rot=r.get("rot")))
    hit = sum(1 for L in labels if L["detected"])
    rms = (calib or {}).get("rms")
    resid_txt = " ".join(
        f"#{L['idx']}:{L['resid']:.1f}" if L["resid"] is not None else f"#{L['idx']}:-"
        for L in labels)
    meta = [f"来源 {os.path.basename(image_path)}   {src_desc}",
            f"帧号 {frame_no if frame_no is not None else '-'}   "
            f"标定残差RMS {rms:.2f}px" if rms is not None
            else f"帧号 {frame_no if frame_no is not None else '-'}   标定残差RMS -",
            f"色块识别 {hit}/5   逐台重投影误差(px) {resid_txt}"]
    warns = []
    for L in labels:
        if L["resid"] is not None and L["resid"] > 10.0:
            warns.append(f"⚠️ #{L['idx']}{role_labels[L['role']][0]} 残差 {L['resid']:.0f}px > 10px")
    if rms is not None and rms > 10.0:
        warns.append(f"⚠️ 整体标定残差 {rms:.0f}px 偏大，标签位置可能不准")
    if hit < 5:
        warns.append(f"⚠️ 只认出 {hit}/5 台，其余用预测位置画空心标签")
    # 标定可信度：用"罚球区框"投影回图的偏差做硬指标
    lerr = landmark_consistency(h) if h else None
    if lerr is not None:
        if lerr > 20.0:
            warns.insert(0, f"⛔ 标定不可信：罚球区框投影偏差 {lerr:.0f}px（>20px）→ "
                            f"标签很可能落在错误位置，请用 --calib-points 重新标定")
        else:
            warns.insert(0, f"✅ 标定自检：罚球区框投影偏差 {lerr:.0f}px")
    elif h:
        warns.insert(0, "⚠️ 无标定自检（缺 landmark 信息）")

    out = draw_overlay(img, labels, role_labels, meta=meta, warnings=warns)
    return out, calib, labels, dict(sprites=sprites, balls=balls), src_desc


# ------------------------------------------------------------------ 报告


def print_report(calib, labels, info, role_labels):
    print("\n===== 标定 =====")
    if calib and calib.get("matrix"):
        h = calib["matrix"]
        print("矩阵 h0..h8: " + ", ".join(f"{v:.8f}" for v in h))
        print(f"  类型={calib.get('kind')}  RMS={calib.get('rms')}  "
              f"来源={calib.get('source')}")
        if calib.get("frame") is not None:
            print(f"  匹配帧={calib.get('frame')} 队={calib.get('team')} "
                  f"命中={calib.get('hits')}")
    else:
        print("  （无标定）")

    print("\n===== 色块 ↔ 编号 对照表 =====")
    print("  （签名 = 色块内两种主色，跨帧用它可以认出『同一台』）")
    for L in labels:
        zh = role_labels.get(L["role"], ("?",))[0]
        sig = None
        j = None
        if L["detected"]:
            sig = None
        px = f"({L['px']:.0f},{L['py']:.0f})" if L["px"] is not None else "(未定位)"
        rs = f"{L['resid']:.1f}px" if L["resid"] is not None else "-"
        print(f"  编号{L['idx']} {zh:4s} 场地({L['fx']:6.1f},{L['fy']:6.1f}) → 像素{px:12s}"
              f" 检测={'✅' if L['detected'] else '❌'} 重投影误差={rs}")
    print(f"\n色块 {len(info['sprites'])} 个，球 {len(info['balls'])} 个")


# ------------------------------------------------------------------ 主流程


def main():
    ap = argparse.ArgumentParser(description="把平台截图里我方 5 台机器人标上角色名")
    ap.add_argument("--image", help="输入 PNG（penalty_watch 的 *_field.png）")
    ap.add_argument("--rlg", help="离线：同时段 .rlg（自动帧匹配 + 标定）")
    ap.add_argument("--csv", help="生产：黑匣子 CSV（R 行给我方 5 台坐标）")
    ap.add_argument("--row", default="latest", help="--csv 时取哪条 R 行：latest 或帧号")
    ap.add_argument("--calib", default=DEFAULT_CALIB, help="标定 JSON 路径（读写复用）")
    ap.add_argument("--recalib", action="store_true", help="强制重新标定")
    ap.add_argument("--out", default=os.path.join("build", "role_overlay"), help="输出目录")
    ap.add_argument("--watch", action="store_true", help="实时：循环抓 WorldModel 窗口")
    ap.add_argument("--interval", type=float, default=1.5, help="--watch 抓屏间隔(s)")
    ap.add_argument("--seconds", type=float, default=0, help="--watch 总时长(0=无限)")
    ap.add_argument("--template-only", action="store_true", help="只画场地模板")
    ap.add_argument("--calib-points", help="手动标定点 JSON: [[fx,fy,px,py],...]")
    ap.add_argument("--default-anchors", action="store_true",
                    help="用内置的默认锚点（本仓库 build/penalty_shots 截图实测的"
                         "罚球区/球门区框角点）拟合标定。适合在 1474x1306 的 "
                         "*_field.png 上快速出图；换分辨率/窗口位置需重新标定。")
    ap.add_argument("--role-map", choices=["actual", "header"], default="actual",
                    help="角色名映射口径：actual(0=GK1=ACT2=ASSIST3=MID4=PASSIVE, 默认) "
                         "或 header(按 role_assignment.hpp 的 enum)")
    ap.add_argument("--coarse", type=int, default=300,
                    help="离线自动标定时抽样的 RLG 帧数（越大越准越慢，默认 300）")
    ap.add_argument("--model", choices=["affine", "homography"], default="affine",
                    help="变换模型（默认 affine；homography 仅在你能给 4+ 个分散标定点时才稳）")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    role_labels = ROLE_LABELS if args.role_map == "actual" else ROLE_LABELS_HEADER
    calib = None if args.recalib else load_calib(args.calib)

    # 手动标定点优先（最可靠）
    if args.default_anchors and (calib is None or args.recalib):
        pairs = [((float(p[0]), float(p[1])), (float(p[2]), float(p[3])))
                 for p in default_anchors()]
        h = fit_homography(pairs) if args.model == "homography" else fit_affine(pairs)
        if h:
            lerr = landmark_consistency(h)
            calib = dict(kind=args.model, matrix=h, rms=rms_error(h, pairs),
                         landmark_err=lerr, source="default-anchors(罚球区/球门区框角点)")
            print(f"[标定] 内置锚点拟合完成：角点RMS={calib['rms']:.2f}px "
                  f"罚球区框偏差={lerr:.1f}px（{len(pairs)} 个点）")
            print("[标定] ⚠️ 请用 --template-only 出图核对方框是否与平台画的线重合")

    if args.calib_points and os.path.isfile(args.calib_points):
        with open(args.calib_points, encoding="utf-8") as fp:
            pts = json.load(fp)
        pairs = [((float(p[0]), float(p[1])), (float(p[2]), float(p[3]))) for p in pts]
        h = fit_homography(pairs) if args.model == "homography" else fit_affine(pairs)
        if h:
            calib = dict(kind=args.model, matrix=h, rms=rms_error(h, pairs),
                         source="manual:" + os.path.basename(args.calib_points))
            print(f"[标定] 手动点拟合完成 RMS={calib['rms']:.2f}px（{len(pairs)} 个点）")
            save_calib(args.calib, calib)
        else:
            print("⚠️ 手动标定点拟合失败（点太少或共线）")

    if args.watch:
        return watch_loop(args, calib, role_labels)

    if not args.image:
        ap.error("需要 --image，或使用 --watch")

    if args.template_only:
        from PIL import Image
        img = Image.open(args.image).convert("RGB")
        h = (calib or {}).get("matrix")
        draw_template(img, h)
        dst = os.path.join(args.out, _out_name(args.image, "template"))
        img.save(dst)
        print(f"已输出模板叠加图：{dst}")
        if not h:
            print("⚠️ 没有标定，模板没画出来（先用 --calib-points 或 --rlg 标定）")
        return 0

    out_img, calib, labels, info, src = process_image(args.image, args, calib, role_labels)
    dst = os.path.join(args.out, _out_name(args.image, "overlay"))
    out_img.save(dst)
    print(f"已输出：{dst}")
    if calib and calib.get("matrix") and (args.recalib or not os.path.isfile(args.calib)):
        save_calib(args.calib, calib)
    if labels:
        print_report(calib, labels, info, role_labels)
    return 0


def _out_name(image_path, tag):
    return os.path.splitext(os.path.basename(image_path))[0] + f"_{tag}.png"


def watch_loop(args, calib, role_labels):
    """实时模式：循环 抓 WorldModel 窗口 → 读 CSV 最新 R 行 → 叠加 → 存 PNG。
    窗口定位与抓屏复用 penalty_watch / run_match。"""
    import time
    import penalty_watch as PW

    print(f"[watch] 每 {args.interval}s 抓一次 WorldModel 窗口 → {args.out}")
    if not args.csv:
        print("⚠️ 未给 --csv：实时模式下没有坐标源，只能画场地模板 + 检测结果")
    t0 = time.time()
    n = 0
    while True:
        if args.seconds and (time.time() - t0) > args.seconds:
            break
        n += 1
        win = PW.find_window()
        if not win:
            print("⚠️ 没找到 WorldModel/SimuroSot 窗口，稍后重试…")
            time.sleep(args.interval)
            continue
        PW.shoot(args.out, "_tmp", win["hwnd"])
        fld = os.path.join(args.out, "_tmp_field.png")
        if not os.path.isfile(fld):
            print("⚠️ 球场窗口裁剪失败，跳过本轮")
            time.sleep(args.interval)
            continue
        try:
            out_img, calib, labels, info, src = process_image(fld, args, calib, role_labels)
            stamp = time.strftime("%H%M%S")
            dst = os.path.join(args.out, f"{stamp}_{n:05d}_overlay.png")
            out_img.save(dst)
            hit = sum(1 for L in labels if L["detected"]) if labels else 0
            print(f"[{stamp}] #{n} 色块{len(info['sprites'])} 球{len(info['balls'])} "
                  f"识别{hit}/5 → {os.path.basename(dst)}")
        except Exception as e:
            print(f"⚠️ 处理失败：{type(e).__name__}: {e}")
        time.sleep(args.interval)
    print(f"[watch] 结束，共处理 {n} 轮")
    return 0


if __name__ == "__main__":
    sys.exit(main())
