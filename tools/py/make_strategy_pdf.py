# -*- coding: utf-8 -*-
"""生成《Hnnu 队伍策略说明书》PDF。

做法：程序化生成排版好的 HTML（截图以 base64 内嵌，单文件自包含），
再用 Edge/Chrome 无头打印成 PDF。不依赖任何第三方 Python 库。

用法：python tools/py/make_strategy_pdf.py
产物：<工作区根目录>/Hnnu策略说明书.pdf
"""
import base64
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = ROOT          # 用户要求：产物直接放工作区根目录
HTML = os.path.join(OUT_DIR, "Hnnu策略说明书.html")
PDF = os.path.join(OUT_DIR, "Hnnu策略说明书.pdf")

# 需要的截图/图（找不到就优雅跳过）
FIGS = [
    ("fig_field", r"build\penalty_shots\review\153147_e01_00_field.png",
     "图 1 · 平台球场窗口（WorldModel 的界面）：左边是 220×180 厘米的球场，我们我方 5 台机器人（黄三角/蓝方块）"
     "与球都在这里；右边是策略菜单与 START、TIME/SCORE、HELP（HELP 里就是平台自带的规则原文）。"),
    ("fig_dialog", r"build\penalty_shots\165855_e15_00_win.png",
     "图 2 · 平台主对话框（SimuroSot5）：显示剩余时间与比分。比分格式是（黄队 : 蓝队）。"),
    ("fig_form", r"docs\work\formation12.png",
     "图 3 · 12 种比赛状态的摆位示意（按蓝队坐标画，黄队镜像）：开球 / 争球 / 点球 / 任意球 / 门球，"
     "以及每种状态里\"主罚方先摆、防守方后摆\"的差别。"),
]


def img_tag(path, caption):
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return ""
    with open(full, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return (f'<figure><img src="data:image/png;base64,{b64}" />'
            f'<figcaption>{caption}</figcaption></figure>')


CSS = """
@page { size: A4; margin: 16mm 14mm; }
* { box-sizing: border-box; }
body { font-family: "Microsoft YaHei", "PingFang SC", sans-serif; font-size: 10.5pt;
       line-height: 1.65; color: #1a1a1a; margin: 0; }
h1 { font-size: 22pt; margin: 0 0 4pt 0; letter-spacing: 1px; }
h2 { font-size: 14pt; margin: 18pt 0 6pt 0; padding: 4pt 8pt; background: #eef3fb;
     border-left: 4px solid #2b6cb0; page-break-after: avoid; }
h3 { font-size: 11.5pt; margin: 12pt 0 4pt 0; color: #2b4a7d; page-break-after: avoid; }
p, li { margin: 3pt 0; }
.sub { color: #666; font-size: 9.5pt; margin-bottom: 10pt; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0; font-size: 9.5pt; }
th, td { border: 1px solid #cbd5e0; padding: 3.5pt 5pt; text-align: left; vertical-align: top; }
th { background: #f2f6fc; }
code, .mono { font-family: Consolas, "Courier New", monospace; font-size: 9.5pt;
              background: #f5f7fa; padding: 0 2pt; border-radius: 2px; }
pre { background: #f5f7fa; border-left: 3px solid #90b4d8; padding: 5pt 7pt; margin: 5pt 0;
      font-family: Consolas, "Courier New", monospace; font-size: 9pt; white-space: pre-wrap; }
figure { margin: 8pt 0; page-break-inside: avoid; text-align: center; }
figure img { max-width: 100%; max-height: 105mm; border: 1px solid #cbd5e0; }
figcaption { font-size: 9pt; color: #555; margin-top: 3pt; text-align: left; }
.box { background: #fffaf0; border: 1px solid #f0c987; padding: 6pt 8pt; margin: 6pt 0; }
.key { background: #f0fff4; border: 1px solid #9ae6b4; padding: 6pt 8pt; margin: 6pt 0; }
ul { margin: 3pt 0 3pt 16pt; padding: 0; }
.small { font-size: 9pt; color: #555; }
hr { border: none; border-top: 1px solid #e2e8f0; margin: 12pt 0; }
.pb { page-break-before: always; }
"""

BODY = """
<h1>Hnnu 队伍策略说明书</h1>
<div class="sub">FIRA SimuroSot 5v5 仿真组 · 2026-09-14 · 本文写给"完全没接触过这个平台的人"</div>

<div class="key">
<b>一分钟看懂我们：</b>我们写一个 <code>Strategy4Blue.dll</code>（以及同源的 <code>Strategy4Yellow.dll</code>），
交给平台加载。平台每 1/40 秒（25 毫秒）问我们一次"你队 5 台机器人左右轮各转多快"，我们回答一次。
<br><b>我们的打法一句话：</b>能射就射 → 射不了就传 → 被围就脱身 → 丢球立刻分层回防，
而<b>所有战术的前提是"绝不犯规"</b>（这个平台判罚极重，见第 5 节）。
<br><b>我们的强项：</b>纪律 + 工程验证体系（每个改动先跑单元测试 → 再跑 50 场无头仿真 → 再上真机）。
</div>

<h2>1 · 平台长什么样（配界面截图）</h2>
<p>比赛跑在两个窗口里：左边是<b>球场窗口</b>（WorldModel），右边是我们策略被调到点球点的瞬间；另一个是<b>主对话框</b>，显示剩余时间和比分。</p>
__FIG_FIELD__
__FIG_DIALOG__
<p class="small">读图要点：球场左上到右下是 220×180 厘米；每队 5 台机器人；
<b>比分格式是（黄队 : 蓝队）</b>——所以看到 <code>(2 : 4)</code> 是我们赢了 4:2。</p>

<h2>2 · 我们每帧做的三件事</h2>
<table>
<tr><th>顺序</th><th>做什么</th><th>输出</th></tr>
<tr><td>① 读世界</td><td>把平台给的 5 台自机、5 台敌机、球（含当前/上一帧/平台预测三份）读进我们的"世界模型"</td><td><code>WorldModel</code>（全队共享的黑板）</td></tr>
<tr><td>② 判局势</td><td>算球权、威胁等级、攻防状态（带 3 帧防抖），并算出 3 个站位参考点</td><td>球权 / 威胁 / 战斗姿态</td></tr>
<tr><td>③ 派活</td><td>5 个角色各跑一次自己的决策，写出左右轮速</td><td>每台机器人的 <code>vl / vr</code></td></tr>
</table>
<p><b>角色是固定的</b>（不是每帧抢活）：0 号=守门员、1 号=主攻、2 号=助攻、3 号=中场、4 号=后卫（清道夫）。</p>

<h2>3 · 12 种比赛状态与摆位</h2>
<p>平台把比赛分成 12 种状态（正常比赛 / 争球 / 开球 / 点球 / 任意球 / 门球，每种的"黄/蓝"代表<b>谁主罚</b>）。
每种状态开始时，平台会要求双方先摆好机器人——这一层由我们的 <code>formation.cpp</code> 负责：</p>
__FIG_FORM__
<div class="box">
<b>摆位层我们自己的设计：</b>坐标一律按蓝队写，再用"镜像"公式换算到黄队
（<code>M(x) = 蓝 ? x : 220−x</code>）⇒ <b>同一份代码两侧通用</b>，这也是"比赛万一分到黄队"能一键切换的原因。
另外两条来自真机复盘的修正：<b>点球时踢球人必须站在球后</b>、<b>开球时人必须在本方半场</b>。
</div>

<h2 class="pb">4 · 九条核心算法（每条：一句话 → 怎么做 → 为什么）</h2>

<h3>4.1 球速跳变滤波 —— 别被"瞬移的球"骗了</h3>
<p><b>一句话：</b>平台在进球/摆位时会把球瞬移（实测一帧跳 117 厘米），直接差分算速度会得到物理上不可能的巨大速度，所以超过 30 厘米/帧就把这一帧的速度记成 0。</p>
<pre>ball.vx = (|本帧x − 上帧x| &gt; 30) ? 0 : 差分值;   // 30cm/帧≈1200cm/s，真实球速只到 ~4cm/帧</pre>
<p><b>为什么阈值是 30：</b>正常球速最高约 22 厘米/帧、复位跳变 100+，30 卡在中间。曾经取 12 → 真机被打 0:3（真球速被误当跳变清零，主攻以为球没动、站着不追 ✗）。</p>

<h3>4.2 球权与威胁分级 —— 决定全队是"压上"还是"回收"</h3>
<p><b>一句话：</b>谁离球更近、并且近到 20 厘米内，就算谁控球；球在我们禁区 → 威胁 1.0，在我们半场 → 0.6，我们控球 → 0.1。</p>
<pre>we_have_ball = (我方最近者到球 &lt; 对方最近者到球) &amp;&amp; (我方最近者 &lt; 20cm);</pre>
<p><b>为什么不用"球在谁半场"：</b>对方压上进攻时球就在我们半场，但那是<b>对方的球</b>——按半场判会让全队误切进攻态、后防空虚。
另外平台也给一个球权字段，我们做成了分层使用：局面明确时按距离算，散球混战时才听平台的。</p>

<h3>4.3 站位锚点 + 15 厘米滞回 —— 让机器人跑直线</h3>
<p><b>一句话：</b>助攻/中场该站哪，每帧都在变（跟着球变）；但我们只在"新目标离旧锚点 ≥15 厘米"时才真的改目标，否则就让它按原目标跑直线。</p>
<pre>if (dist(新目标, 记录的锚点) &gt;= 15.0) { 记录锚点 = 新目标; }   // 差不够就不动</pre>
<p><b>为什么：</b>差速轮不能横着走，改方向必须先转身——目标每帧挪几厘米，机器人就每帧微转一下，走成 S 形、永远追不上、也永远到不了位。
15 厘米这道闸门让路径变成"几段长直线"。<b>防守锚点故意不设滞回</b>（防线上差 15 厘米就是一个空档）。</p>

<h3>4.4 射门：先看"门将挡住了多少"，再决定瞄哪</h3>
<p><b>一句话：</b>把球门看成一个扇面，门将挡住中间一块，我们瞄<b>剩下最宽那段空隙的正中</b>。</p>
<pre>门张角 ±half（球到两门柱的夹角）；
门将遮挡 [gk_mid−gk_half, gk_mid+gk_half]（半径 8cm 在该距离的张角）；
净开口 = 门张角扣除遮挡后的最大连续空隙 → 瞄它的中心。</pre>
<p>再加三道闸门：<b>≤70 厘米无条件射</b>（仿真+真机都验证过的主力区）；<b>70~110 厘米</b>要求净开口 ≥8°、
球前 30 厘米无人挡、且机会质量 ≥0.35；<b>点球</b>走旁路（白送的机会不能被闸门拒掉）。</p>
<p><b>机会质量</b>（0~1，越大越该射）= 0.5×开口分 + 0.3×距离分 + 0.2×球速分。</p>

<h3>4.5 差速轮控制：让机器人"恰好停住"</h3>
<p><b>一句话：</b>命令速度不超过 √(2·加速度·剩余距离)——这是数学上"从当前位置恰好刹车停在目标点"的临界曲线。</p>
<pre>double drive = sqrt(2.0 * kBrakeAccel * max(0.0, de - kStopEps));   // 时间最优制动包线
if (drive &gt; v_max) drive = v_max;</pre>
<p>这就是双积分器时间最优控制里的"切换曲线"（bang-bang 的边界）。改之前速度随距离用 sigmoid 控制，
离目标 5 厘米就命令满速、必然冲过头 → 倒车 → 极限环（实测低速来回蹭 2.3 次/分钟）。
顺带一个已验证的教训：<b>把整条速度律都换成包线，仿真会崩</b>（净胜 +0.3→−2.4），所以只在"到点停"这类任务上用它。</p>

<h3>4.6 避障路径：4 厘米网格 A* + 视线拉直</h3>
<p><b>一句话：</b>把球场切成 4 厘米的格子，用 A*（带方向启发的图搜索）找一条不撞人的路，再把阶梯状路径拉直成 2~4 个折点。</p>
<ul>
<li>直线能走就直接走（绝大多数时候）；</li>
<li>"整格撞到人"的格子判为不可走 ⇒ 网格路径严格不穿人；</li>
<li>边线 4 厘米内也当墙（旧算法完全没有边界概念，路径可以贴线甚至出界）；</li>
<li>实测：绕行长度离理论最优只差 <b>0.9%</b>，200 组随机场景比旧算法多解 7 组，耗时同量级（~20 微秒/次）。</li>
</ul>

<h3>4.7 守门员：预测、封线、不要碰球</h3>
<table>
<tr><th>机制</th><th>做法</th><th>真机教训</th></tr>
<tr><td>预测入球点</td><td>按球速外推到门线，得到"球会从哪个 y 进门"，再按到线时间决定出击深度</td><td>球贴边墙滚时直线外推会算错 ⇒ 加了<b>撞墙反射</b>版预测</td></tr>
<tr><td>门线封堵</td><td>球已经在门框内的轨迹上 → 直接抢门线上的预测落点（优先于清球/站位分支）</td><td>复盘两个丢球：门将当时在追"球侧方 15 厘米的绕行点"，球从旁边进</td></tr>
<tr><td>球外侧禁推</td><td>球已越过自己（比门将更靠门）时，先横向让开，<b>绝不朝球推进</b></td><td>逐帧证据：两个乌龙球的球速方向在"门将碰到球"那一帧翻转成进门</td></tr>
<tr><td>清球先转正</td><td>门前死球要推出去时，机头没对准就先原地转正，再直线推穿</td><td>曾出现门球卡 <b>7.6 秒</b>球一动不动（平台每 5 秒重发一次）</td></tr>
</table>

<h3>4.8 定位球：先"认出"这是我们的点球</h3>
<p><b>一句话：</b>平台在执行期<b>不会</b>把"点球"这个状态告诉我们（实测 16000 帧里"正常比赛"一帧都没出现），
所以我们改用可观测量：<b>球静止在对方罚球点上</b>（实测点 = 门前 39.4 厘米、正中央，两场 14 次摆球完全一致）。</p>
<pre>we_take_penalty = 球静止(&lt;1cm/帧) &amp;&amp; |球x − (对方门线 ∓ 39.4)| &lt; 1.5 &amp;&amp; |球y − 90| &lt; 1.5;</pre>
<p>认出之后：踢球人就地转正、<b>立刻推</b>（真机实测：如果先倒车助跑 0.6 秒，对手会从 34 厘米逼近到 10 厘米把球截走）。</p>

<h3>4.9 防守工具箱</h3>
<table>
<tr><th>工具</th><th>机制</th><th>关键参数</th></tr>
<tr><td>区域断球点</td><td>球轨迹 ∩ 门前拦截线；球朝边墙滚时用<b>反射</b>后的轨迹</td><td>拦截线 = 门前 50 厘米</td></tr>
<tr><td>人盯人</td><td>给 5 个对手打威胁分（离球远近 + 离门远近 + 球是否正传给他 + 是否持球突破），取最高分者贴身</td><td>贴身 16 厘米；预测外推 3 帧；5% 换人滞回；离球/离门 &gt;40 厘米不值得贴</td></tr>
<tr><td>二抢一</td><td>持球者压到门前时，再派一个防守者形成包夹；只派"最近的那一个"，另一个留区域保纵深</td><td>离门 &lt;100 厘米才夹；球在罚球区内要持球者离门 &lt;45 厘米</td></tr>
<tr><td>抢反弹位</td><td>对方射门时，两人分别站在罚球区前缘的左右两侧等门将扑出的第二落点</td><td>只在球朝门速 &gt;8 厘米/帧时才抢</td></tr>
</table>

<h2 class="pb">5 · 为什么"别犯规"比战术更值钱</h2>
<table>
<tr><th>犯规</th><th>代价（规则原文）</th><th>我们的实测</th></tr>
<tr><td>对方门区内 2+ 人 / 单人停留 &gt;20 周期</td><td>判对方点球</td><td>纪律修复前我们一场<b>送 12 次</b>点球 ✗，修复后 3~6 次</td></tr>
<tr><td>禁止推球区（四角黄区）推球</td><td>每 4 次给对方 +1 球 + 判争球</td><td>实测 4 次 = 白送 1 球</td></tr>
<tr><td>僵局（门区外 100 周期无进展）</td><td>判争球</td><td>我们的一场争球重置曾达 100~110 次</td></tr>
</table>
<p>所以我们把纪律做成了代码里的硬约束：门区停留计数器（纯停留 8 帧、总时长 20 帧就撤出）、
角区不许推球、死球期不碰球（但保留"球静止 100 帧还没人来处理就主动去推"的超时逃生，防死球变僵局）。</p>

<h2>6 · 我们怎么验证（这是我们认为最值钱的部分）</h2>
<ul>
<li><b>单元测试</b>：<code>offline_test.exe</code> 覆盖 186 个函数；规矩是"先让测试失败，再改代码"（例如给门将新分支时，故意关掉它让测试报 FAIL，证明测试真的能抓住问题）。</li>
<li><b>无头仿真</b>：<code>sim_bench</code> 不开窗口跑完整比赛，50 场约 20 秒，自动出净胜球/射门/纪律对照表。</li>
<li><b>真机黑匣子</b>：临时变体 DLL 把平台每一帧的原始字段写成 CSV（游戏状态、球权、球速…），
用来回答"文档里查不到、只能实测"的问题（例如发现平台在执行期根本不报"点球"状态）。</li>
<li><b>逐帧复盘</b>：<code>.rlg</code> 录像逐帧 + 平台官方日志（事件日记）交叉验证；进球/丢球/判罚都能定位到帧号。</li>
</ul>
<div class="box">
<b>一个真实案例（点球 0/9 的根因）：</b>官方规则速查里把状态名写成"黄/蓝<b>被罚</b>"，而平台实际语义是"黄/蓝<b>主罚</b>"。
我们通过三条独立证据（官方 demo 的摆位回调、日志比分归因、录像里球被摆在哪个罚球点）确认后修正了代码和文档——
此前 9 次点球"一次都没踢"就是这一个字造成的。
</div>

<h2>7 · 目前战绩与已知短板</h2>
<table>
<tr><th>项目</th><th>现状</th></tr>
<tr><td>近期战绩（对官方 demo）</td><td>近 6 场 3 胜 1 平 2 负（5:3、1:0、4:2、1:1 ｜ 0:2、0:1）</td></tr>
<tr><td>点球转化率</td><td><b>约 11%（18 次 2 球）</b> —— 最大得分空间，也是当前重点</td></tr>
<tr><td>丢球性质</td><td>最近两场 3 个丢球<b>全部疑似乌龙</b>（球速方向证据：门将碰球那一帧球转向自家门）</td></tr>
<tr><td>已知短板</td><td>罚点球时瞄准转正白耗 1.12 秒；机会质量还没算"门将够不够得到"这个时间维度</td></tr>
<tr><td>已修复</td><td>门球卡死 7.6 秒 → 转身即推；点球识别 18/18 成功；定位球重发 0 次</td></tr>
</table>

<h2>8 · 附录：模块分工与参数速查</h2>
<table>
<tr><th>模块</th><th>文件</th><th>职责</th></tr>
<tr><td>世界模型 / 队伍上下文</td><td><code>world_model.cpp</code> <code>team.hpp</code></td><td>平台状态 → 内部结构；球速滤波；坐标系与镜像</td></tr>
<tr><td>局势与站位</td><td><code>situation.cpp</code> <code>strategy.cpp</code></td><td>球权/威胁/攻防状态机/三个站位锚点</td></tr>
<tr><td>角色调度</td><td><code>roles.cpp</code> <code>role_assignment.cpp</code></td><td>门将/主攻/助攻/中场/后卫的决策与调度</td></tr>
<tr><td>射门 / 传球</td><td><code>shoot.cpp</code> <code>pass.cpp</code></td><td>净开口角、机会质量、传球点选择</td></tr>
<tr><td>防守工具箱</td><td><code>defense.cpp/hpp</code></td><td>断球点（含反射）、人盯人、二抢一、抢反弹位</td></tr>
<tr><td>运动与路径</td><td><code>motion.cpp</code> <code>route.cpp</code></td><td>制动包线、到点定向、4 厘米网格 A*</td></tr>
<tr><td>定位球摆位</td><td><code>formation.cpp</code></td><td>12 态摆位、摆球</td></tr>
</table>
<table>
<tr><th>参数</th><th>值</th><th>含义</th></tr>
<tr><td>射程闸门</td><td>≤70cm 无条件 / 70~110cm 需净开口 ≥8°</td><td>超过 110 厘米不射，改带球或传球</td></tr>
<tr><td>机会质量门限</td><td>≥0.35（仅远射档判）</td><td>0.5×开口 + 0.3×距离 + 0.2×球速</td></tr>
<tr><td>到点速度律</td><td>v = min(150, √(2·400·(s−1.5)))</td><td>时间最优制动包线（cm、cm/s）</td></tr>
<tr><td>避障网格</td><td>4 厘米 / 55×45 格</td><td>A* + 视线拉直</td></tr>
<tr><td>锚点滞回</td><td>15 厘米</td><td>防守锚点不设</td></tr>
<tr><td>防守贴身</td><td>16 厘米、外推 3 帧</td><td>人盯人</td></tr>
<tr><td>罚球点</td><td>门前 39.4 厘米、正中央</td><td>实测值，用于识别我方点球</td></tr>
</table>

<hr>
<p class="small">本文由 <code>tools/py/make_strategy_pdf.py</code> 自动生成；
截图来自真机录像窗口（<code>build/penalty_shots/</code>），12 态摆位图由
<code>tools/py/formation_diagram.py</code> 生成。所有数据均来自真机录像（.rlg）与平台官方日志的逐帧复盘。</p>
"""


def main():
    html = BODY
    for key, path, cap in FIGS:
        html = html.replace(f"__FIG_{key[4:].upper()}__", img_tag(path, cap))
    # 未命中的占位符清掉
    for key, _, _ in FIGS:
        html = html.replace(f"__FIG_{key[4:].upper()}__", "")
    doc = (f"<!DOCTYPE html><html><head><meta charset='utf-8'>"
           f"<title>Hnnu 队伍策略说明书</title><style>{CSS}</style></head>"
           f"<body>{html}</body></html>")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(HTML, "w", encoding="utf-8") as f:
        f.write(doc)
    print(f"✓ HTML: {HTML} ({os.path.getsize(HTML)//1024} KB)")

    browsers = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
                r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"]
    exe = next((b for b in browsers if os.path.exists(b)), None)
    if not exe:
        print("✗ 没找到 Edge/Chrome，无法打印 PDF")
        return 1
    url = "file:///" + HTML.replace("\\", "/").replace(" ", "%20")
    cmd = [exe, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
           f"--print-to-pdf={PDF}", url]
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=180)
    if os.path.exists(PDF):
        print(f"✓ PDF : {PDF} ({os.path.getsize(PDF)//1024} KB)")
        return 0
    print("✗ 打印失败：", (r.stdout or "")[-500:], (r.stderr or "")[-500:])
    return 1


if __name__ == "__main__":
    sys.exit(main())
