# -*- coding: utf-8 -*-
"""生成《Hnnu 队伍介绍》Word 文档（队员介绍 + 参赛/备赛经历）。

排版要求：正文 宋体 小四(12pt)、行距 1.3 倍；
标题黑体，三级标题体系；每位队员附照片，图片居中；不少于 3 页。

做法：直接拼 OOXML（无需第三方库），再用 Word 导出 PDF 校验页数。
产物：<工作区根目录>/Hnnu队伍介绍.docx
"""
import os
import zipfile
from xml.sax.saxutils import escape

from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PHOTO_DIR = os.path.abspath(os.path.join(ROOT, "..", "队员"))
OUT = os.path.join(ROOT, "Hnnu队伍介绍.docx")
MEDIA_TMP = os.path.join(ROOT, "build", "team_intro_media")

# 西文/数字字体（中文一律由 eastAsia="宋体"/"黑体" 控制）
LATIN = "Times New Roman"
SONG = "宋体"
HEI = "黑体"

SZ_TITLE = 44   # 22pt 二号
SZ_H1 = 32      # 16pt 三号
SZ_H2 = 28      # 14pt 四号
SZ_H3 = 24      # 12pt 小四
SZ_BODY = 24    # 12pt 小四

# 行距 1.3 倍。
# 注意：lineRule="auto" 时 w:line 的单位是 1/20 磅（不是百分比），
# Word 的“单倍行距”= 字号 × 1.17（Times New Roman 行高系数），
# 即 12pt 正文单倍 ≈ 14.04pt = 281 twips；×1.3 ≈ 365 twips。
LINE = 365
LINERULE = "auto"
JUST = '<w:jc w:val="both"/>'
BOTH_IND = '<w:ind w:firstLineChars="200" w:firstLine="480"/>'

# ---------------------------------------------------------------- 图片准备


def prepare_photo(name):
    """把队员照片统一转成 RGB，返回 (临时文件名, 宽, 高)。"""
    src = os.path.join(PHOTO_DIR, name + ".jpg")
    im = Image.open(src)
    im = ImageOps.exif_transpose(im)
    if im.mode != "RGB":
        im = im.convert("RGB")
    os.makedirs(MEDIA_TMP, exist_ok=True)
    dst = os.path.join(MEDIA_TMP, name + ".jpg")
    if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(src):
        im.save(dst, "JPEG", quality=92)
    return name + ".jpg", im.size[0], im.size[1]


def photo_runs(fname, px_w, px_h, cx=1190700, max_cy=2079000):
    """等比缩放：宽 3.15cm（cx=1190700 EMU，1cm=360000 EMU），高按原图比例，最高 5.5cm。"""
    ext = round(cx * px_h / px_w)
    if ext > max_cy:
        ext = max_cy
    return (
        '<w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0">'
        '<wp:extent cx="%d" cy="%d"/><wp:effectExtent l="0" t="0" r="0" b="0"/>'
        '<wp:docPr id="1" name="%s"/>'
        '<a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main">'
        '<a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">'
        '<pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">'
        '<pic:nvPicPr><pic:cNvPr id="1" name="%s"/><pic:cNvPicPr/></pic:nvPicPr>'
        '<pic:blipFill><a:blip r:embed="rId%s"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>'
        '<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="%d" cy="%d"/></a:xfrm>'
        '<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>'
        '</pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r>'
        % (cx, ext, escape(fname), escape(fname), "%s", cx, ext)
    )


# ---------------------------------------------------------------- XML 构件


def para(text="", style=None, align=None, indent=None, before=0, after=0,
         size=SZ_BODY, cjk=SONG, bold=False, spacing=None):
    ppr = ["<w:pPr>"]
    if style:
        ppr.append('<w:pStyle w:val="%s"/>' % style)
    if align:
        ppr.append('<w:jc w:val="%s"/>' % align)
    if indent:
        ppr.append(indent)
    if bold:
        ppr.append("<w:rPr><w:b/></w:rPr>")
    sp = spacing or '<w:spacing w:before="%d" w:after="%d" w:line="%d" w:lineRule="%s"/>' % (
        before, after, LINE, LINERULE)
    ppr.append(sp)
    ppr.append("</w:pPr>")
    run = ""
    if text:
        run = ('<w:r><w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/>%s'
               '<w:sz w:val="%d"/><w:szCs w:val="%d"/></w:rPr><w:t xml:space="preserve">%s</w:t></w:r>'
               % (LATIN, LATIN, cjk, "<w:b/>" if bold else "", size, size, escape(text)))
    return "<w:p>" + "".join(ppr) + run + "</w:p>"


def h1(text):
    return para(text, style="Heading1", size=SZ_H1, cjk=HEI, bold=True, before=240, after=100)


def h2(text):
    return para(text, style="Heading2", size=SZ_H2, cjk=HEI, bold=True, before=160, after=80)


def h3(text):
    return para(text, style="Heading3", size=SZ_H3, cjk=HEI, bold=True, before=120, after=60)


def body(text):
    # 段后 2 磅，避免“整页密不透风”；行距仍为 1.3 倍
    return para(text, align="both", indent=BOTH_IND,
                spacing='<w:spacing w:before="0" w:after="40" w:line="%d" w:lineRule="%s"/>' % (LINE, LINERULE))


def photo_para(runs):
    return para("", align="center", spacing='<w:spacing w:before="80" w:after="40" w:line="240" w:lineRule="auto"/>').replace(
        "</w:p>", runs + "</w:p>")


def cell(text, width, bold=False, size=SZ_BODY, cjk=SONG, align="center"):
    return ('<w:tc><w:tcPr><w:tcW w:w="%d" w:type="dxa"/>'
            '<w:vAlign w:val="center"/></w:tcPr>'
            '<w:p><w:pPr><w:jc w:val="%s"/>'
            '<w:spacing w:before="40" w:after="40" w:line="240" w:lineRule="auto"/>'
            '%s</w:pPr>'
            '<w:r><w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/>%s'
            '<w:sz w:val="%d"/><w:szCs w:val="%d"/></w:rPr>'
            '<w:t xml:space="preserve">%s</w:t></w:r></w:p></w:tc>'
            % (width, align, "<w:rPr><w:b/></w:rPr>" if bold else "",
               LATIN, LATIN, cjk, "<w:b/>" if bold else "", size, size, escape(text)))


def page_break():
    """在当前位置插入分页符（新起一页）。"""
    return ('<w:p><w:pPr><w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
            '<w:r><w:br w:type="page"/></w:r></w:p>')


def keep_next(p_xml):
    """给段落加上 keepNext，保证标题/图片不会与后面的正文分页。"""
    return p_xml.replace("<w:pPr>", "<w:pPr><w:keepNext/>", 1)


def table(rows, widths):
    """rows: 二维文本；widths: 每列宽度(twips)，合计等于版心宽 8505。"""
    grid = "".join('<w:gridCol w:w="%d"/>' % w for w in widths)
    out = ['<w:tbl><w:tblPr><w:tblW w:w="%d" w:type="dxa"/><w:jc w:val="center"/>'
           '<w:tblBorders>'
           '<w:top w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '<w:left w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '<w:bottom w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '<w:right w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '<w:insideV w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
           '</w:tblBorders>'
           '<w:tblCellMar><w:top w:w="20" w:type="dxa"/><w:left w:w="60" w:type="dxa"/>'
           '<w:bottom w:w="20" w:type="dxa"/><w:right w:w="60" w:type="dxa"/></w:tblCellMar>'
           '</w:tblPr><w:tblGrid>%s</w:tblGrid>' % (sum(widths), grid)]
    for i, row in enumerate(rows):
        tr = ["<w:tr>"]
        if i == 0:
            tr.append('<w:trPr><w:tblHeader/><w:cantSplit/><w:jc w:val="center"/></w:trPr>')
        else:
            tr.append('<w:trPr><w:cantSplit/><w:jc w:val="center"/></w:trPr>')
        for j, txt in enumerate(row):
            tr.append(cell(txt, widths[j], bold=(i == 0), align="center"))
        tr.append("</w:tr>")
        out.append("".join(tr))
    out.append("</w:tbl>")
    out.append(para("", spacing='<w:spacing w:before="0" w:after="0" w:line="120" w:lineRule="auto"/>'))
    return "".join(out)


# ---------------------------------------------------------------- 正文内容


def build_members():
    """队员介绍：每人 居中照片 + 姓名(小标题) + 基本信息/简介/队内职责 分段。"""
    members = [
        dict(
            name="张玥琦（队长）",
            photo="张玥琦",
            basic="张玥琦，女，湖南师范大学信息科学与工程学院计算机科学与技术（师范）专业本科生，"
                  "现读大二。在校学习数据结构、算法设计与分析、计算机组成原理、面向对象程序设计等专业课程，"
                  "具备较完整的计算机专业知识结构。",
            intro=[
                "张玥琦现任本队队长，负责全队的统筹与协调：确定各阶段目标与任务分工，跟踪每名队员的模块进度，"
                "组织联调与问题复盘，并把关最终提交材料的完整性与规范性。在技术工作中，她主要参与策略系统"
                "总体框架的设计与集成，把各队员独立完成的模块串成一条可运行、可复现的完整链路。",
                "作为师范专业的学生，她习惯把复杂问题拆成“讲得清楚”的步骤再动手：先明确输入、输出和判断条件，"
                "再划分模块并逐一验证。这一习惯直接影响了本队的开发方式——先定接口、各自实现、再统一联调，"
                "因此四个人的代码能够较快地拼装成一个整体，也便于在出问题时快速定位到具体模块。",
                "在队内，她同时承担“对外接口”的角色：负责比赛规则与提交要求的解读，把规则条文转成全队能执行的"
                "任务清单（例如比赛状态的判断方式、被判罚的行为红线等），避免出现“程序能跑但违规”的情况。",
            ],
            duty="队内职责：队伍统筹与任务分工；策略系统总体框架设计与集成；比赛规则解读与材料整理；"
                 "联调测试组织与进度把控。",
        ),
        dict(
            name="韦玉宜",
            photo="韦玉宜",
            basic="韦玉宜，女，湖南师范大学信息科学与工程学院人工智能专业本科生，2024 级。",
            intro=[
                "韦玉宜专业基础扎实，积极投身各类科技创新赛事以积累实战经验，具备良好的专业素养与项目实践能力。"
                "在多次参赛过程中，她系统锻炼了数理建模分析、计算机设计开发、人工智能创新方案设计、项目打磨"
                "与团队协作等核心能力，能够较快地把一个想法推进到可演示、可提交的成果。",
                "她参与的竞赛覆盖面较广：湖南省“金种子杯”、“互联网+”大学生创新创业大赛、全国大学生计算机"
                "设计大赛、“智创未来”校园 AI 创新创业挑战赛等。不同赛事的评审侧重点不同，这段经历让她形成了"
                "“先看评分规则，再定工作优先级”的习惯，也让本队在有限时间内能把力气用在评委会真正关注的地方。",
                "在本项目中，她主要负责无球进攻方向的传球与接应部分：判断“传给谁最稳”，检查传球路线是否被"
                "对方挡住，以及无球队员应当跑到哪个位置接应，从而让持球队友始终有出球点。",
            ],
            duty="队内职责：无球进攻模块（传球决策与接应跑位）；负责传球路线避挡判断与接应点选取，"
                 "并参与比赛录像复盘与策略迭代。",
        ),
        dict(
            name="尹肇兴",
            photo="尹肇兴",
            basic="尹肇兴，男，天津市人，现于湖南长沙求学，本科在读。",
            intro=[
                "尹肇兴在校期间长期担任学生工作：2022 年至今任班级团支书，2023 年任新生辅导员、校长助理，"
                "2024 年任校级优秀共青团干部，2025 年任贫困生评定小组成员。多次获得区级优秀学生干部、"
                "校级优秀学生、校级优秀共青团员、校级优秀学生干部等荣誉。这些经历让他养成了先定计划、"
                "再分解任务、最后按期交付的工作习惯，在队伍中主要承担执行与协调类工作。",
                "他在文艺与竞赛方面同样活跃：2024 年出演《雷雨》《德龄与慈禧》《群猴》等话剧并获市级一、二等奖，"
                "2023 年与 2026 年先后获得天津市文艺展演市级二等奖、一等奖及“万水千山”艺术比赛三等奖，"
                "2025 年获评舞台剧艺术特长生，2026 年获数学建模校赛三等奖。",
                "在本项目中，他主要参与防守与工程支持方向的工作：协助守门员与防守站位的逻辑整理，配合完成"
                "比赛日志的整理与复盘，并承担队伍日常事务（材料汇总、会议组织、进度记录）的推进。",
            ],
            duty="队内职责：防守与工程支持（守门员/防守逻辑整理、运行测试与日志复盘）；队伍日常事务与进度记录。",
        ),
        dict(
            name="陈奕睿",
            photo="陈奕睿",
            basic="陈奕睿，男，湖南师范大学信息科学与工程学院计算机科学与技术（师范）专业本科生。",
            intro=[
                "陈奕睿与张玥琦同专业，具备计算机科学与技术（师范）方向的课程基础，熟悉程序设计、数据结构、"
                "算法与计算机系统等专业知识，并具备把技术方案讲清楚、写明白的表达能力。",
                "在本项目中，他主要参与持球进攻方向的工作：判断球在脚下时应当射门还是继续带球推进，"
                "选择合适的射门角度并避开对方守门员的封堵，同时配合完成运动控制参数的现场调试。",
            ],
            duty="队内职责：持球进攻模块（射门决策与带球推进）；运动控制参数调试；协助完成策略测试与数据整理。",
        ),
    ]
    return members


MEMBERS = build_members()

PHOTO_ROWS = []          # (fname, px_w, px_h)
PHOTO_ORDER = []


def document_body():
    parts = []
    parts.append(para("Hnnu 队伍介绍", align="center", size=SZ_TITLE, cjk=HEI, bold=True,
                      spacing='<w:spacing w:before="0" w:after="60" w:line="%d" w:lineRule="%s"/>' % (LINE, LINERULE)))
    parts.append(para("——成员介绍与参赛备赛情况", align="center", size=SZ_H3, cjk=HEI,
                      spacing='<w:spacing w:before="0" w:after="200" w:line="%d" w:lineRule="%s"/>' % (LINE, LINERULE)))

    parts.append(h1("一、队伍简介"))
    parts.append(body("Hnnu 队来自湖南师范大学信息科学与工程学院，由四名本科生组成，"
                      "参加 RoboCup 中国赛 FIRA 小型组仿真 5vs5 项目。四名队员分别来自计算机科学与技术（师范）"
                      "与人工智能两个专业，横跨不同年级，形成“高年级带低年级、专业互补”的结构。"))
    parts.append(body("仿真 5vs5 项目的比赛形式是：官方平台提供一块虚拟球场和双方各五个机器人，"
                      "参赛队伍需要自主编写一套完整的比赛策略程序，让五个机器人在没有人工干预的情况下完成"
                      "开球、传球、带球、射门、防守、守门等全部动作，并遵守比赛规则。换句话说，"
                      "我们要做的是一支“会自己踢球的队伍”：既要会进攻，也要会防守，还要在裁判吹哨后立刻站到正确的位置上。"))
    parts.append(body("针对这一任务，队伍按“球在谁脚下”把工作分成四条线：全局调度（决定谁干什么）、"
                      "持球进攻（带球与射门）、无球进攻（传球与接应）、防守与工程（守门、防守、测试与复盘）。"
                      "每人主责一条线、各自独立实现模块，再按统一接口拼成整体。"))

    parts.append(h1("二、团队成员一览"))
    parts.append(table(
        [["姓名", "年级/专业", "队内分工"],
         ["张玥琦（队长）", "大二·计算机科学与技术（师范）", "队伍统筹；总体框架设计与集成"],
         ["韦玉宜", "2024 级·人工智能", "无球进攻：传球决策与接应跑位"],
         ["尹肇兴", "本科在读", "防守与工程支持；日常事务与复盘"],
         ["陈奕睿", "计算机科学与技术（师范）", "持球进攻：射门决策与带球推进"]],
        [1500, 3300, 3705]))

    parts.append(h1("三、队员介绍"))
    for i, m in enumerate(MEMBERS):
        fname, px_w, px_h = prepare_photo(m["photo"])
        PHOTO_ORDER.append((fname, px_w, px_h))
        if i > 0:
            parts.append(page_break())      # 每位队员各自新起一页，照片与介绍在同一页
        parts.append(keep_next(h2(m["name"])))
        parts.append(keep_next(photo_para(photo_runs(fname, px_w, px_h))))
        parts.append(keep_next(h3("基本信息")))
        parts.append(body(m["basic"]))
        parts.append(h3("个人简介"))
        for t in m["intro"]:
            parts.append(body(t))
        parts.append(h3("队内职责"))
        parts.append(body(m["duty"]))
        if i < len(MEMBERS) - 1:
            parts.append(para("", spacing='<w:spacing w:before="0" w:after="0" w:line="120" w:lineRule="auto"/>'))

    parts.append(h1("四、队内分工与协作方式"))
    parts.append(body("队伍没有按“足球角色”分工，而是按数据流分。这样做的好处是接口清晰：每个模块只关心"
                      "自己的输入和输出，谁改坏了很容易查出来。四条线对应的工作内容如下表。"))
    parts.append(table(
        [["分工线", "负责内容", "具体工作"],
         ["全局调度", "谁该干什么、现在是什么比赛状态", "整理场上信息，给五台机器人分配角色，计算站位参考点，"
                                              "处理开球、争球、点球、任意球、门球等十二种比赛状态的摆位"],
         ["持球进攻", "球在我方脚下时怎么处理", "决定射门还是带球推进，选择射门角度并避开守门员，"
                                        "让机器人平稳走到目标位置"],
         ["无球进攻", "球在队友脚下时我站哪、传给谁", "选择接应队友，判断传球路线是否被挡，"
                                          "无球队员站到合适的接应位置"],
         ["防守与工程", "球在对方脚下时站哪 + 后勤", "守门员站位与扑救、区域防守与拦截、"
                                          "离线测试、比赛日志复盘、构建与调参记录"]],
        [1200, 2600, 4705]))
    parts.append(body("协作上采用“先定接口、再各自实现、最后统一联调”的方式：接口一旦确定就不随意更改，"
                      "各人只在自己负责的文件里修改代码，联调时由队长统一跑测试、看复盘日志，"
                      "把问题定位到具体模块后再回到对应队员。每轮改动都会记录参数与原因，"
                      "避免出现“改了但说不清为什么改”的情况。"))

    parts.append(h1("五、参赛与备赛经历"))
    parts.append(body("本队围绕 FIRA 小型组仿真 5vs5 项目完成了从零搭建的全过程："
                      "先读懂比赛规则与官方平台接口，再设计策略框架，随后按模块分工实现，"
                      "最后用真机平台反复对局、用比赛日志复盘迭代。整个备赛过程分为四个阶段。"))
    parts.append(h3("第一阶段：规则与平台"))
    parts.append(body("通读赛事规则与平台使用手册，明确场地尺寸、机器人数量与编号、比赛状态种类和判罚红线；"
                      "确认平台对策略程序的加载方式与接口要求，跑通“策略程序 → 平台加载 → 机器人动起来”的"
                      "第一条链路，并让队伍名称正确显示在平台控制台上。"))
    parts.append(h3("第二阶段：框架与模块"))
    parts.append(body("确定“获取全场信息 → 判断局势 → 分配角色 → 各角色执行 → 回写轮速”的总体框架，"
                      "并把工作拆成四条分工线。各队员在自己的模块内独立实现，用离线测试逐条验证："
                      "机器人能否走到指定位置、能否追上球、传球能否选出正确队友、守门员能否守住球门。"))
    parts.append(h3("第三阶段：真机联调"))
    parts.append(body("把各模块拼成完整策略后，在官方平台上与陪练对手反复对局，重点解决三类问题："
                      "角色分配在不同局势下反复切换导致的站位抖动；射门、传球等决策在真实平台上的成功率；"
                      "开球、争球、点球等定位球的摆位是否越界或违规。"))
    parts.append(h3("第四阶段：复盘与迭代"))
    parts.append(body("每场对局都保留平台日志，赛后用自建的分析工具逐帧回看：球是怎么丢的、"
                      "射门为什么没进、守门员当时站在哪里。把结论落到具体参数或逻辑上，再跑下一轮验证，"
                      "形成“对局 → 复盘 → 改参 → 再对局”的迭代闭环。"))
    parts.append(h3("队员个人参赛经历"))
    parts.append(body("韦玉宜：曾先后参与湖南省“金种子杯”、全国大学生数学建模竞赛、“互联网+”大学生"
                      "创新创业大赛、全国大学生计算机设计大赛、“智创未来”校园 AI 创新创业挑战赛等多项赛事"
                      "并荣获奖项。"))
    parts.append(body("尹肇兴：2026 年数学建模校赛三等奖，2026 年“万水千山”艺术比赛三等奖；"
                      "2024 年出演《雷雨》《德龄与慈禧》《群猴》等话剧并获市级一、二等奖；"
                      "2023 年获天津市文艺展演市级二等奖、2026 年获该展演一等奖；"
                      "2023 年、2025 年两度获评校级优秀共青团干部。"))
    parts.append(body("张玥琦、陈奕睿：两人为计算机科学与技术（师范）专业同专业队员，"
                      "主要参与本项目的策略开发与测试工作，共同完成框架集成、持球进攻模块实现与真机联调。"))

    parts.append(h1("六、结语"))
    parts.append(body("Hnnu 队是一支从零起步的队伍：没有现成代码可用，规则、平台、策略都要自己啃。"
                      "备赛过程中，我们把一个大问题拆成四条清晰的分工线，靠接口约定和反复联调把四个人的工作"
                      "拼成一个整体，也用比赛日志把每一次失败变成了下一轮的改进依据。"))
    parts.append(body("无论最终成绩如何，这段经历已经让每名队员完整地走了一遍“需求分析—方案设计—编码实现—"
                      "测试复盘”的工程流程。这也是我们参加这项赛事最看重的收获。"))
    parts.append(para("说明：本文档正文采用宋体小四、1.3 倍行距排版；文中涉及的个人获奖与经历信息，"
                      "以队员本人提供的材料为准，正式提交前可对照证书逐条核校。",
                      align="both", indent=BOTH_IND, size=21, cjk=SONG))
    return "".join(parts)


# ---------------------------------------------------------------- 打包 docx

STYLES = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
          '<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">'
          '<w:docDefaults><w:rPrDefault><w:rPr>'
          '<w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s" w:cs="%s"/>'
          '<w:sz w:val="%d"/><w:szCs w:val="%d"/></w:rPr></w:rPrDefault>'
          '<w:pPrDefault><w:pPr><w:spacing w:after="0" w:line="%d" w:lineRule="%s"/>'
          '<w:jc w:val="both"/></w:pPr></w:pPrDefault></w:docDefaults>'
          '<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/>'
          '<w:pPr><w:spacing w:after="0" w:line="%d" w:lineRule="%s"/><w:jc w:val="both"/></w:pPr>'
          '<w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/><w:sz w:val="%d"/></w:rPr></w:style>'
          % (LATIN, LATIN, SONG, LATIN, SZ_BODY, SZ_BODY, LINE, LINERULE, LINE, LINERULE, LATIN, LATIN, SONG, SZ_BODY))


def heading_style(sid, name, size, before, after, outline):
    return ('<w:style w:type="paragraph" w:styleId="%s"><w:name w:val="%s"/>'
            '<w:basedOn w:val="Normal"/>'
            '<w:pPr><w:keepNext/><w:outlineLvl w:val="%d"/>'
            '<w:spacing w:before="%d" w:after="%d" w:line="%d" w:lineRule="%s"/></w:pPr>'
            '<w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/><w:b/>'
            '<w:sz w:val="%d"/><w:szCs w:val="%d"/></w:rPr></w:style>'
            % (sid, name, outline, before, after, LINE, LINERULE, LATIN, LATIN, HEI, size, size))


STYLES += (heading_style("Heading1", "heading 1", SZ_H1, 240, 80, 0)
           + heading_style("Heading2", "heading 2", SZ_H2, 160, 60, 1)
           + heading_style("Heading3", "heading 3", SZ_H3, 120, 40, 2)
           + "</w:styles>")

CONTENT_TYPES = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                 '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                 '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                 '<Default Extension="xml" ContentType="application/xml"/>'
                 '<Default Extension="jpg" ContentType="image/jpeg"/>'
                 '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
                 '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
                 '<Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>'
                 '<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
                 '<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
                 '</Types>')

RELS = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>'
        '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>'
        '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>'
        '</Relationships>')

CORE = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" '
        'xmlns:dcmitype="http://purl.org/dc/dcmitype/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
        '<dc:title>Hnnu 队伍介绍</dc:title><dc:creator>Hnnu 队</dc:creator>'
        '<cp:lastModifiedBy>Hnnu 队</cp:lastModifiedBy></cp:coreProperties>')

APP = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
       '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
       'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
       '<Application>Hnnu</Application></Properties>')

SECTPR = ('<w:sectPr><w:pgSz w:w="11906" w:h="16838"/>'
          '<w:pgMar w:top="1440" w:right="1701" w:bottom="1440" w:left="1701" '
          'w:header="851" w:footer="992" w:gutter="0"/>'
          '<w:cols w:space="425"/><w:docGrid w:type="lines" w:linePitch="312"/>'
          '<w:footerReference w:type="default" r:id="rId9"/></w:sectPr>')

FOOTER = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
          '<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
          'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
          '<w:p><w:pPr><w:jc w:val="center"/>'
          '<w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/></w:pPr>'
          '<w:r><w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/>'
          '<w:sz w:val="21"/><w:szCs w:val="21"/></w:rPr><w:t xml:space="preserve">- </w:t></w:r>'
          '<w:r><w:fldChar w:fldCharType="begin"/></w:r>'
          '<w:r><w:instrText xml:space="preserve"> PAGE </w:instrText></w:r>'
          '<w:r><w:fldChar w:fldCharType="separate"/></w:r>'
          '<w:r><w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/><w:sz w:val="21"/></w:rPr>'
          '<w:t>1</w:t></w:r>'
          '<w:r><w:fldChar w:fldCharType="end"/></w:r>'
          '<w:r><w:rPr><w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/>'
          '<w:sz w:val="21"/><w:szCs w:val="21"/></w:rPr><w:t xml:space="preserve"> -</w:t></w:r>'
          '</w:p></w:ftr>' % (LATIN, LATIN, SONG, LATIN, LATIN, SONG, LATIN, LATIN, SONG))

FOOTER_REL = ('<Relationship Id="rId9" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" '
              'Target="footer1.xml"/>')


def build():
    body = document_body()
    # 图片关系 id 从 rId10 起
    pics = []
    rel_pics = []
    for i, (fname, px_w, px_h) in enumerate(PHOTO_ORDER):
        rid = "rId%d" % (10 + i)
        pics.append((fname, rid))
    xmlbody = body
    for i, (fname, rid) in enumerate(pics):
        xmlbody = xmlbody.replace('r:embed="rId%s"' % "%s", 'r:embed="%s"' % rid, 1)
        rel_pics.append('<Relationship Id="%s" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/%s"/>'
                        % (rid, fname))

    document = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
                'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
                'xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" '
                'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
                'xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">'
                '<w:body>' + xmlbody + SECTPR + '</w:body></w:document>')

    doc_rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
                + FOOTER_REL + "".join(rel_pics) + '</Relationships>')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", CONTENT_TYPES)
        z.writestr("_rels/.rels", RELS)
        z.writestr("docProps/core.xml", CORE)
        z.writestr("docProps/app.xml", APP)
        z.writestr("word/document.xml", document)
        z.writestr("word/styles.xml", STYLES)
        z.writestr("word/footer1.xml", FOOTER)
        z.writestr("word/_rels/document.xml.rels", doc_rels)
        for fname, _ in pics:
            z.write(os.path.join(MEDIA_TMP, fname), "word/media/" + fname)
    print("OK ->", OUT)
    print("photos:", [p[0] for p in pics])
    print("document.xml bytes:", len(document.encode("utf-8")))


if __name__ == "__main__":
    build()
