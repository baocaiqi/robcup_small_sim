#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
update_ua_graph.py — 一键更新 .ua/knowledge-graph.json 到当前代码（确定性重建）

思路（与团队上一轮 build_ua_batches.py 一致）：
  * 结构（文件/函数/行号/imports/calls）由本脚本对当前源码确定性解析；
  * 已存在节点（未改名的函数/文件/文档）直接复用旧图谱中的人工/LLM 摘要；
  * 新增实体生成自动摘要（可后续用 LLM 润色）；
  * 分层与 Tour 尽量沿用旧图谱，仅过滤失效节点并补充新文件归属。

用法：python tools/ua_dashboard/update_ua_graph.py
产物：.ua/knowledge-graph.json（旧文件先备份为 .ua/knowledge-graph.json.bak-<日期>）、.ua/meta.json
"""
import json, os, re, sys, subprocess, datetime, shutil

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UA = os.path.join(ROOT, '.ua')
KG_PATH = os.path.join(UA, 'knowledge-graph.json')
META_PATH = os.path.join(UA, 'meta.json')

ALLOW_EXT = {'.cpp', '.hpp', '.h', '.py', '.ps1', '.md', '.json'}
CTRL_KW = {'if', 'for', 'while', 'switch', 'catch', 'do', 'else', 'return',
           'sizeof', 'alignof', 'decltype', 'noexcept', 'typeid', 'static_assert',
           'throw', 'new', 'delete', 'case', 'operator'}

# ---------------- 工具 ----------------
def git_ls_files():
    try:
        out = subprocess.run(['git', 'ls-files'], cwd=ROOT, capture_output=True, text=True, encoding='utf-8')
        if out.returncode == 0:
            return [l for l in out.stdout.splitlines() if l.strip()]
    except Exception:
        pass
    # 兜底：遍历工作区（按照 .understandignore 默认规则）
    res = []
    for dp, dns, fns in os.walk(ROOT):
        dns[:] = [d for d in dns if d not in ('.git', 'build', '.ua', 'node_modules', 'dist', 'obj', '.vs')]
        for fn in fns:
            res.append(os.path.relpath(os.path.join(dp, fn), ROOT).replace('\\', '/'))
    return res

def allowed(rel):
    if rel.startswith('.ua/') or rel.startswith('build/') or rel.startswith('.git/'):
        return False
    if os.path.basename(rel) == 'CMakeLists.txt':
        return True
    ext = os.path.splitext(rel)[1].lower()
    if ext not in ALLOW_EXT:
        return False
    if ext == '.json':
        base = os.path.basename(rel)
        if re.match(r'^sim_eval_.*\.json$', base) or base == 'SimuroSot5.log':
            return False
    return True

def read_text(rel):
    """按 BOM / NUL 密度自动解码（roles.cpp 等为 UTF-16LE）"""
    raw = open(os.path.join(ROOT, rel), 'rb').read()
    if raw.startswith(b'\xff\xfe'):
        return raw[2:].decode('utf-16-le', errors='replace')
    if raw.startswith(b'\xfe\xff'):
        return raw[2:].decode('utf-16-be', errors='replace')
    if raw.count(b'\x00') > len(raw) * 0.05:
        return raw.decode('utf-16-le', errors='replace')
    for enc in ('utf-8-sig', 'gb18030'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode('utf-8', errors='replace')

def strip_comments(text):
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c == '/' and i + 1 < n and text[i + 1] == '/':
            j = text.find('\n', i)
            i = n if j == -1 else j
        elif c == '/' and i + 1 < n and text[i + 1] == '*':
            j = text.find('*/', i + 2)
            i = n if j == -1 else j + 2
            out.append(' ')
        else:
            out.append(c); i += 1
    return ''.join(out)

def first_comment(text):
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('//') or s.startswith('#'):
            s2 = s.lstrip('/#').strip()
            if s2:
                return s2
    return ''

def complexity_for(lines):
    return 'simple' if lines <= 40 else ('moderate' if lines <= 160 else 'complex')

# ---------------- C++ 函数解析 ----------------
def parse_cpp(text):
    """返回 [{name, start, end, body}]（1-based 行号）。comments 已剥离。"""
    text = strip_comments(text).replace('\r\n', '\n').replace('\r', '\n')
    lines = text.split('\n')
    funcs, n = [], len(lines)
    i = 0
    while i < n:
        s = lines[i].lstrip()
        if not s or not (s[0].isalpha() or s[0] == '_'):
            i += 1; continue
        first = re.match(r'^([A-Za-z_]\w*)', s)
        if not first or first.group(1) in CTRL_KW or first.group(1) in (
                'template', 'namespace', 'class', 'struct', 'enum', 'using', 'typedef', 'extern'):
            i += 1; continue
        # 拼接签名直到括号闭合
        buf, start, depth, j, closed_at = s, i + 1, 0, i, None
        while j < n:
            chunk = lines[j]
            if j > i:
                buf += '\n' + chunk
            for ch in chunk:
                if ch == '(':
                    depth += 1
                elif ch == ')':
                    depth -= 1
                    if depth == 0:
                        closed_at = j
                        break
            if closed_at is not None:
                break
            j += 1
            if j >= n:
                break
        if closed_at is None or closed_at >= n:
            i += 1; continue
        # 找函数体 '{'（'）' 后 4 行内）
        brace_row = None
        for k in range(closed_at, min(closed_at + 4, n)):
            if '{' in lines[k]:
                brace_row = k
                break
        if brace_row is None:
            i = closed_at + 1; continue
        # 函数名 = 首个 '(' 前的最后一个标识符
        sig = buf[:buf.find('(')]
        nm = re.findall(r'([A-Za-z_]\w*)\s*$', sig)
        if not nm:
            i = closed_at + 1; continue
        name = nm[-1]
        prefix = sig[:sig.rfind(name)]
        if re.search(r'[=,(.\->]\s*$', prefix) or '~' in prefix or name in CTRL_KW:
            i = closed_at + 1; continue
        # 匹配 '}' 闭合
        depth_b, end_row = 0, brace_row
        for k in range(brace_row, n):
            depth_b += lines[k].count('{') - lines[k].count('}')
            if depth_b == 0 and k >= brace_row and ('}' in lines[k] or k > brace_row):
                end_row = k
                break
        body_lines = lines[brace_row:end_row + 1]
        if not body_lines or body_lines[0].strip() in ('{}', '{ }'):
            i = max(end_row, closed_at) + 1; continue
        funcs.append({'name': name, 'start': start, 'end': end_row + 1, 'body': '\n'.join(body_lines)})
        i = end_row + 1
    seen, out = set(), []
    for f in funcs:
        if f['name'] not in seen:
            seen.add(f['name'])
            out.append(f)
    return out

def parse_py(text):
    lines = text.replace('\r\n', '\n').replace('\r', '\n').split('\n')
    funcs, n = [], len(lines)
    i = 0
    while i < n:
        m = re.match(r'^def\s+([A-Za-z_]\w*)\s*\(', lines[i])
        if m:
            name, start = m.group(1), i + 1
            end = n
            for k in range(i + 1, n):
                if re.match(r'^(def|class)\s', lines[k]):
                    end = k
                    break
            funcs.append({'name': name, 'start': start, 'end': end, 'body': '\n'.join(lines[i:end])})
            i = end
        else:
            i += 1
    return funcs

# ---------------- 主体 ----------------
def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    old = json.load(open(KG_PATH, encoding='utf-8')) if os.path.exists(KG_PATH) else None
    old_nodes = {n['id']: n for n in (old['nodes'] if old else [])}
    node_ids = set(old_nodes)

    # 1) 文件清单：git 跟踪 + 允许类型，并与旧图谱中仍存在的路径取并集
    scan = set(git_ls_files())
    for n in old_nodes.values():
        fp = n.get('filePath')
        if fp and os.path.exists(os.path.join(ROOT, fp)):
            scan.add(fp)
    scan = sorted(f for f in scan if f and allowed(f))

    # 2) 分类
    def category(rel):
        if os.path.basename(rel) == 'CMakeLists.txt':
            return 'config'
        ext = os.path.splitext(rel)[1].lower()
        if ext == '.md':
            return 'document'
        if ext == '.json':
            return 'document' if rel.startswith('docs/') else 'config'
        return 'file'

    py_mods = {os.path.splitext(os.path.basename(r))[0]: r
               for r in scan if r.startswith('tools/py/') and r.endswith('.py')}

    new_nodes = []
    node_ids = set()
    def add_node(n):
        if n['id'] in node_ids:
            return
        node_ids.add(n['id'])
        if n.get('languageNotes') is None:
            n.pop('languageNotes', None)
        new_nodes.append(n)

    new_edges = []
    files_meta = {}   # rel -> {text, funcs}（text 为原始文本，funcs 为解析结果）

    for rel in scan:
        cat = category(rel)
        if cat == 'document':
            oid = 'document:' + rel
            add_node(dict(old_nodes[oid]) if oid in old_nodes else {
                'id': oid, 'type': 'document', 'name': os.path.basename(rel), 'filePath': rel,
                'summary': '项目文档：' + rel + '。', 'tags': ['docs'], 'complexity': 'simple'})
            continue
        oid = ('config:' if cat == 'config' else 'file:') + rel
        if cat != 'file':
            add_node(dict(old_nodes[oid]) if oid in old_nodes else {
                'id': oid, 'type': 'config', 'name': os.path.basename(rel), 'filePath': rel,
                'summary': '配置/数据文件：' + rel + '。', 'tags': ['config', 'data'], 'complexity': 'simple'})
            continue
        if not os.path.exists(os.path.join(ROOT, rel)):
            if oid in old_nodes:
                add_node(dict(old_nodes[oid]))
            continue
        text = read_text(rel)
        funcs = []
        if rel.endswith('.py'):
            # python：剥掉整行 # 注释与空行，避免注释里的函数名产生假调用
            text_nc = '\n'.join(l for l in text.replace('\r\n', '\n').split('\n') if not l.lstrip().startswith('#'))
            funcs = parse_py(text_nc)
        elif rel.endswith('.cpp'):
            # 与旧图谱语义一致：仅 .cpp/.py 提取函数节点（.hpp 多为声明/内联，不展开）
            funcs = parse_cpp(text)
        files_meta[rel] = {'text': text, 'funcs': funcs}
        # 文件节点
        if oid in old_nodes:
            fn = dict(old_nodes[oid])
        else:
            fn = {'id': oid, 'type': 'file', 'name': os.path.basename(rel), 'filePath': rel,
                  'summary': (first_comment(text) or '项目文件：' + rel + '。')[:120],
                  'tags': ['script'] if rel.endswith('.ps1') else ['file'],
                  'complexity': complexity_for(len(text.splitlines()))}
            if rel.endswith('.py'):
                fn['languageNotes'] = '语言：python'
        add_node(fn)
        # 函数节点 + contains 边
        for f in funcs:
            fid = 'function:' + rel + ':' + f['name']
            if fid in old_nodes:
                on = dict(old_nodes[fid])
                on['lineRange'] = [f['start'], f['end']]
                add_node(on)
            else:
                summ = first_comment(f['body']) or (f['name'] + '：' + os.path.basename(rel) + ' 中的函数（自动更新新增）。')
                add_node({'id': fid, 'type': 'function', 'name': f['name'], 'filePath': rel,
                          'lineRange': [f['start'], f['end']], 'summary': summ[:120],
                          'tags': ['function'], 'complexity': complexity_for(f['end'] - f['start']),
                          'languageNotes': '语言：python' if rel.endswith('.py') else None})
            new_edges.append({'source': oid, 'target': fid, 'type': 'contains',
                              'direction': 'forward', 'weight': 1.0})

    # 3) imports 边（C++ #include + python import）
    inc_re = re.compile(r'#\s*include\s*[<"]([^>"]+)[>"]')
    for rel, meta in files_meta.items():
        src = 'file:' + rel
        if rel.endswith(('.cpp', '.hpp', '.h')):
            for m in inc_re.finditer(meta['text']):
                inc = m.group(1)
                target = None
                if inc.startswith('simuro5/'):
                    target = 'file:include/simuro5/' + inc.split('simuro5/', 1)[1]
                elif inc.endswith(('.hpp', '.h')):
                    cand = 'file:include/simuro5/' + os.path.basename(inc)
                    if cand in node_ids:
                        target = cand
                if target and target in node_ids and target != src:
                    new_edges.append({'source': src, 'target': target, 'type': 'imports',
                                      'direction': 'forward', 'weight': 0.7})
        elif rel.endswith('.py'):
            for m in re.finditer(r'^\s*(?:import|from)\s+([\w.]+)', meta['text'], re.M):
                mod = m.group(1).split('.')[0]
                if mod in py_mods and py_mods[mod] != rel:
                    tgt = 'file:' + py_mods[mod]
                    if tgt in node_ids:
                        new_edges.append({'source': src, 'target': tgt, 'type': 'imports',
                                          'direction': 'forward', 'weight': 0.7})

    # 4) calls 边：源函数体内出现目标函数名（词边界；同文件内 + 跨文件）
    for rel, meta in files_meta.items():
        for f in meta['funcs']:
            fid = 'function:' + rel + ':' + f['name']
            # 同文件内调用
            for g in meta['funcs']:
                if g['name'] != f['name'] and re.search(r'\b' + re.escape(g['name']) + r'\b', f['body']):
                    new_edges.append({'source': fid, 'target': 'function:' + rel + ':' + g['name'],
                                      'type': 'calls', 'direction': 'forward', 'weight': 0.8})
            # 跨文件调用
            for orel, ometa in files_meta.items():
                if orel == rel:
                    continue
                for g in ometa['funcs']:
                    if g['name'] == f['name']:
                        continue
                    if re.search(r'\b' + re.escape(g['name']) + r'\b', f['body']):
                        new_edges.append({'source': fid, 'target': 'function:' + orel + ':' + g['name'],
                                          'type': 'calls', 'direction': 'forward', 'weight': 0.8})

    # 5) related 边（保旧）
    if old:
        for e in old['edges']:
            if e['type'] == 'related' and e['source'] in node_ids and e['target'] in node_ids:
                new_edges.append(e)

    # 6) 边去重
    dedup = {}
    for e in new_edges:
        dedup.setdefault((e['source'], e['target'], e['type']), e)
    edges = list(dedup.values())

    # 7) 分层：保旧 + 新文件归属
    # 7) 分层：按路径规则从零重建（可复现、可自愈；与原始人工分层一致）
    #     规则：dll/simuro_interface→平台层；world_model/team→世界模型层；
    #     strategy/role_assignment/situation/formation→策略调度层；
    #     tools/py→Python 层；offline_test/sim_bench/run_sim→测试层；
    #     技能文档→团队技能层；其余代码→行为控制层；文档→文档层；CMakeLists→构建层
    def layer_of(n):
        t, rel = n['type'], n['id'].split(':', 1)[1]
        base = os.path.basename(rel)
        if t == 'document':
            return 'layer:team-skills' if rel.startswith(('.claude/skills/', '.codex/skills/')) else 'layer:documentation'
        if t == 'config':
            return 'layer:config-build' if rel == 'CMakeLists.txt' else 'layer:python-tools'
        if rel.startswith('tools/py/'):
            return 'layer:python-tools'
        if base in ('offline_test.cpp', 'sim_bench.cpp', 'run_sim.ps1'):
            return 'layer:testing-tools'
        if base in ('dll_blue.cpp', 'dll_yellow.cpp', 'simuro_interface.hpp'):
            return 'layer:platform-interface'
        if base in ('world_model.cpp', 'world_model.hpp', 'team.hpp'):
            return 'layer:world-model'
        if base in ('strategy.cpp', 'strategy.hpp', 'role_assignment.cpp', 'role_assignment.hpp',
                    'situation.cpp', 'situation.hpp', 'formation.cpp', 'formation.hpp'):
            return 'layer:strategy-scheduler'
        return 'layer:behavior-control'
    layer_order = ['layer:platform-interface', 'layer:world-model', 'layer:strategy-scheduler',
                   'layer:behavior-control', 'layer:testing-tools', 'layer:python-tools',
                   'layer:documentation', 'layer:team-skills', 'layer:config-build']
    old_layer_info = {}
    if old:
        for l in old['layers']:
            old_layer_info[l['id']] = l
    layers = []
    for lid in layer_order:
        ids = [n['id'] for n in new_nodes
               if n['type'] in ('file', 'config', 'document') and layer_of(n) == lid]
        if not ids:
            continue
        info = old_layer_info.get(lid, {})
        layers.append({'id': lid, 'name': info.get('name', lid),
                       'description': info.get('description', ''),
                       'nodeIds': ids})

    # 8) Tour：保旧 + 过滤
    tour = []
    if old:
        for s in old['tour']:
            ids = [i for i in s.get('nodeIds', []) if i in node_ids]
            if ids:
                tour.append({'order': s['order'], 'title': s['title'],
                             'description': s.get('description', ''), 'nodeIds': ids})

    # 9) project / meta
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                          capture_output=True, text=True).stdout.strip()
    now = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    langs = sorted({os.path.splitext(n.get('filePath', ''))[1].lstrip('.').lower() or 'markdown'
                    for n in new_nodes if n.get('filePath')})
    project = {
        'name': 'strategy_5v5',
        'languages': langs,
        'frameworks': [],
        'description': 'FIRA 仿真 5v5 策略库（SimuroSot5）：动态角色分配 / 传球链 / 防守体系 / 运动控制；C++ DLL + Python 复盘工具。',
        'analyzedAt': now,
        'gitCommitHash': head,
    }
    graph = {'version': '1.0.0', 'project': project,
             'nodes': new_nodes, 'edges': edges, 'layers': layers, 'tour': tour}

    # 10) 校验 + 备份 + 写盘
    bad = [e for e in edges if e['source'] not in node_ids or e['target'] not in node_ids]
    if bad:
        print('⚠ 校验失败：%d 条悬空边，中止写盘' % len(bad))
        for e in bad[:10]:
            print('  ', e)
        sys.exit(1)
    bak = KG_PATH + '.bak-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    if not os.path.exists(bak):
        shutil.copy2(KG_PATH, bak)
    with open(KG_PATH, 'w', encoding='utf-8') as f:
        json.dump(graph, f, ensure_ascii=False, indent=1)
    json.dump({'lastAnalyzedAt': now, 'gitCommitHash': head, 'version': '1.0.0',
               'analyzedFiles': len(new_nodes)}, open(META_PATH, 'w', encoding='utf-8'),
              ensure_ascii=False, indent=4)

    # 11) 摘要输出
    def cnt(t):
        return sum(1 for n in new_nodes if n['type'] == t)
    def ecnt(t):
        return sum(1 for e in edges if e['type'] == t)
    print('✅ 已更新 ' + KG_PATH)
    print('   节点 %d（function %d / file %d / document %d / config %d）' %
          (len(new_nodes), cnt('function'), cnt('file'), cnt('document'), cnt('config')))
    print('   边 %d（imports %d / calls %d / contains %d / related %d）' %
          (len(edges), ecnt('imports'), ecnt('calls'), ecnt('contains'), ecnt('related')))
    print('   分层 %d / tour %d / commit %s' % (len(layers), len(tour), head[:12]))
    if old:
        old_ids = set(old_nodes)
        added = sorted(node_ids - old_ids)
        removed = sorted(old_ids - node_ids)
        print('   新增节点 %d：%s' % (len(added), ', '.join(added[:15]) + ('…' if len(added) > 15 else '')))
        print('   移除节点 %d：%s' % (len(removed), ', '.join(removed[:15]) + ('…' if len(removed) > 15 else '')))
    print('   备份：' + bak)

if __name__ == '__main__':
    main()