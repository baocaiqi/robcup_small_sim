#!/usr/bin/env node
/**
 * build_ua_dashboard.mjs
 * 从 .ua/knowledge-graph.json 生成自包含的交互式 HTML 代码导图
 * 输出：docs/work/ua-code-map.html（零外部依赖，浏览器直接打开）
 *
 * 用法：node tools/ua_dashboard/build_ua_dashboard.mjs
 */
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const KG_PATH = join(ROOT, '.ua', 'knowledge-graph.json');
const OUT_PATH = join(ROOT, 'docs', 'work', 'ua-code-map.html');

if (!existsSync(KG_PATH)) {
  console.error('未找到 knowledge-graph.json："' + KG_PATH + '"');
  process.exit(1);
}

const graph = JSON.parse(readFileSync(KG_PATH, 'utf8'));
const { version, project, nodes, edges, layers, tour } = graph;

// ---------- 索引 ----------
const byId = new Map(nodes.map((n) => [n.id, n]));
const layerOf = new Map(); // nodeId -> layerId（取第一个出现的层）
for (const l of layers) for (const nid of l.nodeIds) if (!layerOf.has(nid)) layerOf.set(nid, l.id);
const layerById = new Map(layers.map((l) => [l.id, l]));

/** 文件级边：imports(file→file) + 函数级 calls 聚合(file→file) + related 聚合 */
const fileEdges = [];
const seen = new Set();
const aggKey = (a, b, t) => a + '\u0000' + b + '\u0000' + t;
const addFileEdge = (srcFile, tgtFile, type) => {
  if (!srcFile || !tgtFile || srcFile === tgtFile) return;
  const k = aggKey(srcFile, tgtFile, type);
  if (seen.has(k)) return;
  seen.add(k);
  const sn = byId.get(srcFile), tn = byId.get(tgtFile);
  fileEdges.push({
    source: srcFile, target: tgtFile, type,
    weight: type === 'imports' ? 0.7 : 0.8,
  });
  void sn; void tn;
};
for (const e of edges) {
  const s = byId.get(e.source), t = byId.get(e.target);
  if (!s || !t) continue;
  if (e.type === 'imports' && s.type === 'file' && t.type === 'file') {
    addFileEdge(e.source, e.target, 'imports');
  } else if (e.type === 'calls') {
    const sf = s.type === 'function' ? s.filePath : (s.type === 'file' ? s.filePath : null);
    const tf = t.type === 'function' ? t.filePath : (t.type === 'file' ? t.filePath : null);
    if (sf && tf && sf !== tf) addFileEdge('file:' + sf, 'file:' + tf, 'calls');
  } else if (e.type === 'related') {
    const sf = s.filePath || (s.type === 'file' ? s.filePath : null);
    const tf = t.filePath || (t.type === 'file' ? t.filePath : null);
    if (sf && tf && sf !== tf) addFileEdge('file:' + sf, 'file:' + tf, 'related');
  }
}

const data = { version, project, nodes, edges, layers, tour, fileEdges };

const css = `
:root{
  --bg:#0b1220; --panel:#101a2e; --panel2:#0e1a30; --line:#1e2a44; --txt:#dbe4f5; --dim:#5b6b8c;
  --blue:#38bdf8; --orange:#fb923c; --green:#34d399; --purple:#a78bfa; --pink:#f472b6; --gray:#94a3b8;
}
*{box-sizing:border-box}
body{margin:0;font-family:"Segoe UI","Microsoft YaHei",system-ui,sans-serif;background:var(--bg);color:var(--txt);font-size:13px}
header{position:sticky;top:0;z-index:50;background:rgba(11,18,32,.95);border-bottom:1px solid var(--line);backdrop-filter:blur(6px)}
.hd-in{max-width:1600px;margin:0 auto;padding:10px 18px;display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.hd-in h1{font-size:16px;margin:0;font-weight:700;letter-spacing:.5px}
.hd-in .repo{color:var(--dim);font-weight:400;font-size:12px}
.chips{display:flex;gap:6px;flex-wrap:wrap}
.chip{font-size:11px;padding:2px 9px;border-radius:99px;border:1px solid var(--line);color:var(--dim);background:#0d1526;white-space:nowrap}
.chip b{color:var(--txt);font-weight:600}
nav{max-width:1600px;margin:0 auto;display:flex;gap:4px;padding:0 18px}
nav button{background:none;border:1px solid transparent;color:var(--dim);padding:8px 14px;font-size:13px;cursor:pointer;border-radius:8px 8px 0 0}
nav button:hover{color:var(--txt);background:#0e1a30}
nav button.on{color:var(--txt);background:var(--panel);border-color:var(--line);border-bottom-color:var(--panel);font-weight:600}
main{max-width:1600px;margin:0 auto;padding:14px 18px 60px}
.panel{display:none}
.panel.on{display:block}
.panel h2{font-size:14px;margin:0 0 4px}
.panel .sub{color:var(--dim);font-size:12px;margin:0 0 12px}
.legend{display:flex;gap:14px;font-size:12px;color:var(--dim);margin:8px 0}
.legend i{display:inline-block;width:26px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
#layerMapSvg{width:100%;height:auto;background:var(--panel);border:1px solid var(--line);border-radius:10px;display:block}
.filebox{cursor:pointer}
.filebox rect{fill:#12203c;stroke:#2b3d63;stroke-width:1;rx:6}
.filebox text{fill:var(--txt);font-size:11px}
.layerband rect.lb{fill:#0c1626;stroke:var(--line);stroke-width:1;rx:10}
.layerband text.lt{fill:var(--purple);font-size:13px;font-weight:700}
.layerband text.ld{fill:var(--dim);font-size:10.5px}
.chipbox rect{fill:#152138;stroke:#2b3d63;stroke-width:1;rx:6;stroke-dasharray:3 3}
.chipbox text{fill:var(--dim);font-size:10.5px}
.flow{fill:none;stroke-width:1.4}
.flow.imports{stroke:var(--blue)}
.flow.calls{stroke:var(--orange)}
.flow.related{stroke:var(--gray);stroke-dasharray:4 3}
.node.dim rect{opacity:.18}
.node.dim text{opacity:.25}
.flow.dim{opacity:.06}
.flow.hl{stroke-width:2.6;filter:drop-shadow(0 0 5px rgba(56,189,248,.5))}
.flow.hl.imports{filter:drop-shadow(0 0 5px rgba(56,189,248,.7))}
.flow.hl.calls{filter:drop-shadow(0 0 5px rgba(251,146,60,.7))}
table{width:100%;border-collapse:collapse;font-size:12px;background:var(--panel);border-radius:8px;overflow:hidden}
th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.4px}
td code{color:var(--blue);font-size:11px}
.tag{display:inline-block;font-size:10px;padding:1px 7px;border-radius:99px;border:1px solid var(--line);color:var(--dim);margin-right:4px}
.badge{display:inline-block;font-size:10px;padding:1px 8px;border-radius:99px;font-weight:600}
.badge.file{background:#0e2a3e;color:var(--blue);border:1px solid #1d4b6e}
.badge.function{background:#3d2308;color:var(--orange);border:1px solid #7c4a12}
.badge.document{background:#213010;color:var(--green);border:1px solid #3f5c1f}
.badge.config{background:#2a1f40;color:var(--purple);border:1px solid #4c3a70}
.row{display:grid;grid-template-columns:240px 1fr;gap:14px;align-items:start}
.row>aside{position:sticky;top:86px}
#fileList{list-style:none;margin:0;padding:0;background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden}
#fileList li{display:flex;align-items:center;gap:8px;padding:7px 10px;border-bottom:1px solid var(--line);cursor:pointer;font-size:12px}
#fileList li:last-child{border-bottom:none}
#fileList li:hover{background:#13203a}
#fileList li.on{background:#16304f}
#fileList .ly{font-size:10px;color:var(--dim);width:88px;flex:none}
#fileList .fn{font-family:Consolas,monospace;font-size:11.5px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#funcSvg{width:100%;height:auto;background:var(--panel);border:1px solid var(--line);border-radius:10px;display:block}
.fnode rect{fill:#12203c;stroke:#2b3d63;rx:6}
.fnode text{font-size:11px;fill:var(--txt)}
.fnode.src rect{fill:#1b2c4e;stroke:var(--blue)}
.fnode.tgt rect{fill:#33220a;stroke:#7c4a12}
.fcall{fill:none;stroke:var(--orange);stroke-width:1.3}
.fimp{fill:none;stroke:var(--blue);stroke-width:1.2;stroke-dasharray:5 3}
.tourcard{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin-bottom:10px}
.tourcard .no{display:inline-flex;width:24px;height:24px;border-radius:50%;background:#1d4b6e;color:var(--blue);align-items:center;justify-content:center;font-weight:700;font-size:12px;margin-right:8px}
.tourcard h3{display:inline;font-size:13px;margin:0}
.tourcard p{color:var(--dim);font-size:12px;margin:8px 0 0;line-height:1.6}
.nodelink{display:inline-block;font-size:11px;padding:2px 8px;border-radius:99px;border:1px solid var(--line);color:var(--dim);margin:3px 3px 0 0;cursor:pointer}
.nodelink:hover{color:var(--txt);border-color:var(--blue)}
.nodelink.function{color:var(--orange);border-color:#7c4a12}
.nodelink.document{color:var(--green);border-color:#3f5c1f}
.searchbox{width:100%;max-width:560px;padding:9px 14px;border-radius:8px;border:1px solid var(--line);background:var(--panel);color:var(--txt);font-size:13px;outline:none}
.searchbox:focus{border-color:var(--blue)}
#searchResults tr{cursor:pointer}
#searchResults tr:hover{background:#13203a}
.drawer{position:fixed;top:0;right:-420px;width:400px;height:100%;background:#0e1a30;border-left:1px solid var(--line);z-index:90;transition:right .25s ease;overflow-y:auto;padding:16px}
.drawer.open{right:0}
.drawer .close{position:absolute;top:10px;right:12px;background:none;border:none;color:var(--dim);font-size:18px;cursor:pointer}
.drawer h3{font-size:13px;margin:0 0 4px;word-break:break-all}
.drawer .dsub{color:var(--dim);font-size:11px;margin-bottom:10px;word-break:break-all}
.drawer .dsec{font-size:11px;color:var(--dim);text-transform:uppercase;letter-spacing:.5px;margin:14px 0 4px}
.drawer p.sum{font-size:12px;line-height:1.6;color:var(--txt)}
.drawer ul{list-style:none;margin:0;padding:0}
.drawer li{font-size:11.5px;padding:3px 0;border-bottom:1px dashed var(--line)}
.drawer li span{color:var(--dim)}
.stale{color:#fbbf24}
a{color:var(--blue);text-decoration:none}
`;

const html = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>strategy_5v5 · ua 知识图谱代码导图</title>
<style>${css}</style>
</head>
<body>
<header>
  <div class="hd-in">
    <h1>🧭 strategy_5v5 <span class="repo">· ua(Understand-Anything) 知识图谱代码导图</span></h1>
    <div class="chips" id="statChips"></div>
  </div>
  <nav>
    <button data-tab="layer" class="on">🏗 分层导图</button>
    <button data-tab="filemap">📁 文件依赖</button>
    <button data-tab="func">⚙ 函数调用</button>
    <button data-tab="tour">🗺 Tour 导览</button>
    <button data-tab="search">🔍 检索</button>
  </nav>
</header>
<main>
  <section class="panel on" id="tab-layer">
    <h2>分层架构导图</h2>
    <p class="sub">自上而下按 ua 分层展示源码文件依赖；悬停节点高亮关联边，点击固定。导入边=蓝，调用边=橙，关联=灰虚线。</p>
    <div class="legend">
      <span><i style="background:var(--blue)"></i>imports 导入</span>
      <span><i style="background:var(--orange)"></i>calls 跨文件调用（函数级聚合）</span>
      <span><i style="background:var(--gray)"></i>related 关联</span>
    </div>
    <svg id="layerMapSvg"></svg>
  </section>

  <section class="panel" id="tab-filemap">
    <h2>文件依赖清单</h2>
    <p class="sub">文件级 imports / calls / related 边列表。搜索框过滤节点，行首徽标=文件类型。</p>
    <input class="searchbox" id="fileFilter" placeholder="过滤文件依赖（输入文件名关键词…）">
    <div style="height:10px"></div>
    <table><thead><tr><th>来源文件</th><th>方向/类型</th><th>目标文件</th><th>说明</th></tr></thead>
    <tbody id="fileEdgeRows"></tbody></table>
  </section>

  <section class="panel" id="tab-func">
    <h2>函数调用图（按文件查看）</h2>
    <p class="sub">左侧选文件 → 右侧绘制其内部函数与被调函数（橙色实线=调用，蓝色虚线=该文件导入的头文件）。</p>
    <div class="row">
      <aside><ul id="fileList"></ul></aside>
      <div><svg id="funcSvg" width="1180" height="80"></svg></div>
    </div>
  </section>

  <section class="panel" id="tab-tour">
    <h2>Tour 导览</h2>
    <p class="sub">ua 生成的 13 步仓库导览，按顺序带你走一遍整个项目。</p>
    <div id="tourCards"></div>
  </section>

  <section class="panel" id="tab-search">
    <h2>图谱检索</h2>
    <p class="sub">检索所有 function / file / document / config 节点（按名称或 ID）。点击行查看详情。</p>
    <input class="searchbox" id="globalSearch" placeholder="搜索函数名 / 文件名 / 摘要关键词…">
    <div style="height:10px"></div>
    <table><thead><tr><th>节点</th><th>类型</th><th>位置</th><th>摘要</th><th>标签</th></tr></thead>
    <tbody id="searchResults"></tbody></table>
  </section>
</main>

<div class="drawer" id="drawer">
  <button class="close" onclick="closeDrawer()">✕</button>
  <div id="drawerBody"></div>
</div>

<script>
const DATA = ${JSON.stringify(data).replace(/</g, '\\u003c')};
const { nodes, edges, layers, tour, fileEdges, project } = DATA;
const byId = new Map(nodes.map(n => [n.id, n]));
const layerOf = new Map();
for (const l of layers) for (const nid of l.nodeIds) if (!layerOf.has(nid)) layerOf.set(nid, l.id);
const layerById = new Map(layers.map(l => [l.id, l]));
const esc = s => String(s || '').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
const shortName = id => { const n = byId.get(id); return n ? n.name : id.split('/').pop(); };
const layerName = lid => layerById.get(lid) ? layerById.get(lid).name : '—';

/* ---------- 统计 chips ---------- */
const cnt = t => nodes.filter(n => n.type === t).length;
document.getElementById('statChips').innerHTML = [
  ['分析文件', project.analyzedFiles || nodes.filter(n=>['file'].includes(n.type)).length],
  ['节点', nodes.length], ['边', edges.length],
  ['函数', cnt('function')], ['文件', cnt('file')],
  ['文档', cnt('document')], ['分层', layers.length], ['Tour', tour.length],
  ['图谱时间', (project.analyzedAt||'').slice(0,10)],
  ['commit', (project.gitCommitHash||'').slice(0,7)],
].map(([k,v]) => '<span class="chip"><b>'+k+'</b> · '+esc(v)+'</span>').join('');

/* ---------- tabs ---------- */
document.querySelectorAll('nav button').forEach(b => b.onclick = () => {
  document.querySelectorAll('nav button').forEach(x => x.classList.toggle('on', x === b));
  document.querySelectorAll('.panel').forEach(p => p.classList.toggle('on', p.id === 'tab-' + b.dataset.tab));
});

/* ---------- drawer ---------- */
function openNode(id) {
  const n = byId.get(id); if (!n) return;
  const outs = edges.filter(e => e.source === id);
  const ins = edges.filter(e => e.target === id);
  const edgeRow = (e, dir) => {
    const other = dir === 'out' ? e.target : e.source;
    const on = byId.get(other);
    return '<li>'+esc(dir==='out'?'→':'←')+' <span>'+esc(e.type)+'</span> '+
      '<a href="javascript:openNode(\\''+other+'\\')">'+esc(shortName(other))+'</a> <span>· w='+e.weight+'</span></li>';
  };
  document.getElementById('drawerBody').innerHTML =
    '<h3><span class="badge '+esc(n.type)+'">'+esc(n.type)+'</span> '+esc(n.name)+'</h3>' +
    '<div class="dsub">'+esc(n.id)+'</div>' +
    (n.filePath ? '<div class="dsub">📄 '+esc(n.filePath)+(n.lineRange ? ' · L'+n.lineRange[0]+'-'+n.lineRange[1] : '')+'</div>' : '') +
    '<div class="dsub">分层：'+esc(layerName(layerOf.get(id)))+'</div>' +
    (n.summary ? '<p class="sum">'+esc(n.summary)+'</p>' : '') +
    (n.tags && n.tags.length ? '<div>' + n.tags.map(t => '<span class="tag">'+esc(t)+'</span>').join('') + '</div>' : '') +
    '<div class="dsec">出边 '+outs.length+'</div><ul>' + outs.map(e => edgeRow(e,'out')).join('') + '</ul>' +
    '<div class="dsec">入边 '+ins.length+'</div><ul>' + ins.map(e => edgeRow(e,'in')).join('') + '</ul>';
  document.getElementById('drawer').classList.add('open');
}
function closeDrawer(){ document.getElementById('drawer').classList.remove('open'); }

/* ---------- 分层导图 ---------- */
(function buildLayerMap(){
  const W = 1500, PAD = 16, BANDH = 34, BOXW = 172, BOXH = 34, HGAP = 16, VGAP = 30, ROWH = 44;
  const NS = 'http://www.w3.org/2000/svg';
  const svg = document.getElementById('layerMapSvg');
  const layFiles = layers.map(l => ({ l, files: l.nodeIds.filter(id => byId.get(id) && byId.get(id).type === 'file'),
    configs: l.nodeIds.filter(id => byId.get(id) && byId.get(id).type === 'config'),
    docs: l.nodeIds.filter(id => byId.get(id) && byId.get(id).type === 'document') }));
  const pos = new Map(); // nodeId -> {x,y}
  let y = PAD, totalH = PAD;
  const bands = [];
  for (const { l, files, configs, docs } of layFiles) {
    const label = (layerById.get(l.id)?.name) || l.id;
    if (files.length === 0) {
      const parts = [];
      if (docs.length) parts.push(docs.length + ' 个文档节点');
      if (configs.length) parts.push(configs.length + ' 个配置节点');
      bands.push({ y, h: BANDH, label: label + '（' + parts.join('，') + '）', files: [], chips: [] });
      y += BANDH + VGAP; totalH = y;
      continue;
    }
    const rows = [];
    for (let i = 0; i < files.length; i += 6) rows.push(files.slice(i, i + 6));
    const bh = BANDH + rows.length * ROWH + 8;
    let bx = PAD;
    rows.forEach((row, ri) => {
      const ry = y + BANDH + 6 + ri * ROWH;
      bx = PAD;
      row.forEach(fid => {
        pos.set(fid, { x: bx, y: ry });
        bx += BOXW + HGAP;
      });
    });
    bands.push({ y, h: bh, label, files, chips: configs.map(c => ({ id: c, name: byId.get(c)?.name || c })) });
    y += bh + VGAP; totalH = y;
  }
  svg.setAttribute('viewBox', '0 0 ' + W + ' ' + (totalH + 10));
  svg.innerHTML = '<defs>' +
    '<marker id="ah-imports" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto"><path d="M0,0L9,4.5L0,9z" fill="#38bdf8"/></marker>' +
    '<marker id="ah-calls" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto"><path d="M0,0L9,4.5L0,9z" fill="#fb923c"/></marker>' +
    '<marker id="ah-related" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto"><path d="M0,0L9,4.5L0,9z" fill="#94a3b8"/></marker>' +
    '</defs>';
  // bands (先画，在下层)
  for (const b of bands) {
    const g = document.createElementNS(NS, 'g'); g.setAttribute('class', 'layerband');
    g.innerHTML = '<rect class="lb" x="'+PAD+'" y="'+b.y+'" width="'+(W-2*PAD)+'" height="'+b.h+'"/>' +
      '<text class="lt" x="'+(PAD+12)+'" y="'+(b.y+21)+'">'+esc(b.label)+'</text>' +
      (b.files.length ? '' : '<text class="ld" x="'+(PAD+12)+'" y="'+(b.y+22)+'" text-anchor="end">—</text>');
    svg.appendChild(g);
  }
  // 文件节点
  for (const [id, p] of pos) {
    const n = byId.get(id);
    const g = document.createElementNS(NS, 'g');
    g.setAttribute('class', 'node filebox');
    g.setAttribute('data-id', id);
    g.innerHTML = '<rect class="box" x="'+p.x+'" y="'+p.y+'" width="'+BOXW+'" height="'+BOXH+'"/>' +
      '<text x="'+(p.x+BOXW/2)+'" y="'+(p.y+BOXH/2+4)+'" text-anchor="middle">'+esc(n.name)+'</text>';
    g.onclick = () => openNode(id);
    svg.appendChild(g);
  }
  // 配置 chip（无文件层的层内）
  for (const b of bands) {
    let cx = W - PAD;
    for (const c of b.chips) {
      const cw = Math.max(120, 18 + esc(c.name).length * 6.6);
      cx -= cw + 6;
      const g = document.createElementNS(NS, 'g');
      g.setAttribute('class', 'chipbox'); g.setAttribute('data-id', c.id);
      g.innerHTML = '<rect x="'+cx+'" y="'+(b.y+8)+'" width="'+cw+'" height="'+(BANDH-16)+'"/>' +
        '<text x="'+(cx+cw/2)+'" y="'+(b.y+BANDH/2+1)+'" text-anchor="middle">'+esc(c.name)+'</text>';
      g.onclick = () => openNode(c.id);
      svg.appendChild(g);
    }
  }
  // 边
  const edgesG = document.createElementNS(NS, 'g'); edgesG.setAttribute('id', 'layerEdges');
  for (const e of fileEdges) {
    const sp = pos.get(e.source), tp = pos.get(e.target);
    if (!sp || !tp) continue;
    const x1 = sp.x + BOXW / 2, y1 = sp.y + BOXH, x2 = tp.x + BOXW / 2, y2 = tp.y;
    let d;
    if (y2 > y1 + 6) d = 'M'+x1+','+y1+' C'+x1+','+(y1+(y2-y1)/2)+' '+x2+','+(y1+(y2-y1)/2)+' '+x2+','+y2;
    else if (Math.abs(y2 - y1) <= 6) {
      const hop = x2 > x1 ? 34 : -34;
      d = 'M'+x1+','+y1+' C'+(x1+hop)+','+y1+' '+(x2-hop)+','+y2+' '+x2+','+y2;
    } else {
      const hop = x2 > x1 ? 46 : -46;
      d = 'M'+x1+','+y1+' C'+(x1+hop)+','+(y1+14)+' '+(x2-hop)+','+(y2-14)+' '+x2+','+y2;
    }
    const path = document.createElementNS(NS, 'path');
    path.setAttribute('d', d);
    path.setAttribute('class', 'flow ' + e.type);
    path.setAttribute('marker-end', 'url(#ah-' + e.type + ')');
    path.setAttribute('data-src', e.source); path.setAttribute('data-tgt', e.target);
    edgesG.appendChild(path);
  }
  svg.insertBefore(edgesG, svg.firstChild.nextSibling);

  // 悬停高亮
  const nodesG = Array.from(svg.querySelectorAll('.node'));
  const flows = Array.from(svg.querySelectorAll('.flow'));
  let pinned = null;
  const apply = (id) => {
    const active = id ? new Set([id]) : null;
    if (pinned && id === null) return;
    for (const n of nodesG) {
      const on = active ? active.has(n.dataset.id) : true;
      n.classList.toggle('dim', !on);
    }
    if (!active) { flows.forEach(f => f.classList.remove('hl','dim')); }
    else {
      flows.forEach(f => {
        const link = f.dataset.src === id || f.dataset.tgt === id;
        f.classList.toggle('hl', link);
        f.classList.toggle('dim', !link);
      });
    }
  };
  nodesG.forEach(n => {
    n.onmouseenter = () => { if (!pinned) apply(n.dataset.id); };
    n.onmouseleave = () => { if (!pinned) apply(null); };
    n.onclick = () => {
      pinned = pinned === n.dataset.id ? null : n.dataset.id;
      apply(pinned);
      openNode(n.dataset.id);
    };
  });
})();

/* ---------- 文件依赖清单 ---------- */
(function buildFileTable(){
  const tbody = document.getElementById('fileEdgeRows');
  const input = document.getElementById('fileFilter');
  const rows = fileEdges.map(e => ({
    e,
    s: byId.get(e.source), t: byId.get(e.target),
    key: (e.source + ' ' + e.target).toLowerCase()
  }));
  const render = (kw) => {
    const list = kw ? rows.filter(r => r.key.includes(kw)) : rows;
    tbody.innerHTML = list.map(r => {
      const at = r.e.type === 'imports' ? '导入' : (r.e.type === 'related' ? '关联' : '调用');
      return '<tr>' +
        '<td><a href="javascript:openNode(\\''+r.e.source+'\\')"><span class="badge file">F</span> '+esc(r.s.name)+'</a><br><span style="color:var(--dim);font-size:10px">'+esc(layerName(layerOf.get(r.e.source)))+'</span></td>' +
        '<td style="color:var(--'+(r.e.type==='imports'?'blue':r.e.type==='related'?'gray':'orange')+')">→ '+esc(at)+'</td>' +
        '<td><a href="javascript:openNode(\\''+r.e.target+'\\')"><span class="badge file">F</span> '+esc(r.t.name)+'</a><br><span style="color:var(--dim);font-size:10px">'+esc(layerName(layerOf.get(r.e.target)))+'</span></td>' +
        '<td style="color:var(--dim)">'+esc((r.t.summary || r.s.summary || '').slice(0, 60))+'</td>' +
      '</tr>';
    }).join('') || '<tr><td colspan="4" style="color:var(--dim)">无匹配</td></tr>';
  };
  render('');
  input.oninput = () => render(input.value.trim().toLowerCase());
})();

/* ---------- 函数调用图 ---------- */
(function buildFuncView(){
  const list = document.getElementById('fileList');
  const svg = document.getElementById('funcSvg');
  const files = nodes.filter(n => n.type === 'file')
    .sort((a, b) => {
      const la = layers.findIndex(l => l.nodeIds.includes(a.id)), lb = layers.findIndex(l => l.nodeIds.includes(b.id));
      return (la === -1 ? 99 : la) - (lb === -1 ? 99 : lb) || a.id.localeCompare(b.id);
    });
  const funcsOf = fid => nodes.filter(n => n.type === 'function' && n.filePath === (byId.get(fid)?.filePath));
  list.innerHTML = files.map(f => {
    const lname = layerName(layerOf.get(f.id));
    return '<li data-fid="'+f.id+'"><span class="ly">'+esc(lname)+'</span><span class="fn">'+esc(f.name)+'</span></li>';
  }).join('');
  let selected = null;
  const draw = (fid) => {
    const file = byId.get(fid);
    const funcs = funcsOf(fid).sort((a,b) => (a.lineRange?.[0]||0)-(b.lineRange?.[0]||0));
    const xs = 22, xw = 230, gapX = 46, w = 1180, h = 90;
    // 收集出边
    const callOuts = edges.filter(e => e.type === 'calls' && e.source.startsWith('function:') && funcs.some(f => f.id === e.source));
    // 边 → 目标文件分组
    const tgtFiles = [];
    const tgtOf = new Map();
    for (const e of callOuts) {
      const t = byId.get(e.target);
      if (!t) continue;
      if (t.type === 'function' && t.filePath) {
        const tfid = t.filePath.startsWith('include/') ? 'file:include/' + t.filePath : 'file:' + t.filePath;
        if (!tgtOf.has(tfid)) { tgtOf.set(tfid, []); tgtFiles.push(tfid); }
        tgtOf.get(tfid).push(e);
      }
    }
    tgtFiles.sort();
    const cols = []; // 每列 {fileId, funcs:[...], x}
    const perCol = 16;
    let cx = xs + xw + gapX;
    for (const tfid of tgtFiles) {
      const tfuncs = [...new Set(tgtOf.get(tfid).map(e => e.target))].map(id => byId.get(id))
        .sort((a,b) => (a.lineRange?.[0]||0)-(b.lineRange?.[0]||0));
      cols.push({ fileId: tfid, funcs: tfuncs, x: cx });
      cx += xw + gapX;
    }
    const W2 = Math.max(w, cx + 10);
    const rowsPer = Math.max(...cols.map(c => c.funcs.length), funcs.length, 1);
    const rh = 22;
    const H2 = Math.max(h, 46 + rowsPer * rh);
    svg.setAttribute('width', W2); svg.setAttribute('height', H2);
    const NS = 'http://www.w3.org/2000/svg';
    let out = '';
    // 源文件列
    out += '<rect x="8" y="6" width="'+(xw+28)+'" height="'+(H2-12)+'" rx="8" fill="#0c1626" stroke="#1e2a44"/>';
    out += '<text x="'+(xs+6)+'" y="24" fill="#38bdf8" font-size="11" font-weight="700">'+esc(file.name)+'</text>';
    funcs.forEach((f, i) => {
      const y = 40 + i * rh;
      out += '<g class="fnode src" data-id="'+f.id+'">' +
        '<rect x="'+xs+'" y="'+y+'" width="'+xw+'" height="18"/>' +
        '<text x="'+(xs+8)+'" y="'+(y+13)+'">'+esc(f.name)+'</text></g>';
    });
    // 目标列
    cols.forEach(c => {
      out += '<rect x="'+(c.x-14)+'" y="6" width="'+(xw+28)+'" height="'+(H2-12)+'" rx="8" fill="#0c1626" stroke="#7c4a12" stroke-opacity=".5"/>';
      const tn = byId.get(c.fileId);
      out += '<text x="'+(c.x+6)+'" y="24" fill="#fb923c" font-size="11" font-weight="700">'+esc(tn ? tn.name : c.fileId)+'</text>';
      c.funcs.forEach((f, i) => {
        const y = 40 + i * rh;
        out += '<g class="fnode tgt" data-id="'+f.id+'">' +
          '<rect x="'+c.x+'" y="'+y+'" width="'+xw+'" height="18"/>' +
          '<text x="'+(c.x+8)+'" y="'+(y+13)+'">'+esc(f.name)+'</text></g>';
      });
    });
    // 边
    const edgesOut = [];
    for (const col of cols) {
      const yIdx = f => Math.max(0, funcs.findIndex(g => g.id === f)) * rh;
      col.funcs.forEach((f, i) => {
        const srcs = callOuts.filter(c => c.target === f.id);
        srcs.forEach(c => {
          const sy = 40 + yIdx(c.source) + 9, ty = 40 + i * rh + 9;
          edgesOut.push({ x1: xs + xw, y1: sy, x2: col.x, y2: ty, src: c.source, tgt: c.target });
        });
      });
    }
    edgesOut.forEach(g => {
      const mx = (g.x1 + g.x2) / 2;
      out += '<path class="fcall" d="M'+g.x1+','+g.y1+' C'+mx+','+g.y1+' '+mx+','+g.y2+' '+g.x2+','+g.y2+'" data-src="'+g.src+'" data-tgt="'+g.tgt+'" marker-end="url(#ah-calls)"/>';
    });
    // 文件导入
    const imps = edges.filter(e => e.type === 'imports' && e.source === fid);
    let iy = 6;
    for (const imp of imps) {
      const t = byId.get(imp.target); if (!t) continue;
      out += '<text x="'+(W2-8)+'" y="'+(iy+12)+'" text-anchor="end" fill="#38bdf8" font-size="10">↳ import '+esc(t.name)+'</text>';
      iy += 15;
    }
    svg.innerHTML = out +
      '<defs><marker id="ah-calls" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto"><path d="M0,0L9,4.5L0,9z" fill="#fb923c"/></marker>' +
      '<marker id="ah-imp2" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0L8,4L0,8z" fill="#38bdf8"/></marker></defs>';
    Array.from(svg.querySelectorAll('.fnode')).forEach(g => {
      g.onclick = () => openNode(g.dataset.id);
      g.onmouseenter = () => {
        const id = g.dataset.id;
        svg.querySelectorAll('.fnode').forEach(n => n.classList.toggle('dim', n.dataset.id !== id));
        svg.querySelectorAll('.fcall').forEach(p => {
          const link = p.dataset.src === id || p.dataset.tgt === id;
          p.classList.toggle('hl', link); p.classList.toggle('dim', !link);
        });
      };
      g.onmouseleave = () => {
        svg.querySelectorAll('.fnode').forEach(n => n.classList.remove('dim'));
        svg.querySelectorAll('.fcall').forEach(p => p.classList.remove('hl','dim'));
      };
    });
    svg.querySelectorAll('.fcall').forEach(p => {
      p.onclick = () => { openNode(p.dataset.src); setTimeout(() => openNode(p.dataset.tgt), 10); };
    });
  };
  list.querySelectorAll('li').forEach(li => {
    li.onclick = () => {
      list.querySelectorAll('li').forEach(x => x.classList.toggle('on', x === li));
      selected = li.dataset.fid; draw(selected);
    };
  });
  if (files.length) { list.querySelector('li').classList.add('on'); draw(files[0].id); }
})();

/* ---------- Tour ---------- */
(function buildTour(){
  document.getElementById('tourCards').innerHTML = tour.map(s => {
    const chips = (s.nodeIds || []).map(id => {
      const n = byId.get(id);
      return '<span class="nodelink '+(n?n.type:'')+'" onclick="openNode(\\''+id+'\\')">'+esc(shortName(id))+'</span>';
    }).join('');
    return '<div class="tourcard"><span class="no">'+s.order+'</span><h3>'+esc(s.title)+'</h3>' +
      '<p>'+esc(s.description)+'</p><div>'+chips+'</div></div>';
  }).join('');
})();

/* ---------- 检索 ---------- */
(function buildSearch(){
  const input = document.getElementById('globalSearch');
  const tbody = document.getElementById('searchResults');
  const all = nodes.slice();
  const render = kw => {
    kw = kw.toLowerCase();
    const list = kw ? all.filter(n =>
      (n.name||'').toLowerCase().includes(kw) || (n.id||'').toLowerCase().includes(kw) ||
      (n.summary||'').toLowerCase().includes(kw) || (n.filePath||'').toLowerCase().includes(kw)
    ).slice(0, 200) : [];
    tbody.innerHTML = list.map(n =>
      '<tr onclick="openNode(\\''+n.id+'\\')">' +
      '<td><code>'+esc(n.name)+'</code></td>' +
      '<td><span class="badge '+esc(n.type)+'">'+esc(n.type)+'</span></td>' +
      '<td style="color:var(--dim)">'+esc(n.filePath || '')+(n.lineRange ? ' L'+n.lineRange[0] : '')+'</td>' +
      '<td style="color:var(--dim)">'+esc((n.summary||'').slice(0, 70))+'</td>' +
      '<td>'+(n.tags||[]).map(t=>'<span class="tag">'+esc(t)+'</span>').join('')+'</td></tr>'
    ).join('') || '<tr><td colspan="5" style="color:var(--dim)">输入关键词开始检索</td></tr>';
  };
  render('');
  input.oninput = () => render(input.value.trim());
})();
</script>
</body>
</html>
`;

writeFileSync(OUT_PATH, html, 'utf8');
console.log('✅ 已生成：' + OUT_PATH);
console.log('   节点 ' + nodes.length + ' / 边 ' + edges.length + ' / 文件级边 ' + fileEdges.length +
  ' / 分层 ' + layers.length + ' / tour ' + tour.length);
console.log('   project: ' + project.name + ' @ ' + project.gitCommitHash + ' (' + project.analyzedAt + ')');