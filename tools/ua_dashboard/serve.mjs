/**
 * serve.mjs — 只读静态文件服务器（用于在浏览器中查看 ua-code-map.html 等本地产物）
 * 用法：node tools/ua_dashboard/serve.mjs [port]  默认 8754
 * 根目录 = 仓库根；仅允许 GET；路径穿越防护。
 */
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { extname, join, normalize, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { dirname } from 'node:path';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const PORT = Number(process.argv[2] || 8754);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.md': 'text/markdown; charset=utf-8',
};
const SAFE = new Set(['.html', '.js', '.mjs', '.json', '.css', '.svg', '.png', '.md', '.txt']);

createServer(async (req, res) => {
  try {
    const url = new URL(req.url, 'http://x');
    let p = decodeURIComponent(url.pathname);
    if (p === '/' ) p = '/docs/work/ua-code-map.html';
    const file = normalize(join(ROOT, p));
    if (!file.startsWith(ROOT + sep) && file !== ROOT) { res.writeHead(403); return res.end('403'); }
    const data = await readFile(file);
    const mime = MIME[extname(file).toLowerCase()] || 'application/octet-stream';
    res.writeHead(200, { 'Content-Type': mime, 'X-Content-Type-Options': 'nosniff' });
    res.end(data);
  } catch (e) {
    res.writeHead(e.code === 'ENOENT' ? 404 : 500);
    res.end(e.code === 'ENOENT' ? '404' : '500');
  }
}).listen(PORT, () => {
  console.log('serving ' + ROOT + ' at http://127.0.0.1:' + PORT + '/');
});