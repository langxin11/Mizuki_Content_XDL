import { readFile, writeFile, mkdir, readdir } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
const execute = promisify(execFile);

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const imageDir = path.join(root, 'images', 'blog');
const prefix = 'https://raw.githubusercontent.com/langxin11/Picture/main/blog/';
const files = (await readdir(path.join(root, 'posts'))).filter(n => n.endsWith('.md'));
const articles = await Promise.all(files.map(async name => ({
  file: path.join(root, 'posts', name), text: await readFile(path.join(root, 'posts', name), 'utf8'),
})));
const names = new Set();
for (const article of articles) {
  for (const match of article.text.matchAll(/https:\/\/raw\.githubusercontent\.com\/langxin11\/Picture\/main\/blog\/([^\s)"<>]+)/g)) {
    names.add(decodeURIComponent(match[1]));
  }
  if (article.text.includes('../image/HW2_Q1_3.svg')) names.add('HW2_Q1_3.svg');
}
await mkdir(imageDir, { recursive: true });
if (names.size === 0) {
  console.log('Article images are already local; existing provenance manifest retained.');
  process.exit(0);
}
const pending = [...names];
const manifest = [];
async function worker() {
  while (pending.length) {
    const name = pending.shift();
    if (path.basename(name) !== name) throw new Error(`Unsafe image name: ${name}`);
    const url = prefix + encodeURIComponent(name);
    const destination = path.join(imageDir, name);
    await execute('pwsh', ['-NoProfile', '-File', path.join(root, 'scripts', 'Download-BlogImage.ps1'),
      '-Url', url, '-Destination', destination], { timeout: 90000 });
    const data = await readFile(destination);
    const ext = path.extname(name).toLowerCase();
    const valid = ext === '.svg' ? /<svg[\s>]/.test(data.toString('utf8'))
      : ext === '.png' ? data.subarray(0, 8).equals(Buffer.from([137,80,78,71,13,10,26,10]))
      : ext === '.gif' ? /^GIF8[79]a/.test(data.subarray(0, 6).toString()) : false;
    if (!valid) throw new Error(`Unexpected image format: ${name}`);
    manifest.push({ name, source: url, local: `images/blog/${name}`, bytes: data.length,
      sha256: createHash('sha256').update(data).digest('hex'), checked: '2026-10-04' });
  }
}
await Promise.all([worker(), worker(), worker(), worker()]);
// Update references only after every required download and format check succeeds.
for (const article of articles) {
  let text = article.text.replace(/https:\/\/raw\.githubusercontent\.com\/langxin11\/Picture\/main\/blog\/([^\s)"<>]+)/g,
    (_, name) => `/images/blog/${encodeURIComponent(decodeURIComponent(name))}`);
  text = text.replaceAll('../image/HW2_Q1_3.svg', '/images/blog/HW2_Q1_3.svg');
  if (text !== article.text) await writeFile(article.file, text);
}
manifest.sort((a, b) => a.name.localeCompare(b.name));
await writeFile(path.join(root, 'docs', '旧笔记配图来源.json'), JSON.stringify(manifest, null, 2) + '\n');
console.log(`Localized ${manifest.length} verified images; existing animation formats retained.`);
