// Production gate: drafts, old URLs, images and feed destinations must stay correct.
import { readFile, readdir, access } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const dist = path.join(root, 'site', 'dist');
const failures = [];
const origin = 'https://langxin11.github.io/';
async function countPublished(dir) {
  let count = 0;
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const file = path.join(dir, entry.name);
    if (entry.isDirectory()) count += await countPublished(file);
    else if (/\.(md|mdx)$/.test(entry.name)) {
      const source = await readFile(file, 'utf8');
      const header = source.match(/^---\r?\n([\s\S]*?)\r?\n---/)?.[1];
      if (header === undefined) throw new Error(`Missing frontmatter: ${file}`);
      if (!/^draft:\s*true\s*$/m.test(header)) count++;
    }
  }
  return count;
}
const expected = await countPublished(path.join(root, 'posts'));
const home = await readFile(path.join(dist, 'index.html'), 'utf8');
if (/本地预览|草稿预览/.test(home)) failures.push('Preview copy leaked into production homepage');
const postPages = [];
async function walk(dir) {
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const file = path.join(dir, entry.name);
    if (entry.isDirectory()) await walk(file);
    else if (entry.name === 'index.html') postPages.push(file);
  }
}
await walk(path.join(dist, 'posts'));
if (postPages.length !== expected) failures.push(`Expected ${expected} published posts, found ${postPages.length}`);
const oldSlugs = ['astro-blog搭建', '记录wsl2无法运行的修复过程', ...[1,2,3,5].map(i => `cmu_最优控制学习记录${i}`), 'cmu_最优控制学习记录4-'];
for (const slug of oldSlugs) {
  await access(path.join(dist, 'posts', slug, 'index.html')).catch(() => failures.push(`Missing old URL: ${slug}`));
}
for (const file of postPages) {
  const html = await readFile(file, 'utf8');
  if (html.includes('katex-error')) failures.push(`Math render error: ${file}`);
  if (!html.includes('核查状态')) failures.push(`Missing verification status: ${file}`);
  if (html.includes('本地草稿预览') || html.includes('【草稿预览】')) failures.push(`Draft preview leaked: ${file}`);
  for (const match of html.matchAll(/<img\b[^>]*\bsrc="([^"]+)"/g)) {
    const src = match[1].replaceAll('&amp;', '&');
    if (/^https?:/.test(src)) { failures.push(`Remote article image: ${src}`); continue; }
    if (src.startsWith('data:')) continue;
    const relative = decodeURIComponent(src.split(/[?#]/)[0]);
    const target = relative.startsWith('/') ? path.join(dist, relative.slice(1)) : path.resolve(path.dirname(file), relative);
    if (!target.startsWith(dist + path.sep)) failures.push(`Unsafe image path: ${src}`);
    else await access(target).catch(() => failures.push(`Missing article image: ${src}`));
  }
}
for (const name of ['rss.xml', 'atom.xml', 'sitemap-index.xml', 'sitemap-0.xml']) {
  const xml = await readFile(path.join(dist, name), 'utf8');
  if (xml.includes('localhost') || xml.includes('https://github.com/langxin11')) failures.push(`Wrong origin in ${name}`);
  if (!xml.includes(origin)) failures.push(`Missing production origin in ${name}`);
  if (/草稿预览|%E3%80%90%E8%8D%89%E7%A8%BF%E9%A2%84%E8%A7%88/i.test(xml)) failures.push(`Draft listed in ${name}`);
}
for (const file of ['onshape-mujoco-closed-loop', 'ubuntu-wezterm', 'matplotlib-chinese-mathtext', 'julia-vscode-intellisense']) {
  const source = await readFile(path.join(root, 'posts', '知乎迁入', file, 'index.md'), 'utf8');
  if (/^draft:\s*true\s*$/m.test(source.split(/\r?\n---/)[0])) {
    for (const name of ['rss.xml', 'atom.xml', 'sitemap-0.xml']) {
      const xml = await readFile(path.join(dist, name), 'utf8');
      if (xml.includes(file)) failures.push(`Draft listed in ${name}: ${file}`);
    }
    await access(path.join(dist, 'posts', '知乎迁入', file, 'index.html')).then(() => failures.push(`Draft page leaked: ${file}`), () => {});
  }
}
if (failures.length) throw new Error(failures.join('\n'));
console.log(`Production gate passed: ${expected} articles; all 7 old URLs retained; drafts excluded; images and feeds valid.`);
