// Copies content into the disposable local preview; never changes source drafts.
import { cp, mkdir, readFile, writeFile, readdir, rm, lstat, realpath } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const preview = path.join(root, 'preview', 'shirone');
const content = path.join(preview, 'shirones', 'content');
const config = path.join(preview, 'shirones', 'config');
const theme = path.join(preview, 'node_modules', 'shirones');
await readFile(path.join(theme, 'package.json')); // Require an initialized project.
if ((await realpath(preview)).toLowerCase() !== preview.toLowerCase()) {
  throw new Error('Preview directory must not be a junction or symbolic link.');
}

// Only these generated content directories may be replaced.
for (const name of ['posts', 'moments', 'series', 'spec']) {
  const target = path.resolve(content, name);
  if (!target.startsWith(content + path.sep)) throw new Error('Unsafe target');
  const stat = await lstat(target).catch(() => null);
  if (stat?.isSymbolicLink()) throw new Error('Generated content must not be a link');
  await rm(target, { recursive: true, force: true });
  await mkdir(target, { recursive: true });
}
await cp(path.join(root, 'posts'), path.join(content, 'posts'), { recursive: true });
await mkdir(path.join(preview, 'public', 'images'), { recursive: true });
await cp(path.join(root, 'images'), path.join(preview, 'public', 'images'), { recursive: true });
let posts = 0, drafts = 0;
async function labelDrafts(dir) {
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const file = path.join(dir, entry.name);
    if (entry.isDirectory()) { await labelDrafts(file); continue; }
    if (!entry.name.endsWith('.md')) continue;
    posts++;
    let text = await readFile(file, 'utf8');
    const match = text.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n/);
    if (!match) throw new Error(`Missing frontmatter: ${file}`);
    if (!/^draft:\s*true\s*$/m.test(match[1])) continue;
    drafts++;
    let header = match[1].replace(/^draft:\s*true\s*$/m, 'draft: false');
    header = header.replace(/^title:\s*(.+)$/m, (_, title) =>
      `title: ${JSON.stringify('【草稿预览】' + title.trim())}`);
    text = `---\n${header}\n---\n\n> **本地草稿预览**：仅用于检查排版和阅读体验，源文章仍为草稿，未发布，也未同步修改知乎。\n\n` + text.slice(match[0].length);
    await writeFile(file, text);
  }
}
await labelDrafts(path.join(content, 'posts'));
await writeFile(path.join(content, 'spec', 'about.md'), `# 关于\n\n这里是泽夕不嘻嘻的学习与实践笔记，内容包括最优控制、机器人建模，以及科研开发工具。\n\n## 阅读路线\n\nCMU 最优控制系列：动力学与数值优化 → LQR → MPC → 非线性轨迹优化 → 旋转与四元数。\n\n## 当前预览\n\n本站使用 [Shirone](https://github.com/LyraVoid/Shirone) 主题。当前为本地预览，包含 ${posts} 篇笔记，其中 ${drafts} 篇知乎迁入文章仍为草稿。部分实验未完整复现，具体适用环境、核查结果和局限见正文。\n\n[GitHub](https://github.com/langxin11) · [知乎](https://www.zhihu.com/people/63-21-78-38-42)\n`);

async function configure(file, transform) {
  // Reapply changes from the package defaults so synchronization is repeatable.
  const text = await readFile(path.join(theme, 'template', 'shirones', 'config', file), 'utf8');
  await writeFile(path.join(config, file), transform(text));
}
await configure('siteConfig.ts', text => text
  .replace('https://shirone.mysqil.com/', 'http://localhost:4321/')
  .replaceAll('title: "Shirone"', 'title: "langxin11 · 学习笔记"')
  .replace('subtitle: "A Material 3 anime blog"', 'subtitle: "最优控制 · 机器人学 · 科研工具"')
  .replace('lang: "en"', 'lang: "zh_CN"')
  .replace('hue: 315', 'hue: 230')
  .replace('allowMotion: true', 'allowMotion: false')
  .replace(/subtitle: \[[\s\S]*?\],/, 'subtitle: ["最优控制 · 机器人学 · 科研工具"],')
  .replace('depth: 2', 'depth: 3')
  .replace('animation: "ken-burns"', 'animation: "none"'));
await configure('profileConfig.ts', () => `import type { ProfileConfig } from "@/types/config";\nexport const profileConfig: ProfileConfig = {\n  avatar: "/avatar.jpg",\n  name: "泽夕不嘻嘻",\n  bio: "The world is big, you have to go and see",\n  links: [\n    { name: "GitHub", icon: "fa6-brands:github", url: "https://github.com/langxin11" },\n    { name: "知乎", icon: "fa6-brands:zhihu", url: "https://www.zhihu.com/people/63-21-78-38-42" },\n  ],\n};\n`);
await configure('navBarConfig.ts', text => text.replace(
  /const defaultNavBarConfig: NavBarConfig = \{[\s\S]*?\n\};/,
  'const defaultNavBarConfig: NavBarConfig = {\n  links: [LinkPresets.Home, LinkPresets.Archive, LinkPresets.Categories, LinkPresets.Tags, LinkPresets.About],\n};'));
await configure('sidebarConfig.ts', text => text
  .replace('{ type: "music", enable: true', '{ type: "music", enable: false')
  .replace(/(type: "series",\s*)enable: true/, '$1enable: false'));
await configure('announcementConfig.ts', text => text
  .replace('The only way to do great work is to love what you do', `本地预览：${posts} 篇笔记，含 ${drafts} 篇知乎迁入草稿。未完整复现的实验已在正文注明。`)
  .replace('enable: true', 'enable: false'));
await configure('postListConfig.ts', text => text.replace('pageSize: 8', 'pageSize: 12'));
await configure('licenseConfig.ts', text => text.replace('enable: true', 'enable: false'));
for (const name of ['friends', 'moments', 'anime', 'compass', 'albums', 'skills', 'projects', 'devices', 'games', 'timeline', 'series', 'music', 'umami', 'comment', 'llms']) {
  await configure(`${name}Config.ts`, text => text.replace(/(\n\s*)enable: true/, '$1enable: false'));
}
// Retain the existing site's avatar when available; the fallback is bundled locally.
await cp(path.join(theme, 'template', 'public', 'logo', 'icon.webp'), path.join(preview, 'public', 'avatar.jpg')).catch(async () => {
  await cp(path.join(theme, 'src', 'assets', 'images', 'demo-avatar.webp'), path.join(preview, 'public', 'avatar.jpg'));
});
await cp(path.resolve(root, '..', 'Mizuki', 'src', 'assets', 'images', 'avatar.jpg'), path.join(preview, 'public', 'avatar.jpg')).catch(() => {});
await writeFile(path.join(preview, 'public', 'robots.txt'), 'User-agent: *\nDisallow: /\n');
// The package's Pagefind hook uses URL.pathname, which is /F:/... on Windows.
// Build the index separately with native filesystem paths (see index script).
await writeFile(path.join(preview, 'astro.config.mjs'), `import { defineConfig } from "astro/config";\nimport shirones from "shirones";\nexport default defineConfig({ integrations: [shirones({ pagefind: false })] });\n`);
console.log(`Local preview synchronized: ${posts} posts (${drafts} visibly labeled drafts). Source files unchanged.`);
