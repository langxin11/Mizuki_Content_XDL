import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const entries = JSON.parse(await readFile(path.join(root, 'docs', '文章发布状态.json'), 'utf8'));
for (const entry of entries) {
  const file = path.join(root, entry.file);
  let text = await readFile(file, 'utf8');
  const heading = '## 整理与核查说明';
  if (!text.includes(heading)) throw new Error(`Missing verification section: ${entry.file}`);
  const status = `**核查状态：${entry.status}（2026-10-04）。** ${entry.scope}`;
  if (/\*\*核查状态：[^\n]+/.test(text)) text = text.replace(/\*\*核查状态：[^\n]+/, status);
  else text = text.replace(heading, `${heading}\n\n${status}`);
  if (entry.file.includes('CMU_') && !text.includes('本笔记原有许可为')) {
    text = text.replace(heading, `${heading}\n\n本笔记原有许可为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。引用的课程材料、代码和图片仍须遵守其各自的许可。`);
  }
  await writeFile(file, text);
}
console.log(`Verification scope labeled in ${entries.length} notes; publication flags unchanged.`);
