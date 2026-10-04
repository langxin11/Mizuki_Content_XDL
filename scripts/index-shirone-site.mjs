import { realpath, readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const site = path.join(root, 'site');
const themeFile = await realpath(path.join(site, 'node_modules', 'shirones', 'package.json'));
const pagefindDir = path.resolve(path.dirname(themeFile), '..', 'pagefind');
const packageInfo = JSON.parse(await readFile(path.join(pagefindDir, 'package.json'), 'utf8'));
const pagefind = await import(pathToFileURL(path.join(pagefindDir, packageInfo.exports['.'].import)).href);
const { index, errors } = await pagefind.createIndex({ excludeSelectors: ['span.katex', 'span.katex-display', '[data-pagefind-ignore]', '.search-panel', '#search-panel'] });
if (!index || errors?.length) throw new Error(JSON.stringify(errors));
try {
  const added = await index.addDirectory({ path: path.join(site, 'dist'), glob: 'posts/**/*.html' });
  if (added.errors?.length) throw new Error(JSON.stringify(added.errors));
  const written = await index.writeFiles({ outputPath: path.join(site, 'dist', 'pagefind') });
  if (written.errors?.length) throw new Error(JSON.stringify(written.errors));
  console.log('Published articles indexed.', added);
} finally { await pagefind.close(); }
