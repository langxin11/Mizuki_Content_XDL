// Watch the authored folders so an Obsidian save is reflected in Astro's generated content.
import { watch } from 'node:fs';
import { cp, lstat, rm } from 'node:fs/promises';
import { spawn } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const site = path.join(root, 'site');
const targets = { posts: 'shirones/content/posts', spec: 'shirones/content/spec', images: 'public/images' };
const timers = new Map();
const watchers = [];
for (const [name, generated] of Object.entries(targets)) {
  const sourceDir = path.join(root, name);
  const destDir = path.join(site, generated);
  watchers.push(watch(sourceDir, { recursive:true }, (_, filename) => {
    if (!filename) return;
    const relative = filename.toString();
    const source = path.resolve(sourceDir, relative);
    const target = path.resolve(destDir, relative);
    if (!source.startsWith(sourceDir + path.sep) || !target.startsWith(destDir + path.sep)) return;
    clearTimeout(timers.get(target));
    timers.set(target, setTimeout(async () => {
      try {
        const stat = await lstat(source).catch(() => null);
        if (stat?.isSymbolicLink()) throw new Error('Content links are not synchronized');
        if (stat) await cp(source, target, { recursive:stat.isDirectory() });
        else await rm(target, { recursive:true, force:true });
      } catch (error) { console.error(`Content synchronization failed: ${error.message}`); }
      finally { timers.delete(target); }
    }, 300));
  }));
}
const child = spawn(process.execPath, [path.join(site, 'node_modules/astro/bin/astro.mjs'), 'dev', ...process.argv.slice(2)], {cwd:site,stdio:'inherit'});
function close() { for (const watcher of watchers) watcher.close(); for (const timer of timers.values()) clearTimeout(timer); child.kill(); }
process.on('SIGINT', close);
process.on('SIGTERM', close);
child.on('exit', code => { close(); process.exit(code ?? 1); });
