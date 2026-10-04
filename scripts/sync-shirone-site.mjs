import { cp, mkdir, rm, lstat, realpath } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const site = path.join(root, 'site');
const generated = path.join(site, 'shirones', 'content');
await mkdir(generated, { recursive: true });
if ((await realpath(generated)).toLowerCase() !== generated.toLowerCase()) {
  throw new Error('Generated content must not be redirected by symbolic links.');
}
for (const name of ['posts', 'spec', 'moments', 'series']) {
  const target = path.resolve(generated, name);
  if (!target.startsWith(generated + path.sep)) throw new Error('Unsafe target');
  if ((await lstat(target).catch(() => null))?.isSymbolicLink()) throw new Error('Unsafe symbolic link');
  await rm(target, { recursive: true, force: true });
  await mkdir(target, { recursive: true });
  if (name === 'posts' || name === 'spec') await cp(path.join(root, name), target, { recursive: true });
}
const images = path.join(site, 'public', 'images');
await mkdir(images, { recursive: true });
if ((await realpath(images)).toLowerCase() !== images.toLowerCase()) throw new Error('Unsafe image target');
await cp(path.join(root, 'images'), images, { recursive: true });
console.log('Source content synchronized; drafts and personal configuration preserved.');
