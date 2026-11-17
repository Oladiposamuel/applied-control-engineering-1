import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { extname, join, relative, resolve } from 'node:path';

const projectRoot = resolve(import.meta.dirname, '..');
const outputRoot = join(projectRoot, 'dist');

function fail(message) {
  console.error(`ERROR: ${message}`);
  process.exitCode = 1;
}

function walk(directory) {
  return readdirSync(directory).flatMap((entry) => {
    const absolute = join(directory, entry);
    return statSync(absolute).isDirectory() ? walk(absolute) : [absolute];
  });
}

if (!existsSync(join(outputRoot, 'index.html'))) {
  fail('dist/index.html is required as the Vercel entry page.');
  process.exit();
}

const outputFiles = walk(outputRoot);
const htmlFiles = outputFiles.filter((file) => extname(file) === '.html');
const localReference = /\b(?:href|src)="([^"]+)"/g;
const idAttribute = /\bid="([^"]+)"/g;

for (const htmlFile of htmlFiles) {
  const html = readFileSync(htmlFile, 'utf8');
  const pageName = relative(outputRoot, htmlFile);
  const ids = [...html.matchAll(idAttribute)].map((match) => match[1]);
  const duplicateIds = ids.filter((id, index) => ids.indexOf(id) !== index);

  if (duplicateIds.length) {
    fail(`${pageName} contains duplicate IDs: ${[...new Set(duplicateIds)].join(', ')}`);
  }

  for (const match of html.matchAll(localReference)) {
    const reference = match[1].split(/[?#]/, 1)[0];
    if (!reference || /^(?:[a-z]+:|\/\/|#)/i.test(reference)) continue;

    const target = reference.startsWith('/')
      ? join(outputRoot, reference.slice(1))
      : resolve(htmlFile, '..', reference);

    if (!existsSync(target)) fail(`${pageName} references missing asset ${match[1]}`);
  }

  for (const scriptMatch of html.matchAll(/\bsrc="([^"]+\.js)"/g)) {
    const scriptPath = resolve(htmlFile, '..', scriptMatch[1]);
    if (!existsSync(scriptPath)) continue;

    const script = readFileSync(scriptPath, 'utf8');
    const referencedIds = [...script.matchAll(/\$\('([^']+)'\)/g)].map((match) => match[1]);
    const missingIds = [...new Set(referencedIds.filter((id) => !ids.includes(id)))];
    if (missingIds.length) {
      fail(`${relative(outputRoot, scriptPath)} references IDs absent from ${pageName}: ${missingIds.join(', ')}`);
    }
  }
}

for (const script of outputFiles.filter((file) => extname(file) === '.js')) {
  const source = readFileSync(script, 'utf8');
  try {
    new Function(source);
  } catch (error) {
    fail(`${relative(outputRoot, script)} has invalid JavaScript: ${error.message}`);
  }
}

if (!process.exitCode) {
  console.log(`Validated ${htmlFiles.length} pages and ${outputFiles.length} deployable files in dist/.`);
}
