import { build } from 'vite';
import { describe, expect, it } from 'vitest';

// The parts of a bundle this test reads; `build` returns a wider union (bundles or a watcher).
interface Bundle {
  output: { fileName: string; source?: string | Uint8Array; code?: string }[];
}

// Builds in memory with the repo's vite.config.ts, exactly as GitHub Pages will get it.
async function buildOutput(): Promise<Map<string, string>> {
  const result = (await build({ logLevel: 'silent', build: { write: false } })) as unknown as Bundle | Bundle[];
  const files = new Map<string, string>();
  for (const bundle of Array.isArray(result) ? result : [result]) {
    for (const { fileName, source, code } of bundle.output) {
      const content = code ?? source ?? '';
      files.set(fileName, typeof content === 'string' ? content : new TextDecoder().decode(content));
    }
  }
  return files;
}

const BASE = '/impulse/';

describe('production build under the GitHub Pages base path', () => {
  it('emits the demo and the benchmark page with base-aware, non-absolute URLs', async () => {
    const files = await buildOutput();
    expect([...files.keys()]).toEqual(expect.arrayContaining(['index.html', 'bench/index.html']));

    for (const page of ['index.html', 'bench/index.html']) {
      const html = files.get(page) ?? '';
      const urls = [...html.matchAll(/(?:src|href)="([^"]+)"/g)].map((match) => match[1]);
      expect(urls.length, `${page} has script or link URLs`).toBeGreaterThan(0);
      for (const url of urls) {
        const rooted = url.startsWith('/');
        expect(!rooted || url.startsWith(BASE), `${page}: ${url} must be relative or under ${BASE}`).toBe(true);
      }
      expect(html).toContain(`src="${BASE}assets/`);
    }
  }, 60_000);
});
