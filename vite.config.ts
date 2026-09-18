import { defineConfig } from 'vitest/config';

export default defineConfig({
  // GitHub Pages serves the site from https://kaan0d.github.io/impulse/
  base: '/impulse/',
  build: {
    // Two pages: the demo and the benchmark.
    rollupOptions: {
      input: { main: 'index.html', bench: 'bench/index.html' },
    },
  },
  test: {
    include: ['tests/**/*.test.ts'],
  },
});
