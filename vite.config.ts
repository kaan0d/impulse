import { defineConfig } from 'vitest/config';

export default defineConfig({
  // GitHub Pages serves the site from https://kaan0d.github.io/impulse/
  base: '/impulse/',
  test: {
    include: ['tests/**/*.test.ts'],
  },
});
