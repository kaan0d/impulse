import { defineConfig } from '@playwright/test';

const PORT = 4173;

export default defineConfig({
  testDir: 'e2e',
  // The demo runs in real time, so tests run one at a time on an unloaded page.
  workers: 1,
  fullyParallel: false,
  retries: process.env.CI ? 1 : 0,
  reporter: process.env.CI ? 'github' : 'list',
  use: {
    baseURL: `http://localhost:${PORT}/impulse/`,
    // The installed Google Chrome, so no browser download is needed.
    channel: 'chrome',
    trace: 'retain-on-failure',
  },
  webServer: {
    command: `npm run build && npx vite preview --port ${PORT} --strictPort`,
    url: `http://localhost:${PORT}/impulse/`,
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
});
