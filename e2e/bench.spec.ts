import { expect, test } from '@playwright/test';

test('the benchmark page runs to completion and both broadphases agree', async ({ page }) => {
  test.setTimeout(120_000);
  const problems: string[] = [];
  page.on('pageerror', (error) => problems.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') problems.push(message.text());
  });

  await page.goto('bench/');
  await expect(page).toHaveTitle('Impulse Benchmark');
  await page.waitForFunction(() => document.body.dataset.done === 'true', undefined, { timeout: 100_000 });

  await expect(page.locator('#scenes tbody tr')).toHaveCount(5);
  await expect(page.locator('#broadphase tbody tr')).toHaveCount(5);
  const samePairs = await page.locator('#broadphase tbody tr td:last-child').allTextContents();
  expect(samePairs).toEqual(['yes', 'yes', 'yes', 'yes', 'yes']);
  // The average, p95 and max timing cells are real numbers.
  for (const cell of await page.locator('#scenes tbody tr td:nth-child(n+4):nth-child(-n+6)').allTextContents()) {
    expect(Number.isFinite(Number(cell))).toBe(true);
  }
  expect(problems).toEqual([]);
});

test('the demo links to the benchmark and back', async ({ page }) => {
  await page.goto('./');
  await page.getByRole('link', { name: 'Benchmark' }).click();
  await expect(page).toHaveURL(/\/impulse\/bench\/$/);
  await page.getByRole('link', { name: 'Back to the demo' }).click();
  await expect(page).toHaveURL(/\/impulse\/$/);
});
