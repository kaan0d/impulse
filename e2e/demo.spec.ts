import { expect, test, type Page } from '@playwright/test';

// The demo canvas is 900 x 600 px for 18 x 12 m, y up from the bottom edge.
const PIXELS_PER_METER = 50;

interface BodySnapshot {
  kind: string;
  x: number;
  y: number;
  movable: boolean;
  awake: boolean;
}

async function bodies(page: Page): Promise<BodySnapshot[]> {
  return page.evaluate(() =>
    window.impulse.world.bodies.map((body) => ({
      kind: body.shape.kind,
      x: body.position.x,
      y: body.position.y,
      movable: body.invMass !== 0,
      awake: body.awake,
    })),
  );
}

async function movableBodies(page: Page): Promise<BodySnapshot[]> {
  return (await bodies(page)).filter((body) => body.movable);
}

async function canvasBox(page: Page): Promise<{ x: number; y: number; width: number; height: number }> {
  const box = await page.locator('#view').boundingBox();
  if (!box) throw new Error('canvas not laid out');
  return box;
}

// Screen position of a world point, for mouse and drag actions.
async function screenPoint(page: Page, worldX: number, worldY: number): Promise<{ x: number; y: number }> {
  const box = await canvasBox(page);
  return {
    x: box.x + (worldX * PIXELS_PER_METER * box.width) / 900,
    y: box.y + ((600 - worldY * PIXELS_PER_METER) * box.height) / 600,
  };
}

// Drags a palette item onto the canvas so it lands at the given world point.
async function dropShape(page: Page, shape: string, worldX: number, worldY: number): Promise<void> {
  const box = await canvasBox(page);
  const point = await screenPoint(page, worldX, worldY);
  await page.locator(`.item[data-shape="${shape}"]`).dragTo(page.locator('#view'), {
    targetPosition: { x: point.x - box.x, y: point.y - box.y },
  });
}

async function chooseScene(page: Page, name: string): Promise<void> {
  await page.locator('#scene').selectOption({ label: name });
}

const problemsByPage = new WeakMap<Page, string[]>();

test.beforeEach(async ({ page }) => {
  const problems: string[] = [];
  problemsByPage.set(page, problems);
  page.on('pageerror', (error) => problems.push(`pageerror: ${error.message}`));
  page.on('console', (message) => {
    if (message.type() === 'error') problems.push(`console: ${message.text()}`);
  });
  await page.goto('./');
  await page.waitForFunction(() => Boolean(window.impulse));
});

test.afterEach(async ({ page }) => {
  expect(problemsByPage.get(page)).toEqual([]);
});

test('loads the playground with a running simulation', async ({ page }) => {
  await expect(page).toHaveTitle('Impulse');
  await expect(page.locator('#view')).toBeVisible();
  const before = await movableBodies(page);
  expect(before.length).toBeGreaterThan(5);
  await page.waitForTimeout(500);
  const after = await movableBodies(page);
  expect(after.some((body, i) => body.x !== before[i].x || body.y !== before[i].y)).toBe(true);
  await expect(page.locator('#status')).toContainText('bodies');
});

test('the scene picker lists every scene and switching rebuilds the world', async ({ page }) => {
  const names = await page.locator('#scene option').allTextContents();
  expect(names).toEqual(expect.arrayContaining(['Playground', 'Pyramid', 'Joints', 'Polygon pile', 'Empty']));

  await chooseScene(page, 'Empty');
  expect(await bodies(page)).toHaveLength(1); // just the ground
  await chooseScene(page, 'Pyramid');
  expect(await bodies(page)).toHaveLength(1 + 55);
  await chooseScene(page, 'Polygon pile');
  const kinds = new Set((await bodies(page)).map((body) => body.kind));
  expect(kinds).toEqual(new Set(['box', 'circle', 'polygon']));
});

test('pause freezes the world, step advances once, resume continues', async ({ page }) => {
  await page.getByRole('button', { name: 'Pause' }).click();
  await expect(page.locator('#pause')).toHaveText('Resume');
  const frozen = await movableBodies(page);
  await page.waitForTimeout(400);
  expect(await movableBodies(page)).toEqual(frozen);

  await page.getByRole('button', { name: 'Step' }).click();
  const stepped = await movableBodies(page);
  expect(stepped).not.toEqual(frozen);
  await page.waitForTimeout(300);
  expect(await movableBodies(page)).toEqual(stepped);

  await page.getByRole('button', { name: 'Resume' }).click();
  await page.waitForTimeout(300);
  expect(await movableBodies(page)).not.toEqual(stepped);
});

test('reset restores the scene as it was built', async ({ page }) => {
  // Paused first, so the rebuilt scene is captured before any step has run.
  await page.getByRole('button', { name: 'Pause' }).click();
  await page.getByRole('button', { name: 'Reset' }).click();
  const initial = await movableBodies(page);
  await page.getByRole('button', { name: 'Resume' }).click();
  await page.waitForTimeout(800);
  expect(await movableBodies(page)).not.toEqual(initial);

  await page.getByRole('button', { name: 'Pause' }).click();
  await page.getByRole('button', { name: 'Reset' }).click();
  expect(await movableBodies(page)).toEqual(initial);
});

test('a shape dragged from the palette lands where dropped and falls to the ground', async ({ page }) => {
  await chooseScene(page, 'Empty');
  await dropShape(page, 'circle', 9, 8);

  await expect.poll(async () => (await movableBodies(page)).length).toBe(1);
  const spawned = (await movableBodies(page))[0];
  expect(spawned.kind).toBe('circle');
  expect(Math.abs(spawned.x - 9)).toBeLessThan(0.5);

  // Ground top is at y = 1 and the circle's radius is 0.5.
  await expect.poll(async () => (await movableBodies(page))[0].y, { timeout: 10_000 }).toBeCloseTo(1.5, 1);
});

test('every palette shape spawns as the right kind', async ({ page }) => {
  await chooseScene(page, 'Empty');
  const expected: [string, string][] = [
    ['circle', 'circle'],
    ['box', 'box'],
    ['triangle', 'polygon'],
    ['diamond', 'polygon'],
    ['pentagon', 'polygon'],
    ['hexagon', 'polygon'],
  ];
  for (const [index, [shape]] of expected.entries()) await dropShape(page, shape, 2.5 + index * 2.6, 9);
  await expect.poll(async () => (await movableBodies(page)).map((body) => body.kind)).toEqual(expected.map(([, kind]) => kind));
});

test('the mouse grabs a resting body, carries it, and drops it', async ({ page }) => {
  await chooseScene(page, 'Empty');
  await dropShape(page, 'box', 9, 3);
  await expect.poll(async () => (await movableBodies(page))[0]?.awake, { timeout: 15_000 }).toBe(false);

  const resting = (await movableBodies(page))[0];
  const grabAt = await screenPoint(page, resting.x, resting.y);
  const carryTo = await screenPoint(page, 4, 7);
  await page.mouse.move(grabAt.x, grabAt.y);
  await page.mouse.down();
  await page.mouse.move((grabAt.x + carryTo.x) / 2, (grabAt.y + carryTo.y) / 2, { steps: 8 });
  await page.mouse.move(carryTo.x, carryTo.y, { steps: 8 });
  await expect.poll(async () => (await movableBodies(page))[0].x, { timeout: 5000 }).toBeLessThan(4.6);
  await expect.poll(async () => (await movableBodies(page))[0].y, { timeout: 5000 }).toBeGreaterThan(6);
  expect(await page.evaluate(() => window.impulse.world.joints.length)).toBe(1);

  await page.mouse.up();
  expect(await page.evaluate(() => window.impulse.world.joints.length)).toBe(0);
  // Released in mid-air, it falls back to the ground.
  await expect.poll(async () => (await movableBodies(page))[0].y, { timeout: 10_000 }).toBeLessThan(1.6);
});

test('grabbing empty space or the ground does nothing', async ({ page }) => {
  await chooseScene(page, 'Empty');
  const air = await screenPoint(page, 9, 8);
  const ground = await screenPoint(page, 9, 0.5);
  for (const point of [air, ground]) {
    await page.mouse.move(point.x, point.y);
    await page.mouse.down();
    await page.mouse.up();
  }
  expect(await page.evaluate(() => window.impulse.world.joints.length)).toBe(0);
});

test('continuous collision stops bullets at the wall and switching it off lets them through', async ({ page }) => {
  const wallX = 13;
  await chooseScene(page, 'Bullets (continuous collision)');
  await page.waitForTimeout(1200);
  expect((await movableBodies(page)).filter((body) => body.x > wallX)).toHaveLength(0);

  await page.locator('#continuous').uncheck();
  await page.getByRole('button', { name: 'Reset' }).click();
  await page.waitForTimeout(1200);
  expect((await movableBodies(page)).filter((body) => body.x > wallX).length).toBeGreaterThan(0);
});

test('overlay checkboxes switch the debug drawing', async ({ page }) => {
  const flags = () => page.evaluate(() => [window.impulse.renderer.showContacts, window.impulse.renderer.showJoints]);
  expect(await flags()).toEqual([true, true]);
  await page.locator('#show-contacts').uncheck();
  expect(await flags()).toEqual([false, true]);
  await page.locator('#show-joints').uncheck();
  expect(await flags()).toEqual([false, false]);
});

test('the pyramid falls asleep and the status line says so', async ({ page }) => {
  await chooseScene(page, 'Pyramid');
  await expect.poll(async () => (await movableBodies(page)).every((body) => !body.awake), { timeout: 20_000 }).toBe(true);
  await expect(page.locator('#status')).toContainText('0 awake');
});

test('the joint scene keeps the motor paddle turning', async ({ page }) => {
  await chooseScene(page, 'Joints');
  await page.waitForTimeout(600);
  const paddleAngle = () =>
    page.evaluate(() => {
      const body = window.impulse.world.bodies.find((b) => b.shape.kind === 'box' && b.mass === 2);
      return body ? body.angle : Number.NaN;
    });
  const first = await paddleAngle();
  await page.waitForTimeout(600);
  expect(Math.abs((await paddleAngle()) - first)).toBeGreaterThan(0.3);
});
