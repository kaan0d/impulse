import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { collide } from '../src/engine/collide';
import { CONTACT_MARGIN, Manifold } from '../src/engine/manifold';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';
import { run } from './helpers';

const square = (x: number) => Body.polygon([new Vec2(-0.5, -0.5), new Vec2(0.5, -0.5), new Vec2(0.5, 0.5), new Vec2(-0.5, 0.5)], 1, x, 0);
const disc = (x: number) => Body.circle(0.5, 1, x, 0);
const box = (x: number) => Body.box(1, 1, 1, x, 0);

// Each pair is built with unit-sized shapes, so the second center sits at 1 + gap.
const pairs: [string, (x: number) => Body, (x: number) => Body][] = [
  ['circle and circle', disc, disc],
  ['circle and box', disc, box],
  ['box and circle', box, disc],
  ['box and box', box, box],
  ['box and polygon', box, square],
  ['polygon and box', square, box],
  ['polygon and polygon', square, square],
  ['circle and polygon', disc, square],
  ['polygon and circle', square, disc],
];

describe('contacts within the margin', () => {
  it.each(pairs)('%s: a gap inside the margin gives a contact with negative depth', (_name, makeA, makeB) => {
    const gap = CONTACT_MARGIN * 0.75;
    const a = makeA(0);
    const b = makeB(1 + gap);
    const manifold = new Manifold(a, b);
    expect(collide(a, b, manifold)).toBe(true);
    expect(manifold.normal.x).toBeCloseTo(1, 9);
    expect(manifold.count).toBeGreaterThan(0);
    for (let k = 0; k < manifold.count; k++) expect(manifold.depths[k]).toBeCloseTo(-gap, 9);
  });

  it.each(pairs)('%s: a gap outside the margin gives no contact', (_name, makeA, makeB) => {
    const a = makeA(0);
    const b = makeB(1 + CONTACT_MARGIN * 1.5);
    expect(collide(a, b, new Manifold(a, b))).toBe(false);
  });
});

describe('speculative stopping', () => {
  function wallWorld(): { world: World; wall: Body } {
    const world = new World();
    world.gravity.set(0, 0);
    // Wall face at x = 4.5.
    const wall = world.add(Body.box(1, 20, Infinity, 5, 0));
    return { world, wall };
  }

  it('a body closing on a wall from inside the margin stops at the surface instead of sinking in', () => {
    const { world } = wallWorld();
    world.continuousCollision = false;
    const ball = world.add(Body.circle(0.5, 1, 4 - CONTACT_MARGIN * 0.75, 0));
    ball.velocity.set(20, 0);
    world.step(1 / 60);
    // Unchecked it would travel 0.33 m in one step.
    expect(ball.position.x).toBeLessThanOrEqual(4 + 1e-6);
    expect(ball.velocity.x).toBeLessThan(0.1);
  });

  it('a body sliding along a wall inside the margin keeps its speed', () => {
    const { world } = wallWorld();
    world.continuousCollision = false;
    const ball = world.add(Body.circle(0.5, 1, 4 - CONTACT_MARGIN * 0.75, 0));
    ball.velocity.set(0, 5);
    world.step(1 / 60);
    expect(ball.velocity.y).toBeCloseTo(5, 9);
    expect(ball.velocity.x).toBeCloseTo(0, 9);
  });

  it('a fast bullet stopped by continuous collision is resolved by the solver and never stays pinned', () => {
    const { world } = wallWorld();
    const bullet = world.add(Body.circle(0.15, 1, 0, 0));
    bullet.velocity.set(250, 0);
    run(world, 0.25);
    expect(bullet.position.x).toBeLessThan(4.5 - 0.15 + 0.05);
    expect(bullet.velocity.x).toBeLessThan(1);
  });
});
