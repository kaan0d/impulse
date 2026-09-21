import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { World } from '../src/engine/world';
import { groundWorld, run } from './helpers';

describe('raycast', () => {
  it('hits a circle at its near surface with an outward normal', () => {
    const world = new World();
    const ball = world.add(Body.circle(1, 1, 5, 0));
    const hit = world.raycast(0, 0, 10, 0);
    expect(hit?.body).toBe(ball);
    expect(hit?.point.x).toBeCloseTo(4, 12);
    expect(hit?.normal.x).toBeCloseTo(-1, 12);
    expect(hit?.normal.y).toBeCloseTo(0, 12);
    expect(hit?.fraction).toBeCloseTo(0.4, 12);
  });

  it('hits a rotated box on the face it meets, with the rotated normal', () => {
    const world = new World();
    const box = world.add(Body.box(2, 2, 1, 5, 0));
    box.angle = Math.PI / 4;
    const hit = world.raycast(0, 0, 10, 0);
    // A box turned 45 degrees shows a corner first, so the ray meets one of two faces.
    expect(hit?.point.x).toBeCloseTo(5 - Math.SQRT2, 9);
    expect(Math.abs(hit?.normal.x ?? 0)).toBeCloseTo(Math.SQRT1_2, 9);
    expect(hit?.normal.x).toBeLessThan(0);
  });

  it('hits a polygon and returns the nearest of several bodies', () => {
    const world = new World();
    const far = world.add(Body.circle(0.5, 1, 8, 0));
    const near = world.add(Body.regularPolygon(6, 1, 1, 4, 0));
    const hit = world.raycast(0, 0, 10, 0);
    expect(hit?.body).toBe(near);
    expect(hit?.body).not.toBe(far);
    // The hexagon has flat sides facing the ray, at its apothem cos(30deg) from the center.
    expect(hit?.point.x).toBeCloseTo(4 - Math.cos(Math.PI / 6), 9);
  });

  it('misses when the segment ends short, passes beside, or points away', () => {
    const world = new World();
    world.add(Body.circle(1, 1, 5, 0));
    expect(world.raycast(0, 0, 3, 0)).toBeNull();
    expect(world.raycast(0, 2, 10, 2)).toBeNull();
    expect(world.raycast(0, 0, -10, 0)).toBeNull();
  });

  it('skips a body that contains the start point', () => {
    const world = new World();
    world.add(Body.box(4, 4, 1, 0, 0));
    expect(world.raycast(0, 0, 10, 0)).toBeNull();
  });

  it('respects the category mask and ignores non-collidable bodies', () => {
    const world = new World();
    const wall = world.add(Body.box(1, 4, Infinity, 5, 0));
    wall.category = 2;
    expect(world.raycast(0, 0, 10, 0, 1)).toBeNull();
    expect(world.raycast(0, 0, 10, 0, 2)?.body).toBe(wall);
    wall.collidable = false;
    expect(world.raycast(0, 0, 10, 0, 2)).toBeNull();
  });

  it('agrees with the analytic distance for random rays at random circles', () => {
    const world = new World();
    const ball = world.add(Body.circle(0.75, 1, 3, 2));
    for (let i = 0; i < 200; i++) {
      const angle = (i / 200) * Math.PI * 2;
      const hit = world.raycast(-4, -3, -4 + 20 * Math.cos(angle), -3 + 20 * Math.sin(angle));
      if (!hit) continue;
      expect(Math.hypot(hit.point.x - ball.position.x, hit.point.y - ball.position.y)).toBeCloseTo(0.75, 9);
      expect(hit.normal.x * hit.normal.x + hit.normal.y * hit.normal.y).toBeCloseTo(1, 12);
    }
  });
});

describe('boundsOverlapping', () => {
  it('returns bodies whose boxes overlap, static included, and reuses the given array', () => {
    const world = groundWorld();
    const inside = world.add(Body.circle(0.5, 1, 0, 3));
    world.add(Body.circle(0.5, 1, 20, 3));
    const out: Body[] = [];
    const found = world.boundsOverlapping(-1, 2, 1, 4, out);
    expect(found).toBe(out);
    expect(found).toEqual([inside]);
    expect(world.boundsOverlapping(-1, -2, 1, 4)).toHaveLength(2);
  });
});

describe('collision filtering', () => {
  function dropOntoGround(configure: (ball: Body, ground: Body) => void): number {
    const world = groundWorld();
    const ground = world.bodies[0];
    const ball = world.add(Body.circle(0.5, 1, 0, 2));
    configure(ball, ground);
    run(world, 2);
    return ball.position.y;
  }

  it('bodies with matching masks collide as before', () => {
    expect(dropOntoGround(() => {})).toBeCloseTo(0.5, 1);
  });

  it('a ball whose mask excludes the ground category falls through', () => {
    expect(dropOntoGround((ball) => (ball.mask = ~1))).toBeLessThan(-5);
  });

  it('the mask must hold in both directions', () => {
    expect(dropOntoGround((_ball, ground) => (ground.mask = ~1))).toBeLessThan(-5);
  });

  it('a shared negative group never collides and a shared positive group always does', () => {
    expect(dropOntoGround((ball, ground) => (ball.group = ground.group = -1))).toBeLessThan(-5);
    expect(
      dropOntoGround((ball, ground) => {
        ball.mask = 0;
        ball.group = ground.group = 3;
      }),
    ).toBeCloseTo(0.5, 1);
  });

  it('continuous collision also honours the filter', () => {
    const world = groundWorld();
    world.gravity.set(0, 0);
    const wall = world.add(Body.box(0.05, 6, Infinity, 5, 0));
    const bullet = world.add(Body.circle(0.15, 1, 0, 0));
    bullet.velocity.set(300, 0);
    bullet.mask = ~1;
    run(world, 0.5);
    expect(bullet.position.x).toBeGreaterThan(wall.position.x + 1);
  });
});
