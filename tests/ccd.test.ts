import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { sweepAgainstStatics } from '../src/engine/ccd';
import { FIXED_DT } from '../src/engine/stepper';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';

// Steps until `stop` returns true or `limit` steps pass; returns the steps used.
function runUntil(world: World, limit: number, stop: () => boolean): number {
  for (let step = 1; step <= limit; step++) {
    world.step(FIXED_DT);
    if (stop()) return step;
  }
  return limit;
}

// Static wall face at x = 5, only 0.05 m thick: a 300 m/s bullet moves 5 m per step.
function wallWorld(makeWall: () => Body, continuous: boolean): { world: World; wall: Body } {
  const world = new World();
  world.gravity.set(0, 0);
  world.continuousCollision = continuous;
  const wall = world.add(makeWall());
  return { world, wall };
}

const thinBoxWall = () => Body.box(0.05, 20, Infinity, 5, 0);
const thinPolygonWall = () => Body.polygon([new Vec2(-0.025, -10), new Vec2(0.025, -10), new Vec2(0.025, 10), new Vec2(-0.025, 10)], Infinity, 5, 0);

describe('continuous collision', () => {
  it.each([
    ['box wall', thinBoxWall],
    ['polygon wall', thinPolygonWall],
  ])('a fast circle tunnels through a thin %s without CCD and is stopped with it', (_name, makeWall) => {
    const fire = (continuous: boolean) => {
      const { world } = wallWorld(makeWall, continuous);
      const bullet = world.add(Body.circle(0.2, 1, 0, 0));
      bullet.velocity.set(300, 0);
      runUntil(world, 60, () => false);
      return bullet;
    };
    expect(fire(false).position.x).toBeGreaterThan(5.5); // proves the setup really tunnels
    const stopped = fire(true);
    expect(stopped.position.x).toBeLessThan(5);
    expect(stopped.velocity.x).toBeLessThan(1);
  });

  it('a fast box and a fast polygon are stopped too', () => {
    for (const make of [() => Body.box(0.4, 0.4, 1, 0, 0), () => Body.regularPolygon(5, 0.3, 1, 0, 0)]) {
      const { world } = wallWorld(thinBoxWall, true);
      const bullet = world.add(make());
      bullet.velocity.set(250, 0);
      const worst = { x: -Infinity };
      runUntil(world, 90, () => {
        worst.x = Math.max(worst.x, bullet.position.x);
        return false;
      });
      expect(worst.x).toBeLessThan(5);
    }
  });

  it('a bullet hitting the floor from above stays on top of a thin static floor', () => {
    const world = new World();
    const floor = world.add(Body.box(40, 0.05, Infinity, 0, 0));
    const bullet = world.add(Body.circle(0.15, 1, 0, 30));
    bullet.velocity.set(0, -400);
    const lowest = { y: Infinity };
    runUntil(world, 120, () => {
      lowest.y = Math.min(lowest.y, bullet.position.y);
      return false;
    });
    expect(lowest.y).toBeGreaterThan(floor.position.y);
  });

  it('a bullet is stopped by a small static circle hit head-on', () => {
    const { world } = wallWorld(() => Body.circle(0.3, Infinity, 5, 0), true);
    const bullet = world.add(Body.circle(0.1, 1, 0, 0));
    bullet.velocity.set(400, 0);
    const worst = { x: -Infinity };
    runUntil(world, 60, () => {
      worst.x = Math.max(worst.x, bullet.position.x);
      return false;
    });
    expect(worst.x).toBeLessThan(5);
  });

  it('a bounce off the wall still uses the solver, so restitution applies', () => {
    const { world } = wallWorld(thinBoxWall, true);
    const bullet = world.add(Body.circle(0.2, 1, 0, 0));
    bullet.restitution = 0.5;
    bullet.velocity.set(300, 0);
    runUntil(world, 10, () => bullet.velocity.x < 0);
    expect(bullet.velocity.x).toBeLessThan(0);
    expect(bullet.velocity.x).toBeCloseTo(-150, -1);
  });

  it('bodies that move less than their core radius per step are untouched', () => {
    const on = wallWorld(thinBoxWall, true);
    const off = wallWorld(thinBoxWall, false);
    const results = [on, off].map(({ world }) => {
      const ball = world.add(Body.circle(0.5, 1, 0, 0));
      ball.velocity.set(20, 0); // 0.33 m per step, below the 0.5 m core
      runUntil(world, 30, () => false);
      return [ball.position.x, ball.velocity.x];
    });
    expect(results[0]).toEqual(results[1]);
  });

  it('a body that starts inside a static shape keeps its position (the solver handles existing overlap)', () => {
    const block = Body.box(4, 4, Infinity, 0, 0);
    const insideToInside = Body.circle(0.5, 1, 1.5, 0);
    sweepAgainstStatics(insideToInside, 0.5, 0, [block]);
    expect(insideToInside.position.x).toBe(1.5);

    const insideToOutside = Body.circle(0.5, 1, 3, 0);
    sweepAgainstStatics(insideToOutside, 0.5, 0, [block]);
    expect(insideToOutside.position.x).toBe(3);
  });

  it('a body that ends before the wall is not moved', () => {
    const wall = Body.box(0.05, 20, Infinity, 5, 0);
    const body = Body.circle(0.2, 1, 3, 0);
    sweepAgainstStatics(body, 0, 0, [wall]);
    expect(body.position.x).toBe(3);
  });

  it('core radius is the inscribed circle', () => {
    expect(Body.circle(0.7, 1, 0, 0).coreRadius).toBe(0.7);
    expect(Body.box(2, 0.5, 1, 0, 0).coreRadius).toBe(0.25);
    expect(Body.regularPolygon(4, Math.SQRT2, 1, 0, 0).coreRadius).toBeCloseTo(1, 9);
    expect(Body.regularPolygon(3, 1, 1, 0, 0).coreRadius).toBeCloseTo(0.5, 9);
  });
});
