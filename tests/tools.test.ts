import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { RevoluteJoint } from '../src/engine/joint';
import { FIXED_DT } from '../src/engine/stepper';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';
import { scenes } from '../src/demo/scenes';
import { run } from './helpers';

describe('Body.containsPoint', () => {
  it('circle', () => {
    const circle = Body.circle(1, 1, 2, 3);
    expect(circle.containsPoint(2, 3)).toBe(true);
    expect(circle.containsPoint(2.9, 3)).toBe(true);
    expect(circle.containsPoint(3, 3)).toBe(true); // the edge counts
    expect(circle.containsPoint(3.1, 3)).toBe(false);
  });

  it('rotated box', () => {
    const box = Body.box(4, 2, 1, 0, 0);
    box.angle = Math.PI / 2; // now 2 wide and 4 tall
    expect(box.containsPoint(0.9, 1.9)).toBe(true);
    expect(box.containsPoint(1.9, 0.9)).toBe(false);
    expect(box.containsPoint(0, -2.1)).toBe(false);
  });

  it('rotated polygon', () => {
    const triangle = Body.regularPolygon(3, 1, 1, 5, 5); // corner up
    expect(triangle.containsPoint(5, 5)).toBe(true);
    expect(triangle.containsPoint(5, 5.9)).toBe(true); // just under the top corner
    expect(triangle.containsPoint(5.8, 5.9)).toBe(false); // outside the slanted edge
    triangle.angle = Math.PI; // corner down
    expect(triangle.containsPoint(5, 4.1)).toBe(true);
    expect(triangle.containsPoint(5, 5.9)).toBe(false);
  });
});

describe('World.bodyAt', () => {
  it('finds the topmost movable body and ignores static ones', () => {
    const world = new World();
    world.add(Body.box(10, 1, Infinity, 0, 0));
    const lower = world.add(Body.circle(1, 1, 0, 1));
    const upper = world.add(Body.circle(1, 1, 0.5, 1));
    expect(world.bodyAt(0.2, 1)).toBe(upper); // both contain it; the later one is on top
    expect(world.bodyAt(-0.9, 1)).toBe(lower);
    expect(world.bodyAt(3, 0)).toBeNull(); // only the static ground is there
    expect(world.bodyAt(50, 50)).toBeNull();
  });
});

describe('World.clear', () => {
  it('empties the world and lets it be reused from scratch', () => {
    const world = new World();
    world.add(Body.box(10, 1, Infinity, 0, -0.5));
    const a = world.add(Body.box(1, 1, 1, 0, 0.5));
    const b = world.add(Body.box(1, 1, 1, 0.5, 0.5));
    world.addJoint(new RevoluteJoint(a, b, new Vec2(0.25, 0.5)));
    run(world, 0.5);

    world.clear();
    expect(world.bodies).toHaveLength(0);
    expect(world.joints).toHaveLength(0);
    expect(world.manifoldCount).toBe(0);
    world.step(FIXED_DT); // an empty world steps fine

    // Fresh bodies get ids from zero again, and the old joint's collision filter is gone.
    const c = world.add(Body.box(1, 1, 1, 0, 0));
    const d = world.add(Body.box(1, 1, 1, 0.5, 0));
    expect([c.id, d.id]).toEqual([0, 1]);
    world.gravity.set(0, 0);
    world.detectCollisions();
    expect(world.manifoldCount).toBe(1);
  });

  it('keeps settings such as gravity and continuous collision', () => {
    const world = new World();
    world.gravity.set(1, 2);
    world.continuousCollision = false;
    world.clear();
    expect(world.gravity.x).toBe(1);
    expect(world.continuousCollision).toBe(false);
  });
});

describe('demo scenes', () => {
  it.each(scenes.map((scene) => [scene.name, scene] as const))('%s builds and simulates 3 seconds without blowing up', (_name, scene) => {
    const world = new World();
    scene.build(world);
    expect(world.bodies.length).toBeGreaterThan(0);
    run(world, 3);
    for (const body of world.bodies) {
      expect(Number.isFinite(body.position.x + body.position.y + body.angle + body.velocity.x + body.velocity.y)).toBe(true);
      // Nothing should have been flung out of the 18 x 12 m view.
      expect(Math.abs(body.position.x)).toBeLessThan(40);
      expect(body.position.y).toBeGreaterThan(-5);
      expect(body.position.y).toBeLessThan(40);
    }
  });

  it('scene names are unique', () => {
    const names = scenes.map((scene) => scene.name);
    expect(new Set(names).size).toBe(names.length);
  });

  it('the pyramid stands and falls asleep', () => {
    const world = new World();
    scenes.find((scene) => scene.name === 'Pyramid')?.build(world);
    run(world, 8);
    const movable = world.bodies.filter((body) => body.invMass !== 0);
    expect(movable.every((body) => !body.awake)).toBe(true);
    const top = movable[movable.length - 1];
    expect(top.position.y).toBeGreaterThan(1 + 0.4 + 9 * 0.8 - 0.15);
  });

  it('the bullets stop at the wall with continuous collision and tunnel without it', () => {
    const wallX = 13;
    const bulletsPastWall = (continuous: boolean) => {
      const world = new World();
      world.continuousCollision = continuous;
      scenes.find((scene) => scene.name.startsWith('Bullets'))?.build(world);
      run(world, 1);
      return world.bodies.filter((body) => body.invMass !== 0 && body.position.x > wallX).length;
    };
    expect(bulletsPastWall(true)).toBe(0);
    expect(bulletsPastWall(false)).toBeGreaterThan(0);
  });

  it('the joint showcase keeps its motor, hinge and spring alive', () => {
    const world = new World();
    scenes.find((scene) => scene.name === 'Joints')?.build(world);
    const paddle = world.bodies.find((body) => body.shape.kind === 'box' && body.mass === 2);
    run(world, 2);
    expect(paddle?.awake).toBe(true);
    expect(Math.abs(paddle?.angularVelocity ?? 0)).toBeCloseTo(1.5, 1);
  });
});
