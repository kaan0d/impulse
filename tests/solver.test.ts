import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { SUBSTEPS } from '../src/engine/solver';
import { FIXED_DT } from '../src/engine/stepper';
import { World } from '../src/engine/world';
import { groundWorld, run } from './helpers';

const G = 9.81;

function kineticEnergy(bodies: Body[]): number {
  return bodies.reduce((sum, b) => sum + 0.5 * b.mass * b.velocity.lengthSq() + 0.5 * b.inertia * b.angularVelocity ** 2, 0);
}

describe('resting contact', () => {
  it('a dropped box comes to rest on the ground', () => {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 3));
    run(world, 3);
    expect(Math.abs(box.velocity.y)).toBeLessThan(0.01);
    expect(Math.abs(box.velocity.x)).toBeLessThan(0.01);
    expect(Math.abs(box.angularVelocity)).toBeLessThan(0.01);
    const penetration = 0 - (box.position.y - 0.5);
    expect(penetration).toBeLessThan(0.01);
    expect(penetration).toBeGreaterThan(-0.01);
  });

  it('a tilted box lands, settles flat and stops', () => {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 3));
    box.angle = 0.3;
    run(world, 6);
    expect(box.velocity.length()).toBeLessThan(0.01);
    expect(Math.abs(box.angularVelocity)).toBeLessThan(0.01);
    expect(Math.abs(Math.sin(2 * box.angle))).toBeLessThan(0.02); // flat means angle is a multiple of pi/2
    expect(0 - (box.position.y - 0.5)).toBeLessThan(0.01);
  });

  it('a circle comes to rest on the ground', () => {
    const world = groundWorld();
    const ball = world.add(Body.circle(0.5, 1, 0, 3));
    run(world, 3);
    expect(ball.velocity.length()).toBeLessThan(0.01);
    expect(0 - (ball.position.y - 0.5)).toBeLessThan(0.01);
  });
});

describe('warm starting', () => {
  it('resting impulses equal the weight per substep and persist in one pooled manifold', () => {
    const world = groundWorld();
    world.add(Body.box(1, 1, 2, 0, 0.5));
    run(world, 0.2);
    const manifold = world.manifolds[0];
    const supportImpulse = () => manifold.normalImpulses[0] + manifold.normalImpulses[1];
    expect(supportImpulse()).toBeCloseTo((2 * G * FIXED_DT) / SUBSTEPS, 3);

    run(world, 0.1);
    expect(world.manifolds[0]).toBe(manifold);
    expect(supportImpulse()).toBeCloseTo((2 * G * FIXED_DT) / SUBSTEPS, 3);
  });
});

describe('two-point contacts', () => {
  it('splits the weight of a centred box evenly over its two contact points', () => {
    const world = groundWorld();
    world.add(Body.box(1, 1, 2, 0, 0.5));
    run(world, 0.2);
    const manifold = world.manifolds[0];
    expect(manifold.count).toBe(2);
    const [left, right] = manifold.normalImpulses;
    expect(left).toBeCloseTo(right, 6);
    expect(left + right).toBeCloseTo((2 * G * FIXED_DT) / SUBSTEPS, 3);
  });

  it('never applies a pulling normal impulse', () => {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 2));
    box.angle = 0.3;
    for (let step = 0; step < 240; step++) {
      world.step(FIXED_DT);
      for (let m = 0; m < world.manifoldCount; m++) {
        for (const impulse of world.manifolds[m].normalImpulses) expect(impulse).toBeGreaterThanOrEqual(0);
      }
    }
  });
});

describe('restitution', () => {
  function headOn(e: number, massB: number): { a: Body; b: Body; world: World } {
    const world = new World();
    world.gravity.set(0, 0);
    const a = world.add(Body.circle(0.5, 1, -3, 0));
    const b = world.add(Body.circle(0.5, massB, 3, 0));
    a.velocity.set(2, 0);
    b.velocity.set(-2, 0);
    a.restitution = e;
    b.restitution = e;
    return { a, b, world };
  }

  it('elastic collision conserves kinetic energy and momentum (equal masses swap velocity)', () => {
    const { a, b, world } = headOn(1, 1);
    run(world, 4);
    expect(a.velocity.x).toBeCloseTo(-2, 9);
    expect(b.velocity.x).toBeCloseTo(2, 9);
    expect(kineticEnergy([a, b])).toBeCloseTo(4, 9);
  });

  it('elastic collision conserves energy and momentum with unequal masses', () => {
    const { a, b, world } = headOn(1, 3);
    const energy = kineticEnergy([a, b]);
    const momentum = a.mass * a.velocity.x + b.mass * b.velocity.x;
    run(world, 4);
    expect(kineticEnergy([a, b])).toBeCloseTo(energy, 9);
    expect(a.mass * a.velocity.x + b.mass * b.velocity.x).toBeCloseTo(momentum, 9);
  });

  it('separation speed equals e times approach speed', () => {
    const { a, b, world } = headOn(0.5, 1);
    run(world, 4);
    expect(b.velocity.x - a.velocity.x).toBeCloseTo(0.5 * 4, 9);
  });

  it('a bouncy ball rebounds to about its drop height', () => {
    const world = groundWorld();
    const ball = world.add(Body.circle(0.5, 1, 0, 5));
    ball.restitution = 1;
    let peak = 0;
    let bounced = false;
    for (let i = 0; i < 300 && !(bounced && ball.velocity.y < 0); i++) {
      const before = ball.velocity.y;
      world.step(FIXED_DT);
      bounced ||= before < 0 && ball.velocity.y > 0;
      if (bounced) peak = Math.max(peak, ball.position.y);
    }
    // Restitution reads the speed before gravity is added, so the rebound does not gain g*dt.
    const dropHeight = 4.5;
    expect(bounced).toBe(true);
    expect(peak - 0.5).toBeGreaterThan(dropHeight * 0.98);
    expect(peak - 0.5).toBeLessThan(dropHeight * 1.02);
  });
});

describe('friction', () => {
  function slidingBox(friction: number): Body {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 0.5));
    box.friction = friction;
    box.velocity.set(5, 0);
    run(world, 0.5);
    return box;
  }

  it('frictionless box keeps its speed', () => {
    expect(slidingBox(0).velocity.x).toBeCloseTo(5, 6);
  });

  it('sliding box decelerates at mu * g', () => {
    const mu = 0.5;
    const expected = 5 - mu * G * 0.5;
    expect(Math.abs(slidingBox(mu).velocity.x - expected)).toBeLessThan(0.1);
  });

  it('friction stops the box and it stays stopped', () => {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 0.5));
    box.velocity.set(2, 0);
    run(world, 2);
    expect(Math.abs(box.velocity.x)).toBeLessThan(0.01);
  });
});

describe('static bodies', () => {
  it('ground does not move under impacts', () => {
    const world = groundWorld();
    const ground = world.bodies[0];
    world.add(Body.box(1, 1, 5, 0, 3));
    run(world, 2);
    expect(ground.position.y).toBe(-0.5);
    expect(ground.velocity.length()).toBe(0);
  });
});
