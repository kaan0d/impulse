import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { DistanceJoint, RevoluteJoint, type Joint } from '../src/engine/joint';
import { FIXED_DT } from '../src/engine/stepper';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';
import { run } from './helpers';

const G = 9.81;

function anchorError(joint: Joint): number {
  return joint.anchorA(new Vec2()).sub(joint.anchorB(new Vec2())).length();
}

// Static pivot at (0, 10) and a body swinging on a rod of length 3 to its right.
function pendulum(
  makeJoint: (pivot: Body, bob: Body) => Joint,
  bobAt: Vec2,
  bobRadius = 0.5,
): { world: World; bob: Body; joint: Joint } {
  const world = new World();
  const pivot = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
  const bob = world.add(Body.circle(bobRadius, 1, bobAt.x, bobAt.y));
  const joint = world.addJoint(makeJoint(pivot, bob));
  return { world, bob, joint };
}

describe('RevoluteJoint', () => {
  it('a pinned circle swings with the physical-pendulum period and never leaves its pin', () => {
    const rod = 3;
    const radius = 0.5;
    const swing = 0.1; // small amplitude keeps the linear period accurate
    const start = new Vec2(rod * Math.sin(swing), 10 - rod * Math.cos(swing));
    const { world, bob, joint } = pendulum((pivot, b) => new RevoluteJoint(pivot, b, new Vec2(0, 10)), start, radius);

    // Rigid pendulum about the pin: I = m r^2 / 2 + m d^2.
    const expectedPeriod = 2 * Math.PI * Math.sqrt((0.5 * radius ** 2 + rod ** 2) / (G * rod));
    const angleOf = () => Math.atan2(bob.position.x, 10 - bob.position.y);

    const crossings: number[] = [];
    let previous = angleOf();
    let worstError = 0;
    for (let step = 1; step <= 600; step++) {
      world.step(FIXED_DT);
      const angle = angleOf();
      if (previous > 0 && angle <= 0) crossings.push(step * FIXED_DT);
      previous = angle;
      worstError = Math.max(worstError, anchorError(joint));
    }

    expect(worstError).toBeLessThan(0.005);
    expect(crossings.length).toBeGreaterThanOrEqual(2);
    expect(crossings[1] - crossings[0]).toBeCloseTo(expectedPeriod, 1);
    expect(Math.abs(crossings[1] - crossings[0] - expectedPeriod) / expectedPeriod).toBeLessThan(0.02);
  });

  it('a hanging chain keeps every pin closed while it swings', () => {
    const world = new World();
    let previous = world.add(Body.box(1, 0.2, Infinity, 0, 10.1));
    const joints: Joint[] = [];
    for (let i = 0; i < 6; i++) {
      const link = world.add(Body.box(0.25, 0.8, 1, 0, 10 - 0.8 * i - 0.4));
      joints.push(world.addJoint(new RevoluteJoint(previous, link, new Vec2(0, 10 - 0.8 * i))));
      previous = link;
    }
    previous.velocity.set(4, 0);

    let worstError = 0;
    for (let step = 0; step < 600; step++) {
      world.step(FIXED_DT);
      worstError = Math.max(worstError, ...joints.map(anchorError));
    }
    expect(worstError).toBeLessThan(0.05);
  });

  it('a heavy box hangs from a light link without tearing the pins apart', () => {
    const world = new World();
    const bar = world.add(Body.box(1, 0.2, Infinity, 0, 10.1));
    const link = world.add(Body.box(0.25, 0.8, 1, 0, 9.6));
    const heavy = world.add(Body.box(1, 1, 20, 0, 8.7));
    const top = world.addJoint(new RevoluteJoint(bar, link, new Vec2(0, 10)));
    const bottom = world.addJoint(new RevoluteJoint(link, heavy, new Vec2(0, 9.2)));

    let worstError = 0;
    for (let step = 0; step < 600; step++) {
      world.step(FIXED_DT);
      worstError = Math.max(worstError, anchorError(top), anchorError(bottom));
    }
    expect(worstError).toBeLessThan(0.05);
    expect(Math.abs(heavy.position.y - 8.7)).toBeLessThan(0.1);
  });
});

describe('DistanceJoint', () => {
  it('keeps the rod length through a wide swing and does not gain energy', () => {
    const rod = 3;
    const { world, bob, joint } = pendulum(
      (pivot, b) => new DistanceJoint(pivot, b, new Vec2(0, 10), new Vec2(rod, 10)),
      new Vec2(rod, 10),
    );
    const energy = () => 0.5 * bob.velocity.lengthSq() + G * bob.position.y;
    const startEnergy = energy();

    let worstLengthError = 0;
    let worstEnergy = startEnergy;
    for (let step = 0; step < 600; step++) {
      world.step(FIXED_DT);
      worstLengthError = Math.max(worstLengthError, Math.abs(joint.anchorB(new Vec2()).sub(joint.anchorA(new Vec2())).length() - rod));
      worstEnergy = Math.max(worstEnergy, energy());
    }
    expect(worstLengthError).toBeLessThan(0.02);
    // Semi-implicit Euler and the joint bias add a little energy; 3% of the swing's range is generous.
    expect(worstEnergy - startEnergy).toBeLessThan(0.03 * G * rod);
  });

  it('defaults the rod length to the anchor separation', () => {
    const { world, bob } = pendulum((pivot, b) => new DistanceJoint(pivot, b, new Vec2(0, 10), new Vec2(0, 7)), new Vec2(0, 7));
    run(world, 3);
    expect(bob.position.y).toBeCloseTo(7, 2);
  });
});

describe('joints and collisions', () => {
  it('jointed bodies never collide with each other, unjointed overlapping bodies do', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const a = world.add(Body.box(1, 1, 1, 0, 0));
    const b = world.add(Body.box(1, 1, 1, 0.5, 0));
    const c = world.add(Body.box(1, 1, 1, 0, 0.5));
    world.addJoint(new RevoluteJoint(a, b, new Vec2(0.25, 0)));
    world.detectCollisions();

    const pairs = [];
    for (let k = 0; k < world.manifoldCount; k++) pairs.push([world.manifolds[k].bodyA.id, world.manifolds[k].bodyB.id]);
    expect(pairs).not.toContainEqual([a.id, b.id]);
    expect(pairs).toContainEqual([a.id, c.id]);
  });

  it('a pinned link is still pushed by bodies that hit it', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const bar = world.add(Body.box(1, 0.2, Infinity, 0, 10.1));
    const link = world.add(Body.box(0.25, 0.8, 1, 0, 9.6));
    const joint = world.addJoint(new RevoluteJoint(bar, link, new Vec2(0, 10)));
    const ball = world.add(Body.circle(0.3, 1, 1.5, 9.6));
    ball.velocity.set(-4, 0);
    run(world, 1);
    expect(link.angle).toBeLessThan(-0.05);
    expect(ball.velocity.x).toBeGreaterThan(-4);
    expect(anchorError(joint)).toBeLessThan(0.01);
  });
});

describe('joints and sleeping', () => {
  function hangingChain(): { world: World; links: Body[] } {
    const world = new World();
    let previous = world.add(Body.box(1, 0.2, Infinity, 0, 10.1));
    const links: Body[] = [];
    for (let i = 0; i < 4; i++) {
      const link = world.add(Body.box(0.25, 0.8, 1, 0, 10 - 0.8 * i - 0.4));
      world.addJoint(new RevoluteJoint(previous, link, new Vec2(0, 10 - 0.8 * i)));
      links.push(link);
      previous = link;
    }
    return { world, links };
  }

  it('a chain hanging at rest falls asleep as one island', () => {
    const { world, links } = hangingChain();
    run(world, 3);
    expect(links.every((link) => !link.awake)).toBe(true);
  });

  it('waking one link wakes the whole chain', () => {
    const { world, links } = hangingChain();
    run(world, 3);
    expect(links.every((link) => !link.awake)).toBe(true);

    world.add(Body.circle(0.3, 1, 0.5, 8.4)).velocity.set(-5, 0);
    world.step(FIXED_DT);
    world.step(FIXED_DT);
    expect(links.every((link) => link.awake)).toBe(true);
  });

  it('a swinging chain never sleeps', () => {
    const { world, links } = hangingChain();
    links[3].velocity.set(3, 0);
    run(world, 1.5);
    expect(links.some((link) => link.awake)).toBe(true);
    expect(links.every((link) => link.awake)).toBe(true);
  });
});

describe('joint determinism', () => {
  function runChain(): number[] {
    const world = new World();
    let previous = world.add(Body.box(1, 0.2, Infinity, 0, 10.1));
    for (let i = 0; i < 5; i++) {
      const link = world.add(Body.box(0.25, 0.8, 1, 0, 10 - 0.8 * i - 0.4));
      world.addJoint(new RevoluteJoint(previous, link, new Vec2(0, 10 - 0.8 * i)));
      previous = link;
    }
    previous.velocity.set(4, 1);
    run(world, 5);
    return world.bodies.flatMap((b) => [b.position.x, b.position.y, b.angle, b.velocity.x, b.velocity.y, b.angularVelocity]);
  }

  it('the same chain run twice is identical to the last bit', () => {
    const first = runChain();
    const second = runChain();
    expect(first.every((value, i) => Object.is(value, second[i]))).toBe(true);
  });
});
