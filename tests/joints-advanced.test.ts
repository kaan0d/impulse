import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { DistanceJoint, MouseJoint, RevoluteJoint } from '../src/engine/joint';
import { FIXED_DT } from '../src/engine/stepper';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';
import { run } from './helpers';

const G = 9.81;

// A 2 m arm pinned at its left end to a fixed point at (0, 10); joint angle 0 means horizontal.
function pinnedArm(mass = 1): { world: World; arm: Body; hinge: RevoluteJoint } {
  const world = new World();
  const pivot = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
  const arm = world.add(Body.box(2, 0.2, mass, 1, 10));
  const hinge = world.addJoint(new RevoluteJoint(pivot, arm, new Vec2(0, 10))) as RevoluteJoint;
  return { world, arm, hinge };
}

// Lowest and highest joint angle seen over `seconds` of simulation.
function angleRange(world: World, hinge: RevoluteJoint, seconds: number): { min: number; max: number } {
  const range = { min: hinge.angle, max: hinge.angle };
  for (let step = 0; step < Math.round(seconds / FIXED_DT); step++) {
    world.step(FIXED_DT);
    range.min = Math.min(range.min, hinge.angle);
    range.max = Math.max(range.max, hinge.angle);
  }
  return range;
}

describe('revolute limits', () => {
  it('without limits the arm swings well past the angle a limit would stop it at', () => {
    const { world, hinge } = pinnedArm();
    expect(angleRange(world, hinge, 2).min).toBeLessThan(-1.4);
  });

  it('a lower limit holds the falling arm', () => {
    const { world, hinge } = pinnedArm();
    hinge.setLimits(-1, 0.3);
    const range = angleRange(world, hinge, 4);
    expect(range.min).toBeGreaterThan(-1.05);
    expect(hinge.angle).toBeCloseTo(-1, 1);
  });

  it('an upper limit stops an arm thrown upward', () => {
    const { world, arm, hinge } = pinnedArm();
    hinge.setLimits(-1, 0.3);
    arm.angularVelocity = 6;
    expect(angleRange(world, hinge, 1).max).toBeLessThan(0.35);
  });

  it('clearing the limits frees the arm again', () => {
    const { world, hinge } = pinnedArm();
    hinge.setLimits(-1, 0.3);
    hinge.clearLimits();
    expect(angleRange(world, hinge, 2).min).toBeLessThan(-1.4);
  });

  it('rejects inverted limits', () => {
    expect(() => pinnedArm().hinge.setLimits(1, -1)).toThrow(RangeError);
  });

  it('the pin stays closed while the limit is hit', () => {
    const { world, arm, hinge } = pinnedArm();
    hinge.setLimits(-0.5, 0.5);
    arm.angularVelocity = 8;
    let worst = 0;
    for (let step = 0; step < 240; step++) {
      world.step(FIXED_DT);
      worst = Math.max(worst, hinge.anchorA(new Vec2()).sub(hinge.anchorB(new Vec2())).length());
    }
    expect(worst).toBeLessThan(0.02);
  });
});

describe('revolute motor', () => {
  function wheel(maxTorque: number, speed: number): { world: World; wheel: Body } {
    const world = new World();
    world.gravity.set(0, 0);
    const axle = world.add(Body.box(0.2, 0.2, Infinity, 0, 5));
    const disc = world.add(Body.circle(0.5, 2, 0, 5));
    const joint = world.addJoint(new RevoluteJoint(axle, disc, new Vec2(0, 5))) as RevoluteJoint;
    joint.setMotor(speed, maxTorque);
    return { world, wheel: disc };
  }

  it('spins up to the target speed when torque is plentiful', () => {
    const { world, wheel: disc } = wheel(100, 3);
    run(world, 1);
    expect(disc.angularVelocity).toBeCloseTo(3, 3);
  });

  it('accelerates at torque / inertia when torque is the limit', () => {
    // I = m r^2 / 2 = 0.25, so 1 N m gives 4 rad/s^2.
    const { world, wheel: disc } = wheel(1, 10);
    run(world, 0.5);
    expect(disc.angularVelocity).toBeCloseTo(2, 1);
  });

  it('runs in reverse', () => {
    const { world, wheel: disc } = wheel(100, -4);
    run(world, 1);
    expect(disc.angularVelocity).toBeCloseTo(-4, 3);
  });

  it('holds an arm against gravity only if it has the torque', () => {
    const strong = pinnedArm();
    strong.hinge.setMotor(0, 50);
    run(strong.world, 2);
    expect(Math.abs(strong.hinge.angle)).toBeLessThan(0.05);

    const weak = pinnedArm();
    weak.hinge.setMotor(0, 1);
    run(weak.world, 2);
    expect(weak.hinge.angle).toBeLessThan(-0.5);
  });

  it('clearing the motor stops driving the wheel', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const axle = world.add(Body.box(0.2, 0.2, Infinity, 0, 5));
    const disc = world.add(Body.circle(0.5, 2, 0, 5));
    const joint = world.addJoint(new RevoluteJoint(axle, disc, new Vec2(0, 5))) as RevoluteJoint;
    joint.setMotor(3, 100);
    run(world, 0.5);
    joint.clearMotor();
    disc.angularVelocity = 0;
    run(world, 0.5);
    expect(disc.angularVelocity).toBeCloseTo(0, 6);
  });
});

describe('distance joint spring', () => {
  const restLength = 2;

  function hangingBob(frequency: number, damping: number): { world: World; bob: Body } {
    const world = new World();
    const anchor = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
    const bob = world.add(Body.circle(0.3, 1, 0, 10 - restLength));
    const joint = world.addJoint(new DistanceJoint(anchor, bob, new Vec2(0, 10), new Vec2(0, 10 - restLength))) as DistanceJoint;
    joint.setSpring(frequency, damping);
    return { world, bob };
  }

  it('sags to the static deflection g / omega^2', () => {
    const { world, bob } = hangingBob(1, 0.6);
    run(world, 10);
    const stretch = G / (2 * Math.PI) ** 2;
    expect(bob.position.y).toBeCloseTo(10 - restLength - stretch, 2);
  });

  it('an undamped spring oscillates at the requested frequency', () => {
    const { world, bob } = hangingBob(1, 0);
    const equilibrium = 10 - restLength - G / (2 * Math.PI) ** 2;
    const crossings: number[] = [];
    let previous = bob.position.y - equilibrium;
    for (let step = 1; step <= 600; step++) {
      world.step(FIXED_DT);
      const offset = bob.position.y - equilibrium;
      if (previous < 0 && offset >= 0) crossings.push(step * FIXED_DT);
      previous = offset;
    }
    expect(crossings.length).toBeGreaterThanOrEqual(3);
    for (let i = 1; i < crossings.length; i++) expect(crossings[i] - crossings[i - 1]).toBeCloseTo(1, 1);
  });

  it('a stiffer spring stretches less', () => {
    const soft = hangingBob(0.8, 0.7);
    const stiff = hangingBob(2, 0.7);
    run(soft.world, 10);
    run(stiff.world, 10);
    expect(stiff.bob.position.y).toBeGreaterThan(soft.bob.position.y);
  });

  it('clearing the spring makes it a rigid rod again', () => {
    const world = new World();
    const anchor = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
    const bob = world.add(Body.circle(0.3, 1, 0, 8));
    const joint = world.addJoint(new DistanceJoint(anchor, bob, new Vec2(0, 10), new Vec2(0, 8))) as DistanceJoint;
    joint.setSpring(1, 0.5);
    joint.clearSpring();
    run(world, 4);
    expect(bob.position.y).toBeCloseTo(8, 2);
  });
});

describe('distance joint rope', () => {
  function ropeWorld(): { world: World; bob: Body } {
    const world = new World();
    const anchor = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
    const bob = world.add(Body.circle(0.3, 1, 0, 9)); // 1 m below the pivot, on a 2 m rope
    const rope = new DistanceJoint(anchor, bob, new Vec2(0, 10), new Vec2(0, 9), 2);
    rope.setRope();
    world.addJoint(rope);
    return { world, bob };
  }

  it('lets the bob fall freely while the rope is slack', () => {
    const { world, bob } = ropeWorld();
    run(world, 0.3);
    const t = Math.round(0.3 / FIXED_DT) * FIXED_DT;
    // Semi-implicit Euler drops slightly more than the continuous 1/2 g t^2.
    expect(bob.position.y).toBeCloseTo(9 - 0.5 * G * t * t, 1);
  });

  it('catches the bob at full length and never lets it past', () => {
    const { world, bob } = ropeWorld();
    let farthest = 0;
    for (let step = 0; step < 300; step++) {
      world.step(FIXED_DT);
      farthest = Math.max(farthest, Math.hypot(bob.position.x, 10 - bob.position.y));
    }
    expect(farthest).toBeLessThan(2.06);
    expect(bob.position.y).toBeGreaterThan(7.9);
  });

  it('a rope goes slack again when the anchors move closer', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const anchor = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
    const bob = world.add(Body.circle(0.3, 1, 0, 8));
    const rope = new DistanceJoint(anchor, bob, new Vec2(0, 10), new Vec2(0, 8), 2);
    rope.setRope();
    world.addJoint(rope);
    bob.velocity.set(0, 3); // moving toward the pivot: allowed
    run(world, 0.2);
    expect(bob.position.y).toBeGreaterThan(8.5);
  });
});

describe('mouse joint', () => {
  function grab(maxForce: number): { world: World; body: Body; cursor: Body; joint: MouseJoint } {
    const world = new World();
    world.gravity.set(0, 0);
    const cursor = world.add(Body.circle(0.01, Infinity, 2, 5));
    cursor.collidable = false;
    const body = world.add(Body.circle(0.5, 1, 2, 5));
    const joint = world.addJoint(new MouseJoint(cursor, body, new Vec2(2, 5), maxForce)) as MouseJoint;
    return { world, body, cursor, joint };
  }

  it('pulls the body to the target', () => {
    const { world, body, joint } = grab(1000);
    joint.setTarget(6, 8);
    run(world, 3);
    expect(body.position.x).toBeCloseTo(6, 1);
    expect(body.position.y).toBeCloseTo(8, 1);
    expect(body.velocity.length()).toBeLessThan(0.1);
  });

  it('caps the pull at the maximum force', () => {
    const { world, body, joint } = grab(0.1);
    joint.setTarget(6, 5);
    run(world, 0.5);
    // At most F t^2 / (2 m) = 0.0125 m.
    expect(body.position.x - 2).toBeLessThan(0.02);
    expect(body.position.x - 2).toBeGreaterThan(0);
  });

  it('a target move wakes a sleeping body', () => {
    const { world, body, joint } = grab(1000);
    run(world, 1);
    expect(body.awake).toBe(false);
    joint.setTarget(3, 5);
    expect(body.awake).toBe(true);
  });

  it('a non-collidable cursor does not push bodies it overlaps', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const cursor = world.add(Body.circle(0.3, Infinity, 0, 0));
    cursor.collidable = false;
    const bystander = world.add(Body.circle(0.5, 1, 0.1, 0));
    run(world, 1);
    expect(bystander.position.x).toBe(0.1);
    expect(world.manifoldCount).toBe(0);
  });

  it('can drag a body that is resting on the ground', () => {
    const world = new World();
    world.add(Body.box(40, 1, Infinity, 0, -0.5));
    const cursor = world.add(Body.circle(0.01, Infinity, 0, 0.5));
    cursor.collidable = false;
    const box = world.add(Body.box(1, 1, 1, 0, 0.5));
    const joint = world.addJoint(new MouseJoint(cursor, box, new Vec2(0, 0.5), 200)) as MouseJoint;
    joint.setTarget(4, 0.5);
    run(world, 3);
    expect(box.position.x).toBeGreaterThan(3.5);
    expect(box.position.y).toBeLessThan(0.8);
  });
});

describe('removing joints', () => {
  it('a removed joint no longer holds its bodies', () => {
    const world = new World();
    const anchor = world.add(Body.box(0.2, 0.2, Infinity, 0, 10));
    const bob = world.add(Body.circle(0.3, 1, 0, 8));
    const joint = world.addJoint(new DistanceJoint(anchor, bob, new Vec2(0, 10), new Vec2(0, 8)));
    run(world, 1);
    expect(bob.position.y).toBeCloseTo(8, 1);
    world.removeJoint(joint);
    run(world, 1);
    expect(bob.position.y).toBeLessThan(6);
    expect(world.joints).toHaveLength(0);
  });

  it('bodies collide again once the joint between them is gone, but not while another joint remains', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const a = world.add(Body.box(1, 1, 1, 0, 0));
    const b = world.add(Body.box(1, 1, 1, 0.5, 0));
    const first = world.addJoint(new RevoluteJoint(a, b, new Vec2(0.25, 0)));
    const second = world.addJoint(new RevoluteJoint(a, b, new Vec2(0.25, 0.1)));
    world.detectCollisions();
    expect(world.manifoldCount).toBe(0);

    world.removeJoint(first);
    world.detectCollisions();
    expect(world.manifoldCount).toBe(0);

    world.removeJoint(second);
    world.detectCollisions();
    expect(world.manifoldCount).toBe(1);
  });

  it('removing a joint that is not in the world does nothing', () => {
    const world = new World();
    const a = world.add(Body.box(1, 1, 1, 0, 0));
    const b = world.add(Body.box(1, 1, 1, 3, 0));
    const stray = new RevoluteJoint(a, b, new Vec2(1, 0));
    expect(() => world.removeJoint(stray)).not.toThrow();
  });
});

describe('advanced joint determinism', () => {
  function runScene(): number[] {
    const world = new World();
    const bar = world.add(Body.box(1, 0.2, Infinity, 0, 10));
    const arm = world.add(Body.box(2, 0.2, 1, 1, 10));
    const hinge = world.addJoint(new RevoluteJoint(bar, arm, new Vec2(0, 10))) as RevoluteJoint;
    hinge.setLimits(-1, 0.5);
    hinge.setMotor(2, 5);
    const spring = new DistanceJoint(bar, world.add(Body.circle(0.3, 1, 3, 9)), new Vec2(0, 10), new Vec2(3, 9));
    spring.setSpring(2, 0.3);
    world.addJoint(spring);
    const rope = new DistanceJoint(bar, world.add(Body.circle(0.3, 1, -1, 9)), new Vec2(0, 10), new Vec2(-1, 9), 2.5);
    rope.setRope();
    world.addJoint(rope);
    run(world, 5);
    return world.bodies.flatMap((body) => [body.position.x, body.position.y, body.angle, body.velocity.x, body.velocity.y, body.angularVelocity]);
  }

  it('limits, motor, spring and rope together run identically twice', () => {
    const first = runScene();
    const second = runScene();
    expect(first.every((value, i) => Object.is(value, second[i]))).toBe(true);
  });
});
