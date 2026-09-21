import { describe, expect, it } from 'vitest';
import { pile } from '../bench/scenes';
import { Body } from '../src/engine/body';
import { SUBSTEPS } from '../src/engine/solver';
import { FIXED_DT } from '../src/engine/stepper';
import { World } from '../src/engine/world';
import { groundWorld, run } from './helpers';

const G = 9.81;

describe('damping', () => {
  function drifting(linear: number, angular: number): Body {
    const world = new World();
    world.gravity.set(0, 0);
    const body = world.add(Body.box(1, 1, 1, 0, 0));
    body.velocity.set(4, 0);
    body.angularVelocity = 2;
    body.linearDamping = linear;
    body.angularDamping = angular;
    run(world, 2);
    return body;
  }

  it('scales speed by 1 / (1 + h d) every substep, close to exp(-d t)', () => {
    const body = drifting(0.5, 1);
    const factor = 1 / (1 + (FIXED_DT / SUBSTEPS) * 0.5);
    expect(body.velocity.x).toBeCloseTo(4 * factor ** (120 * SUBSTEPS), 9);
    expect(body.velocity.x).toBeCloseTo(4 * Math.exp(-1), 1);
    expect(body.angularVelocity).toBeCloseTo(2 * Math.exp(-2), 1);
  });

  it('zero damping keeps the speed exactly', () => {
    const body = drifting(0, 0);
    expect(body.velocity.x).toBe(4);
    expect(body.angularVelocity).toBe(2);
  });
});

describe('rolling resistance', () => {
  function rollingBall(resistance: number): Body {
    const world = groundWorld();
    const ball = world.add(Body.circle(0.5, 1, 0, 0.5));
    ball.velocity.set(3, 0);
    // Rolling without slipping to the right spins clockwise, at v / r.
    ball.angularVelocity = -6;
    ball.rollingResistance = resistance;
    run(world, 2);
    return ball;
  }

  it('a ball keeps rolling at its speed without resistance', () => {
    expect(rollingBall(0).velocity.x).toBeGreaterThan(2.97);
  });

  it('slows a rolling ball at (2/3) * resistance * g / r', () => {
    const resistance = 0.05;
    const expected = 3 - ((2 / 3) * resistance * G * 2) / 0.5;
    expect(rollingBall(resistance).velocity.x).toBeCloseTo(expected, 1);
  });
});

describe('big piles', () => {
  function secondsUntilAsleep(rolling: number, limit: number): number {
    const world = pile(500, rolling);
    for (let step = 0; step < limit * 60; step++) {
      world.step(FIXED_DT);
      if (world.bodies.every((body) => !body.isSimulated)) return step / 60;
    }
    return Infinity;
  }

  it('a 500-body pile falls asleep', () => {
    expect(secondsUntilAsleep(0, 15)).toBeLessThan(12);
  });

  it('rolling resistance makes the pile sleep sooner', () => {
    expect(secondsUntilAsleep(0.03, 15)).toBeLessThan(secondsUntilAsleep(0, 15));
  });
});
