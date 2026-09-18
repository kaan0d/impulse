import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { FIXED_DT } from '../src/engine/stepper';
import { World } from '../src/engine/world';

const G = 9.81;
const STEPS = 120; // 2 seconds

function fall(steps: number): Body {
  const world = new World();
  const body = world.add(Body.circle(0.5, 1, 3, 100));
  body.velocity.set(2, 0);
  body.angularVelocity = 0.5;
  for (let i = 0; i < steps; i++) world.step(FIXED_DT);
  return body;
}

describe('free fall', () => {
  it('velocity matches v = v0 + g t', () => {
    const t = STEPS * FIXED_DT;
    const body = fall(STEPS);
    expect(body.velocity.y).toBeCloseTo(-G * t, 9);
    expect(body.velocity.x).toBe(2);
  });

  it('position matches the exact semi-implicit Euler sum', () => {
    const body = fall(STEPS);
    const expectedY = 100 - G * FIXED_DT ** 2 * ((STEPS * (STEPS + 1)) / 2);
    expect(body.position.y).toBeCloseTo(expectedY, 9);
  });

  it('position matches analytic y = y0 - g t^2 / 2 within the first-order error', () => {
    const t = STEPS * FIXED_DT;
    const body = fall(STEPS);
    const analyticY = 100 - 0.5 * G * t * t;
    const maxError = 0.5 * G * t * FIXED_DT; // semi-implicit Euler is O(dt)
    expect(Math.abs(body.position.y - analyticY)).toBeLessThanOrEqual(maxError + 1e-9);
    expect(body.position.x).toBeCloseTo(3 + 2 * t, 9);
  });

  it('angle advances linearly with angular velocity', () => {
    const body = fall(STEPS);
    expect(body.angle).toBeCloseTo(0.5 * STEPS * FIXED_DT, 9);
  });

  it('static bodies do not move', () => {
    const world = new World();
    const ground = world.add(Body.box(10, 1, Infinity, 0, 0));
    for (let i = 0; i < STEPS; i++) world.step(FIXED_DT);
    expect(ground.position.y).toBe(0);
    expect(ground.velocity.y).toBe(0);
  });
});
