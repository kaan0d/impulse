import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { Vec2 } from '../src/engine/vec2';

describe('Vec2', () => {
  it('dot, cross and length', () => {
    const a = new Vec2(3, 4);
    const b = new Vec2(-4, 3);
    expect(a.dot(b)).toBe(0);
    expect(a.cross(b)).toBe(25);
    expect(a.length()).toBe(5);
  });

  it('mutating methods chain and change the receiver', () => {
    const v = new Vec2(1, 1).add(new Vec2(1, 2)).scale(2).addScaled(new Vec2(1, 0), 3).sub(new Vec2(1, 1));
    expect(v).toEqual(new Vec2(6, 5));
  });
});

describe('Body', () => {
  it('computes circle and box inertia', () => {
    expect(Body.circle(2, 3, 0, 0).inertia).toBe(0.5 * 3 * 4);
    expect(Body.box(2, 4, 6, 0, 0).inertia).toBe((6 * (4 + 16)) / 12);
  });

  it('infinite mass is static', () => {
    const ground = Body.box(10, 1, Infinity, 0, 0);
    expect(ground.invMass).toBe(0);
    expect(ground.invInertia).toBe(0);
  });

  it('rejects non-positive mass', () => {
    expect(() => Body.circle(1, 0, 0, 0)).toThrow(RangeError);
    expect(() => Body.circle(1, NaN, 0, 0)).toThrow(RangeError);
  });
});
