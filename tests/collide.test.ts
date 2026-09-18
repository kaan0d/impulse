import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { collide } from '../src/engine/collide';
import { Manifold } from '../src/engine/manifold';
import { World } from '../src/engine/world';

const DIGITS = 9;

function hit(a: Body, b: Body): Manifold {
  const manifold = new Manifold(a, b);
  expect(collide(a, b, manifold)).toBe(true);
  return manifold;
}

function expectMiss(a: Body, b: Body): void {
  const manifold = new Manifold(a, b);
  expect(collide(a, b, manifold)).toBe(false);
  expect(manifold.count).toBe(0);
}

function expectNormal(manifold: Manifold, x: number, y: number): void {
  expect(manifold.normal.x).toBeCloseTo(x, DIGITS);
  expect(manifold.normal.y).toBeCloseTo(y, DIGITS);
}

function expectContact(manifold: Manifold, index: number, x: number, y: number, depth: number): void {
  expect(manifold.points[index].x).toBeCloseTo(x, DIGITS);
  expect(manifold.points[index].y).toBeCloseTo(y, DIGITS);
  expect(manifold.depths[index]).toBeCloseTo(depth, DIGITS);
}

// Contact order is an implementation detail, so compare sorted by y.
function contactsByY(manifold: Manifold): { x: number; y: number; depth: number }[] {
  const contacts = [];
  for (let i = 0; i < manifold.count; i++) {
    contacts.push({ x: manifold.points[i].x, y: manifold.points[i].y, depth: manifold.depths[i] });
  }
  return contacts.sort((p, q) => p.y - q.y);
}

describe('circle vs circle', () => {
  it('overlapping on an axis', () => {
    const manifold = hit(Body.circle(1, 1, 0, 0), Body.circle(1, 1, 1.5, 0));
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 1, 0);
    expectContact(manifold, 0, 0.75, 0, 0.5);
  });

  it('overlapping diagonally', () => {
    const manifold = hit(Body.circle(1, 1, 0, 0), Body.circle(1, 1, 1, 1));
    expectNormal(manifold, Math.SQRT1_2, Math.SQRT1_2);
    expectContact(manifold, 0, 0.5, 0.5, 2 - Math.SQRT2);
  });

  it('coincident centers still give a unit normal', () => {
    const manifold = hit(Body.circle(1, 1, 3, 3), Body.circle(1, 1, 3, 3));
    expect(manifold.normal.length()).toBeCloseTo(1, DIGITS);
    expect(manifold.depths[0]).toBeCloseTo(2, DIGITS);
  });

  it('separated circles do not collide', () => {
    expectMiss(Body.circle(1, 1, 0, 0), Body.circle(1, 1, 2.5, 0));
  });
});

describe('circle vs box', () => {
  const box = () => Body.box(2, 2, 1, 0, 0);

  it('circle against a face, box first', () => {
    const manifold = hit(box(), Body.circle(0.5, 1, 1.4, 0));
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 1, 0);
    expectContact(manifold, 0, 0.95, 0, 0.1);
  });

  it('circle first flips the normal', () => {
    const manifold = hit(Body.circle(0.5, 1, 1.4, 0), box());
    expectNormal(manifold, -1, 0);
    expectContact(manifold, 0, 0.95, 0, 0.1);
  });

  it('circle against a corner', () => {
    const manifold = hit(box(), Body.circle(0.5, 1, 1.3, 1.3));
    const depth = 0.5 - Math.hypot(0.3, 0.3);
    expectNormal(manifold, Math.SQRT1_2, Math.SQRT1_2);
    expectContact(manifold, 0, 1 - (Math.SQRT1_2 * depth) / 2, 1 - (Math.SQRT1_2 * depth) / 2, depth);
  });

  it('circle center inside the box exits through the nearest face', () => {
    const manifold = hit(box(), Body.circle(0.5, 1, 0.8, 0.1));
    expectNormal(manifold, 1, 0);
    expectContact(manifold, 0, 0.65, 0.1, 0.7);
  });

  it('rotated box', () => {
    const rotated = Body.box(4, 2, 1, 0, 0);
    rotated.angle = Math.PI / 2; // occupies x in [-1, 1], y in [-2, 2]
    const manifold = hit(rotated, Body.circle(0.5, 1, 1.4, 0));
    expectNormal(manifold, 1, 0);
    expectContact(manifold, 0, 0.95, 0, 0.1);
  });

  it('separated circle and box do not collide', () => {
    expectMiss(box(), Body.circle(0.5, 1, 2, 0));
  });
});

describe('box vs box', () => {
  it('aligned overlap gives two contacts on the shared face', () => {
    const manifold = hit(Body.box(2, 2, 1, 0, 0), Body.box(2, 2, 1, 1.8, 0));
    expect(manifold.count).toBe(2);
    expectNormal(manifold, 1, 0);
    const [low, high] = contactsByY(manifold);
    expect(low).toEqual({ x: expect.closeTo(0.9, DIGITS), y: expect.closeTo(-1, DIGITS), depth: expect.closeTo(0.2, DIGITS) });
    expect(high).toEqual({ x: expect.closeTo(0.9, DIGITS), y: expect.closeTo(1, DIGITS), depth: expect.closeTo(0.2, DIGITS) });
  });

  it('swapping the bodies flips the normal and keeps the contacts', () => {
    const forward = hit(Body.box(2, 2, 1, 0, 0), Body.box(2, 2, 1, 1.8, 0));
    const swapped = hit(Body.box(2, 2, 1, 1.8, 0), Body.box(2, 2, 1, 0, 0));
    expectNormal(swapped, -1, 0);
    const expected = contactsByY(forward);
    const actual = contactsByY(swapped);
    expect(actual).toHaveLength(expected.length);
    actual.forEach((contact, i) => {
      expect(contact.x).toBeCloseTo(expected[i].x, DIGITS);
      expect(contact.y).toBeCloseTo(expected[i].y, DIGITS);
      expect(contact.depth).toBeCloseTo(expected[i].depth, DIGITS);
    });
  });

  // A 45 degree box resting corner-down on a flat box; only the tip is below the face.
  function diamondOnGround(): { ground: Body; diamond: Body } {
    const diamond = Body.box(1, 1, 1, 0, 1.6);
    diamond.angle = Math.PI / 4;
    return { ground: Body.box(10, 2, 1, 0, 0), diamond };
  }
  const tipDepth = 1 - (1.6 - Math.SQRT1_2);
  const tipY = 1.6 - Math.SQRT1_2 + tipDepth / 2;

  it('rotated box corner gives one contact (reference face on A)', () => {
    const { ground, diamond } = diamondOnGround();
    const manifold = hit(ground, diamond);
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 0, 1);
    expectContact(manifold, 0, 0, tipY, tipDepth);
  });

  it('rotated box corner gives one contact (reference face on B)', () => {
    const { ground, diamond } = diamondOnGround();
    const manifold = hit(diamond, ground);
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 0, -1);
    expectContact(manifold, 0, 0, tipY, tipDepth);
  });

  it('separated boxes do not collide', () => {
    expectMiss(Body.box(2, 2, 1, 0, 0), Body.box(2, 2, 1, 2.5, 0));
  });

  it('overlap on A axes but separated on a rotated B axis', () => {
    const rotated = Body.box(2, 2, 1, 2.2, 1.9);
    rotated.angle = Math.PI / 4;
    expectMiss(Body.box(2, 2, 1, 0, 0), rotated);
  });
});

describe('World.detectCollisions', () => {
  it('finds overlapping pairs and skips static-static pairs', () => {
    const world = new World();
    world.add(Body.circle(1, 1, 0, 0));
    world.add(Body.circle(1, 1, 1.5, 0));
    world.add(Body.circle(1, 1, 50, 50));
    world.add(Body.box(4, 4, Infinity, 100, 0));
    world.add(Body.box(4, 4, Infinity, 101, 0));
    world.detectCollisions();
    expect(world.manifoldCount).toBe(1);
    expect(world.manifolds[0].count).toBe(1);
  });

  it('step refreshes contacts and reuses the manifold pool', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const mover = world.add(Body.circle(1, 1, 0, 0));
    world.add(Body.circle(1, 1, 5, 0));
    world.step(1 / 60);
    expect(world.manifoldCount).toBe(0);

    mover.position.set(4, 0);
    world.step(1 / 60);
    expect(world.manifoldCount).toBe(1);
    const pooled = world.manifolds[0];

    mover.position.set(0, 0);
    world.step(1 / 60);
    mover.position.set(4, 0);
    world.step(1 / 60);
    expect(world.manifolds[0]).toBe(pooled);
  });
});
