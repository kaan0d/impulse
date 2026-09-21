import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { collide } from '../src/engine/collide';
import { CONTACT_MARGIN, Manifold } from '../src/engine/manifold';
import { Vec2 } from '../src/engine/vec2';
import { World } from '../src/engine/world';
import { groundWorld, run } from './helpers';

const DIGITS = 9;

function points(...coordinates: [number, number][]): Vec2[] {
  return coordinates.map(([x, y]) => new Vec2(x, y));
}

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

function expectContact(manifold: Manifold, index: number, x: number, y: number, depth: number): void {
  expect(manifold.points[index].x).toBeCloseTo(x, DIGITS);
  expect(manifold.points[index].y).toBeCloseTo(y, DIGITS);
  expect(manifold.depths[index]).toBeCloseTo(depth, DIGITS);
}

function expectNormal(manifold: Manifold, x: number, y: number): void {
  expect(manifold.normal.x).toBeCloseTo(x, DIGITS);
  expect(manifold.normal.y).toBeCloseTo(y, DIGITS);
}

// Seeded generator so fuzz cases are reproducible.
function makeRandom(seed: number): () => number {
  let state = seed;
  return () => (state = (state * 1664525 + 1013904223) >>> 0) / 4294967296;
}

describe('polygon construction', () => {
  it('recentres on the centroid and keeps the requested position', () => {
    const body = Body.polygon(points([0, 0], [3, 0], [0, 3]), 1, 5, 7);
    expect(body.position.x).toBe(5);
    expect(body.position.y).toBe(7);
    if (body.shape.kind !== 'polygon') throw new Error('expected polygon');
    const { vertices } = body.shape;
    expect(vertices.map((v) => [v.x, v.y])).toEqual([
      [-1, -1],
      [2, -1],
      [-1, 2],
    ]);
  });

  it('inertia matches the closed forms', () => {
    const square = Body.polygon(points([0, 0], [2, 0], [2, 2], [0, 2]), 3, 0, 0);
    expect(square.inertia).toBeCloseTo((3 * (4 + 4)) / 12, DIGITS);
    // Equilateral triangle with circumradius r: I = m r^2 / 4. Regular hexagon with side r: I = 5 m r^2 / 12.
    expect(Body.regularPolygon(3, 1, 2, 0, 0).inertia).toBeCloseTo(0.5, DIGITS);
    expect(Body.regularPolygon(6, 1, 1, 0, 0).inertia).toBeCloseTo(5 / 12, DIGITS);
  });

  it('clockwise input is reordered and normals point outward', () => {
    const body = Body.polygon(points([0, 0], [0, 2], [2, 2], [2, 0]), 1, 0, 0);
    if (body.shape.kind !== 'polygon') throw new Error('expected polygon');
    const { vertices, normals } = body.shape;
    normals.forEach((normal, i) => {
      expect(Math.hypot(normal.x, normal.y)).toBeCloseTo(1, DIGITS);
      expect(normal.dot(vertices[i])).toBeGreaterThan(0);
    });
  });

  it('rejects bad outlines', () => {
    expect(() => Body.polygon(points([0, 0], [1, 0]), 1, 0, 0)).toThrow(RangeError);
    expect(() => Body.polygon(Array.from({ length: 17 }, (_, i) => new Vec2(Math.cos(i), Math.sin(i))), 1, 0, 0)).toThrow(RangeError);
    expect(() => Body.polygon(points([0, 0], [1, 0], [2, 0]), 1, 0, 0)).toThrow(RangeError);
    // An arrowhead has a reflex vertex.
    expect(() => Body.polygon(points([0, 0], [2, 1], [0, 2], [0.5, 1]), 1, 0, 0)).toThrow(RangeError);
  });
});

describe('polygon collision', () => {
  it('a square polygon against a box gives the same contacts as box against box', () => {
    const square = Body.polygon(points([-1, -1], [1, -1], [1, 1], [-1, 1]), 1, 0, 0);
    const manifold = hit(square, Body.box(2, 2, 1, 1.8, 0));
    expect(manifold.count).toBe(2);
    expectNormal(manifold, 1, 0);
    const byY = [0, 1].sort((i, j) => manifold.points[i].y - manifold.points[j].y);
    expectContact(manifold, byY[0], 0.9, -1, 0.2);
    expectContact(manifold, byY[1], 0.9, 1, 0.2);
  });

  it('a triangle lying on a box gives two contacts along its flat side', () => {
    const r = 1;
    const ground = Body.box(10, 2, Infinity, 0, -1);
    const triangle = Body.regularPolygon(3, r, 1, 0, r / 2 - 0.1);
    const manifold = hit(ground, triangle);
    expect(manifold.count).toBe(2);
    expectNormal(manifold, 0, 1);
    const byX = [0, 1].sort((i, j) => manifold.points[i].x - manifold.points[j].x);
    expectContact(manifold, byX[0], (-r * Math.sqrt(3)) / 2, -0.05, 0.1);
    expectContact(manifold, byX[1], (r * Math.sqrt(3)) / 2, -0.05, 0.1);
  });

  it('a triangle standing on its corner gives one contact', () => {
    const ground = Body.box(10, 2, Infinity, 0, -1);
    const triangle = Body.regularPolygon(3, 1, 1, 0, 1 - 0.1);
    triangle.angle = Math.PI;
    const manifold = hit(ground, triangle);
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 0, 1);
    expectContact(manifold, 0, 0, -0.05, 0.1);
  });

  it('swapping the bodies flips the normal', () => {
    const ground = Body.box(10, 2, Infinity, 0, -1);
    const triangle = Body.regularPolygon(3, 1, 1, 0, 0.4);
    expectNormal(hit(ground, triangle), 0, 1);
    expectNormal(hit(triangle, ground), 0, -1);
  });

  it('separated polygons do not collide', () => {
    expectMiss(Body.regularPolygon(5, 1, 1, 0, 0), Body.regularPolygon(5, 1, 1, 2.5, 0));
    expectMiss(Body.regularPolygon(4, 1, 1, 0, 0), Body.box(1, 1, 1, 3, 0));
  });

  it('feature ids are distinct and survive small motion', () => {
    const ground = Body.box(10, 2, Infinity, 0, -1);
    const triangle = Body.regularPolygon(3, 1, 1, 0, 0.4);
    const before = hit(ground, triangle);
    const idsBefore = [before.ids[0], before.ids[1]];
    expect(before.count).toBe(2);
    expect(idsBefore[0]).not.toBe(idsBefore[1]);

    triangle.position.set(0.03, 0.39);
    triangle.angle = 0.01;
    const after = hit(ground, triangle);
    expect([after.ids[0], after.ids[1]].sort()).toEqual([...idsBefore].sort());
  });
});

describe('polygon against circle', () => {
  const hexagon = () => Body.regularPolygon(6, 1, 1, 0, 0); // corner 0 points up, vertical edges at x = +-sqrt(3)/2

  it('circle against a corner', () => {
    const manifold = hit(hexagon(), Body.circle(0.5, 1, 0, 1.4));
    expect(manifold.count).toBe(1);
    expectNormal(manifold, 0, 1);
    expectContact(manifold, 0, 0, 0.95, 0.1);
  });

  it('circle against a face', () => {
    const face = Math.sqrt(3) / 2;
    const manifold = hit(hexagon(), Body.circle(0.5, 1, face + 0.4, 0));
    expectNormal(manifold, 1, 0);
    expectContact(manifold, 0, face - 0.05, 0, 0.1);
  });

  it('circle center inside exits through the nearest face', () => {
    const manifold = hit(hexagon(), Body.circle(0.2, 1, 0.3, 0.1));
    const face = Math.sqrt(3) / 2;
    expectNormal(manifold, 1, 0);
    // The center is 0.3 from the axis, so it sits face - 0.3 inside; depth is that plus the radius.
    expectContact(manifold, 0, face - (face - 0.1) / 2, 0.1, face - 0.1);
  });

  it('circle first flips the normal', () => {
    const manifold = hit(Body.circle(0.5, 1, 0, 1.4), hexagon());
    expectNormal(manifold, 0, -1);
  });

  it('separated circle and polygon do not collide', () => {
    expectMiss(hexagon(), Body.circle(0.5, 1, 3, 0));
    expectMiss(hexagon(), Body.circle(0.5, 1, 0, 1.6));
  });

  it('depth matches an independent distance computation on random pairs', () => {
    const random = makeRandom(3);
    let checked = 0;
    for (let i = 0; i < 3000; i++) {
      const polygon = Body.regularPolygon(3 + Math.floor(random() * 6), 0.5 + random(), 1, 0, 0);
      polygon.angle = random() * Math.PI * 2;
      const circle = Body.circle(0.2 + random() * 0.5, 1, (random() - 0.5) * 4, (random() - 0.5) * 4);
      if (circle.shape.kind !== 'circle') throw new Error('expected circle');

      const distance = signedDistanceToPolygon(polygon, circle.position);
      const manifold = new Manifold(polygon, circle);
      const collided = collide(polygon, circle, manifold);
      // Skip the razor-thin boundary where either answer is legitimate.
      const reach = circle.shape.radius + CONTACT_MARGIN;
      if (Math.abs(distance - reach) < 1e-9) continue;
      expect(collided, `case ${i}`).toBe(distance <= reach);
      if (!collided) continue;
      expect(manifold.depths[0], `case ${i} depth`).toBeCloseTo(circle.shape.radius - distance, 9);
      checked++;
    }
    expect(checked).toBeGreaterThan(300);
  });
});

// Distance from a point to a polygon body's outline; negative when the point is inside.
function signedDistanceToPolygon(body: Body, point: Vec2): number {
  if (body.shape.kind !== 'polygon') throw new Error('expected polygon');
  const cos = Math.cos(body.angle);
  const sin = Math.sin(body.angle);
  const world = body.shape.vertices.map((v) => new Vec2(body.position.x + cos * v.x - sin * v.y, body.position.y + sin * v.x + cos * v.y));
  let nearest = Infinity;
  let inside = true;
  for (let i = 0; i < world.length; i++) {
    const a = world[i];
    const b = world[(i + 1) % world.length];
    const edge = b.clone().sub(a);
    const toPoint = point.clone().sub(a);
    const t = Math.max(0, Math.min(1, toPoint.dot(edge) / edge.dot(edge)));
    nearest = Math.min(nearest, toPoint.clone().sub(edge.clone().scale(t)).length());
    if (edge.cross(toPoint) < 0) inside = false;
  }
  return inside ? -nearest : nearest;
}

describe('polygon fuzz', () => {
  function randomBox(random: () => number): Body {
    const box = Body.box(0.4 + random() * 1.6, 0.4 + random() * 1.6, 1, (random() - 0.5) * 4, (random() - 0.5) * 4);
    box.angle = random() * Math.PI * 2;
    return box;
  }

  function asPolygon(box: Body): Body {
    if (box.shape.kind !== 'box') throw new Error('expected box');
    const hx = box.shape.width / 2;
    const hy = box.shape.height / 2;
    const polygon = Body.polygon(points([-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]), 1, box.position.x, box.position.y);
    polygon.angle = box.angle;
    return polygon;
  }

  it('boxes as polygons overlap exactly when the box-box path says they do', () => {
    const random = makeRandom(11);
    let hits = 0;
    for (let i = 0; i < 4000; i++) {
      const a = randomBox(random);
      const b = randomBox(random);
      const viaBoxes = collide(a, b, new Manifold(a, b));
      const pa = asPolygon(a);
      const pb = asPolygon(b);
      const viaPolygons = collide(pa, pb, new Manifold(pa, pb));
      const mixed = collide(a, pb, new Manifold(a, pb));
      expect(viaPolygons, `case ${i} polygons`).toBe(viaBoxes);
      expect(mixed, `case ${i} mixed`).toBe(viaBoxes);
      if (viaBoxes) hits++;
    }
    expect(hits).toBeGreaterThan(300);
  });

  it('random polygon pairs give finite unit-normal manifolds', () => {
    const random = makeRandom(5);
    let hits = 0;
    for (let i = 0; i < 4000; i++) {
      const a = Body.regularPolygon(3 + Math.floor(random() * 6), 0.4 + random(), 1, (random() - 0.5) * 4, (random() - 0.5) * 4);
      const b = Body.regularPolygon(3 + Math.floor(random() * 6), 0.4 + random(), 1, (random() - 0.5) * 4, (random() - 0.5) * 4);
      a.angle = random() * 6.28;
      b.angle = random() * 6.28;
      const manifold = new Manifold(a, b);
      if (!collide(a, b, manifold)) continue;
      hits++;
      expect(Math.abs(manifold.normal.length() - 1)).toBeLessThan(1e-9);
      for (let k = 0; k < manifold.count; k++) {
        expect(Number.isFinite(manifold.points[k].x + manifold.points[k].y + manifold.depths[k])).toBe(true);
        expect(manifold.depths[k]).toBeGreaterThanOrEqual(-CONTACT_MARGIN);
      }
    }
    expect(hits).toBeGreaterThan(300);
  });
});

describe('polygon dynamics', () => {
  // Lowest points of a polygon body in world space, to check it rests flat.
  function lowestVertexHeights(body: Body): number[] {
    if (body.shape.kind !== 'polygon') throw new Error('expected polygon');
    const cos = Math.cos(body.angle);
    const sin = Math.sin(body.angle);
    return body.shape.vertices.map((v) => body.position.y + sin * v.x + cos * v.y).sort((p, q) => p - q);
  }

  it.each([
    ['triangle', () => Body.regularPolygon(3, 0.7, 1, 0, 3)],
    ['pentagon', () => Body.regularPolygon(5, 0.6, 1, 0, 3)],
    ['hexagon', () => Body.regularPolygon(6, 0.6, 1, 0, 3)],
    ['right triangle', () => Body.polygon(points([0, 0], [1.5, 0], [0, 1]), 1, 0, 3)],
  ])('a dropped %s comes to rest lying on a flat side', (_name, make) => {
    const world = groundWorld();
    const body = world.add(make());
    body.angle = 0.4;
    run(world, 6);
    expect(body.velocity.length()).toBeLessThan(0.01);
    expect(Math.abs(body.angularVelocity)).toBeLessThan(0.01);
    const [lowest, second] = lowestVertexHeights(body);
    expect(lowest).toBeGreaterThan(-0.01);
    expect(lowest).toBeLessThan(0.01);
    expect(second - lowest).toBeLessThan(0.02);
  });

  it('polygons and boxes stack: a triangle-topped column stands and sleeps', () => {
    const world = groundWorld();
    const boxes = [0, 1, 2].map((i) => world.add(Body.box(1, 1, 1, 0, 0.5 + i)));
    const cap = world.add(Body.regularPolygon(3, 0.6, 1, 0, 3 + 0.3));
    run(world, 6);
    expect(boxes.every((box) => Math.abs(box.position.x) < 0.05)).toBe(true);
    expect(Math.abs(cap.position.x)).toBeLessThan(0.05);
    expect(cap.awake).toBe(false);
  });

  it('a head-on collision of two square polygons conserves momentum and nearly all energy', () => {
    const world = new World();
    world.gravity.set(0, 0);
    const square = (x: number) => Body.polygon(points([-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]), 1, x, 0);
    const a = world.add(square(-3));
    const b = world.add(square(3));
    a.velocity.set(2, 0);
    b.velocity.set(-2, 0);
    a.restitution = 1;
    b.restitution = 1;
    run(world, 4);
    expect(a.velocity.x + b.velocity.x).toBeCloseTo(0, 9);
    expect(a.velocity.x).toBeCloseTo(-2, 6);
    expect(b.velocity.x).toBeCloseTo(2, 6);
  });
});
