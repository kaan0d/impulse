import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { PAIR_KEY_STRIDE, SpatialHash } from '../src/engine/broadphase';
import { collide } from '../src/engine/collide';
import { Manifold } from '../src/engine/manifold';
import { World } from '../src/engine/world';

// Seeded generator so scenes are reproducible.
function makeRandom(seed: number): () => number {
  let state = seed;
  return () => (state = (state * 1664525 + 1013904223) >>> 0) / 4294967296;
}

interface SceneOptions {
  count: number;
  extent: number;
  staticShare?: number;
  sleepingShare?: number;
}

function randomWorld(seed: number, { count, extent, staticShare = 0.1, sleepingShare = 0.1 }: SceneOptions): World {
  const random = makeRandom(seed);
  const world = new World();
  for (let i = 0; i < count; i++) {
    const x = (random() - 0.5) * extent;
    const y = (random() - 0.5) * extent;
    const mass = random() < staticShare ? Infinity : 1;
    const body =
      random() < 0.4
        ? Body.circle(0.2 + random() * 1.2, mass, x, y)
        : Body.box(0.3 + random() * 2.5, 0.3 + random() * 2.5, mass, x, y);
    body.angle = random() * Math.PI * 2;
    body.awake = random() >= sleepingShare;
    world.add(body);
  }
  return world;
}

// Independent oracle: corners of the rotated shape, not the closed-form extent the hash uses.
function boundsOf(body: Body): [number, number, number, number] {
  const { shape, position } = body;
  if (shape.kind === 'circle') {
    return [position.x - shape.radius, position.y - shape.radius, position.x + shape.radius, position.y + shape.radius];
  }
  const xs: number[] = [];
  const ys: number[] = [];
  for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
    const lx = (sx * shape.width) / 2;
    const ly = (sy * shape.height) / 2;
    xs.push(position.x + Math.cos(body.angle) * lx - Math.sin(body.angle) * ly);
    ys.push(position.y + Math.sin(body.angle) * lx + Math.cos(body.angle) * ly);
  }
  return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
}

// Every pair with touching-or-overlapping bounds and at least one simulated body, as ascending keys.
function bruteForceKeys(bodies: Body[]): number[] {
  const keys: number[] = [];
  for (let i = 0; i < bodies.length; i++) {
    for (let j = i + 1; j < bodies.length; j++) {
      if (!bodies[i].isSimulated && !bodies[j].isSimulated) continue;
      const [ax0, ay0, ax1, ay1] = boundsOf(bodies[i]);
      const [bx0, by0, bx1, by1] = boundsOf(bodies[j]);
      if (ax0 <= bx1 && bx0 <= ax1 && ay0 <= by1 && by0 <= ay1) keys.push(i * PAIR_KEY_STRIDE + j);
    }
  }
  return keys;
}

function hashKeys(hash: SpatialHash, bodies: Body[]): number[] {
  const count = hash.findPairs(bodies);
  return Array.from(hash.pairKeys.subarray(0, count));
}

describe('SpatialHash pair set', () => {
  // Tiny cells stress multi-cell bodies and bucket collisions; huge cells put everything in one cell.
  it.each([0.25, 0.5, 2, 7, 1000])('matches brute force with cell size %s', (cellSize) => {
    for (const seed of [1, 2, 3]) {
      const world = randomWorld(seed, { count: 150, extent: 30 });
      const expected = bruteForceKeys(world.bodies);
      expect(expected.length).toBeGreaterThan(20);
      expect(hashKeys(new SpatialHash(cellSize), world.bodies)).toEqual(expected);
    }
  });

  it('matches brute force in a dense pile and a sparse scene', () => {
    for (const extent of [6, 300]) {
      const world = randomWorld(9, { count: 120, extent });
      expect(hashKeys(new SpatialHash(2), world.bodies)).toEqual(bruteForceKeys(world.bodies));
    }
  });

  it('handles negative coordinates and bodies spanning many cells', () => {
    const world = new World();
    world.add(Body.box(60, 1, Infinity, -30, -20));
    world.add(Body.circle(0.5, 1, -55, -19.6));
    world.add(Body.box(1, 1, 1, -3.2, -19.2));
    world.add(Body.box(1, 1, 1, 40, 40));
    expect(hashKeys(new SpatialHash(2), world.bodies)).toEqual(bruteForceKeys(world.bodies));
    expect(bruteForceKeys(world.bodies)).toHaveLength(2);
  });

  it('counts exactly touching bounds as overlapping', () => {
    const world = new World();
    world.add(Body.box(1, 1, 1, 0, 0));
    world.add(Body.box(1, 1, 1, 1, 0));
    world.add(Body.box(1, 1, 1, 2.001, 0));
    expect(hashKeys(new SpatialHash(1), world.bodies)).toEqual([0 * PAIR_KEY_STRIDE + 1]);
  });

  it('skips pairs where neither body is simulated', () => {
    const world = new World();
    world.add(Body.box(2, 2, Infinity, 0, 0));
    world.add(Body.box(2, 2, Infinity, 1, 0));
    const sleeper = world.add(Body.circle(1, 1, 0.5, 0));
    sleeper.awake = false;
    expect(hashKeys(new SpatialHash(2), world.bodies)).toEqual([]);
    sleeper.awake = true;
    expect(hashKeys(new SpatialHash(2), world.bodies)).toEqual([0 * PAIR_KEY_STRIDE + 2, 1 * PAIR_KEY_STRIDE + 2]);
  });

  it('stays correct as bodies move and the scene shrinks between calls', () => {
    const hash = new SpatialHash(2);
    const world = randomWorld(5, { count: 100, extent: 20 });
    for (let frame = 0; frame < 5; frame++) {
      world.bodies.forEach((body, i) => body.position.set(body.position.x + Math.sin(i + frame) * 3, body.position.y + Math.cos(i * frame) * 3));
      expect(hashKeys(hash, world.bodies)).toEqual(bruteForceKeys(world.bodies));
    }
    // Far fewer pairs than before: stale keys from the larger frame must not leak through.
    world.bodies.forEach((body) => body.position.set(body.position.x * 40, body.position.y * 40));
    expect(hashKeys(hash, world.bodies)).toEqual(bruteForceKeys(world.bodies));
  });

  it('an empty world has no pairs', () => {
    expect(hashKeys(new SpatialHash(2), [])).toEqual([]);
  });
});

describe('World contacts through the broadphase', () => {
  it('finds exactly the contacts a brute-force narrowphase finds', () => {
    for (const seed of [11, 12, 13]) {
      const world = randomWorld(seed, { count: 120, extent: 25 });
      world.detectCollisions();

      const found = new Set<number>();
      for (let k = 0; k < world.manifoldCount; k++) {
        const { bodyA, bodyB } = world.manifolds[k];
        found.add(bodyA.id * PAIR_KEY_STRIDE + bodyB.id);
      }

      const expected = new Set<number>();
      const scratch = new Manifold(world.bodies[0], world.bodies[1]);
      for (let i = 0; i < world.bodies.length; i++) {
        for (let j = i + 1; j < world.bodies.length; j++) {
          const a = world.bodies[i];
          const b = world.bodies[j];
          if (!a.isSimulated && !b.isSimulated) continue;
          if (collide(a, b, scratch)) expected.add(i * PAIR_KEY_STRIDE + j);
        }
      }
      expect(expected.size).toBeGreaterThan(10);
      expect(found).toEqual(expected);
    }
  });
});
