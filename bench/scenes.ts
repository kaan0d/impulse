import { Body } from '../src/engine/body';
import { SpatialHash } from '../src/engine/broadphase';
import { FIXED_DT } from '../src/engine/stepper';
import { World } from '../src/engine/world';

// Steps run but not timed, so JIT compilation does not skew the numbers.
const WARMUP_STEPS = 20;

export interface Scene {
  name: string;
  build: () => World;
  steps: number;
  // Off pins every body awake, which measures the solver instead of a world that has gone quiet.
  sleeping: boolean;
}

export interface SceneResult {
  name: string;
  bodies: number;
  steps: number;
  avgMs: number;
  p95Ms: number;
  maxMs: number;
  peakContacts: number;
  awakeAtEnd: number;
}

export interface BroadphaseResult {
  bodies: number;
  pairs: number;
  spatialHashMs: number;
  bruteForceMs: number;
  // Both methods must find the same pairs, or the timing comparison means nothing.
  samePairs: boolean;
}

// Seeded so every run builds the same scene.
function makeRandom(seed: number): () => number {
  let state = seed;
  return () => (state = (state * 1664525 + 1013904223) >>> 0) / 4294967296;
}

function groundedWorld(): World {
  const world = new World();
  world.add(Body.box(80, 1, Infinity, 0, -0.5));
  return world;
}

// Unit boxes, `rows` at the bottom shrinking to one at the top, each resting on two below.
function pyramid(rows: number): World {
  const world = groundedWorld();
  for (let row = 0; row < rows; row++) {
    for (let i = 0; i < rows - row; i++) {
      world.add(Body.box(1, 1, 1, (i - (rows - row - 1) / 2) * 1.02, 0.5 + row));
    }
  }
  return world;
}

// Mixed circles and boxes dropped in staggered layers into a walled container 20 m wide.
// A nonzero `rolling` gives every body that rolling resistance, so a box cannot rock on a ball forever.
export function pile(count: number, rolling = 0): World {
  const random = makeRandom(42);
  const world = groundedWorld();
  world.add(Body.box(1, 80, Infinity, -10.5, 40));
  world.add(Body.box(1, 80, Infinity, 10.5, 40));
  const columns = 20;
  for (let i = 0; i < count; i++) {
    const x = -9.5 + (i % columns) * 0.95 + random() * 0.1;
    const y = 3 + Math.floor(i / columns) * 1.2 + random() * 0.2;
    const body =
      random() < 0.4 ? Body.circle(0.3 + random() * 0.15, 1, x, y) : Body.box(0.5 + random() * 0.4, 0.5 + random() * 0.4, 1, x, y);
    body.angle = random() * Math.PI;
    body.rollingResistance = rolling;
    world.add(body);
  }
  return world;
}

export const scenes: Scene[] = [
  { name: 'Pyramid, 210 boxes', build: () => pyramid(20), steps: 240, sleeping: false },
  { name: 'Pyramid, 210 boxes, sleeping on', build: () => pyramid(20), steps: 240, sleeping: true },
  { name: 'Pile, 500 bodies', build: () => pile(500), steps: 480, sleeping: false },
  { name: 'Pile, 500 bodies, sleeping on', build: () => pile(500), steps: 480, sleeping: true },
  { name: 'Pile, 500 bodies, sleeping on, rolling resistance', build: () => pile(500, 0.03), steps: 600, sleeping: true },
  { name: 'Pile, 1000 bodies', build: () => pile(1000), steps: 300, sleeping: false },
];

export const broadphaseSizes = [250, 500, 1000, 2000, 4000];

export function runScene(scene: Scene): SceneResult {
  const world = scene.build();
  const dynamic = world.bodies.filter((body) => body.invMass !== 0);
  const timings = new Float64Array(scene.steps);
  let peakContacts = 0;

  for (let step = 0; step < WARMUP_STEPS + scene.steps; step++) {
    if (!scene.sleeping) for (const body of dynamic) body.sleepTime = 0;
    const start = performance.now();
    world.step(FIXED_DT);
    const elapsed = performance.now() - start;
    if (step < WARMUP_STEPS) continue;
    timings[step - WARMUP_STEPS] = elapsed;
    peakContacts = Math.max(peakContacts, world.manifoldCount);
  }

  const sorted = timings.slice().sort();
  return {
    name: scene.name,
    bodies: dynamic.length,
    steps: scene.steps,
    avgMs: timings.reduce((sum, ms) => sum + ms, 0) / scene.steps,
    p95Ms: sorted[Math.floor(scene.steps * 0.95)],
    maxMs: sorted[scene.steps - 1],
    peakContacts,
    awakeAtEnd: dynamic.filter((body) => body.awake).length,
  };
}

// Unit boxes scattered at constant density, so pair count grows with body count, not with crowding.
export function runBroadphase(bodyCount: number): BroadphaseResult {
  const random = makeRandom(7);
  const world = new World();
  const side = Math.sqrt(bodyCount) * 2;
  for (let i = 0; i < bodyCount; i++) world.add(Body.box(1, 1, 1, random() * side, random() * side));

  const hash = new SpatialHash(2);
  let pairs = hash.findPairs(world.bodies);
  const hashReps = 50;
  const hashStart = performance.now();
  for (let rep = 0; rep < hashReps; rep++) pairs = hash.findPairs(world.bodies);
  const spatialHashMs = (performance.now() - hashStart) / hashReps;

  let brutePairs = bruteForcePairCount(world.bodies);
  const bruteReps = Math.max(1, Math.min(20, Math.round(4e7 / bodyCount ** 2)));
  const bruteStart = performance.now();
  for (let rep = 0; rep < bruteReps; rep++) brutePairs = bruteForcePairCount(world.bodies);
  const bruteForceMs = (performance.now() - bruteStart) / bruteReps;

  return { bodies: bodyCount, pairs, spatialHashMs, bruteForceMs, samePairs: pairs === brutePairs };
}

// Every pair tested; valid for the axis-aligned unit boxes above.
function bruteForcePairCount(bodies: Body[]): number {
  let count = 0;
  for (let i = 0; i < bodies.length; i++) {
    const a = bodies[i].position;
    for (let j = i + 1; j < bodies.length; j++) {
      const b = bodies[j].position;
      if (Math.abs(a.x - b.x) <= 1 && Math.abs(a.y - b.y) <= 1) count++;
    }
  }
  return count;
}
