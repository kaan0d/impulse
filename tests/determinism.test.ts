import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { FIXED_DT, FixedStepper } from '../src/engine/stepper';
import { World } from '../src/engine/world';
import { buildTower, groundWorld } from './helpers';

function makeScene(): World {
  const world = new World();
  const circle = world.add(Body.circle(0.5, 1, 0, 10));
  circle.velocity.set(1, 2);
  circle.angularVelocity = 0.5;
  const box = world.add(Body.box(2, 1, 3, 3, 5));
  box.angularVelocity = 1;
  world.add(Body.box(10, 1, Infinity, 0, 0));
  return world;
}

// Raw bits, so +0/-0 and last-digit differences count as mismatches.
function snapshot(world: World): BigUint64Array {
  const values: number[] = [];
  for (const b of world.bodies) {
    values.push(b.position.x, b.position.y, b.angle, b.velocity.x, b.velocity.y, b.angularVelocity, b.awake ? 1 : 0);
  }
  return new BigUint64Array(new Float64Array(values).buffer);
}

function runManual(steps: number): BigUint64Array {
  const world = makeScene();
  for (let i = 0; i < steps; i++) world.step(FIXED_DT);
  return snapshot(world);
}

function runFrames(frameSeconds: number[]): { steps: number; state: BigUint64Array } {
  const world = makeScene();
  const stepper = new FixedStepper(world);
  let steps = 0;
  for (const seconds of frameSeconds) steps += stepper.advance(seconds);
  return { steps, state: snapshot(world) };
}

const irregularFrames = Array.from({ length: 200 }, (_, i) => 0.008 + (i % 7) * 0.004);

describe('determinism', () => {
  it('same scene run twice is bit-identical', () => {
    expect(runManual(300)).toEqual(runManual(300));
  });

  it('same frame sequence run twice is bit-identical', () => {
    expect(runFrames(irregularFrames)).toEqual(runFrames(irregularFrames));
  });

  it.each([
    ['60 fps', Array(120).fill(1 / 60)],
    ['30 fps', Array(60).fill(1 / 30)],
    ['144 fps', Array(288).fill(1 / 144)],
    ['irregular', irregularFrames],
  ])('%s frames give the same state as manual stepping of the same step count', (_name, frames) => {
    const { steps, state } = runFrames(frames as number[]);
    expect(steps).toBeGreaterThan(0);
    expect(state).toEqual(runManual(steps));
  });

  it('renders at different rates but simulates the same time to within one step', () => {
    const at60 = runFrames(Array(120).fill(1 / 60)).steps;
    const at144 = runFrames(Array(288).fill(1 / 144)).steps;
    expect(Math.abs(at60 - at144)).toBeLessThanOrEqual(1);
  });

  it('clamps a huge frame instead of running away', () => {
    expect(runFrames([10]).steps).toBeLessThanOrEqual(15);
  });
});

describe('determinism with stacking and sleeping', () => {
  function runTower(): BigUint64Array {
    const world = groundWorld();
    buildTower(world, 10, 0.05);
    world.add(Body.circle(0.3, 1, 0.2, 14));
    for (let i = 0; i < 900; i++) world.step(FIXED_DT);
    return snapshot(world);
  }

  it('a tower hit by a ball twice gives bit-identical results, sleep flags included', () => {
    expect(runTower()).toEqual(runTower());
  });
});

describe('determinism with polygons', () => {
  function runPolygonPile(): BigUint64Array {
    const world = groundWorld();
    for (let i = 0; i < 24; i++) {
      const body = i % 3 === 0 ? Body.box(0.8, 0.8, 1, 0, 0) : Body.regularPolygon(3 + (i % 4), 0.5, 1, 0, 0);
      body.position.set(((i % 6) - 2.5) * 0.9, 1 + Math.floor(i / 6) * 1.1);
      body.angle = i * 0.37;
      world.add(body);
    }
    for (let i = 0; i < 600; i++) world.step(FIXED_DT);
    return snapshot(world);
  }

  it('a pile of polygons and boxes run twice is bit-identical', () => {
    expect(runPolygonPile()).toEqual(runPolygonPile());
  });
});
