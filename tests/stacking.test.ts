import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { FIXED_DT } from '../src/engine/stepper';
import { buildTower, groundWorld, run } from './helpers';

const HEIGHT = 10;
const SMALL_HEIGHT = 5;

// Every box must stay near its starting spot: sag from slop penetration is tolerated, drift and tilt are not.
function expectStanding(boxes: Body[], wobble: number): void {
  boxes.forEach((box, i) => {
    const startX = i % 2 === 0 ? 0 : wobble;
    expect(Math.abs(box.position.x - startX), `box ${i} x`).toBeLessThan(0.1);
    expect(Math.abs(box.position.y - (0.5 + i)), `box ${i} y`).toBeLessThan(0.1);
    expect(Math.abs(box.angle), `box ${i} angle`).toBeLessThan(0.05);
  });
}

describe('tower', () => {
  it('a 10-box tower stands for 10 seconds', () => {
    const world = groundWorld();
    const boxes = buildTower(world, HEIGHT);
    run(world, 10);
    expectStanding(boxes, 0);
  });

  it('a slightly staggered 10-box tower stands for 10 seconds', () => {
    const world = groundWorld();
    const boxes = buildTower(world, HEIGHT, 0.05);
    run(world, 10);
    expectStanding(boxes, 0.05);
  });

  it('a tower dropped from a small gap settles and stands', () => {
    const world = groundWorld();
    const boxes = buildTower(world, HEIGHT);
    boxes.forEach((box, i) => box.position.set(0, 0.5 + i * 1.1));
    run(world, 10);
    expectStanding(boxes, 0);
  });
});

describe('block solver: tall stacks', () => {
  it('a 10-box tower settles and sleeps within 3 seconds', () => {
    const world = groundWorld();
    const boxes = buildTower(world, HEIGHT);
    run(world, 3);
    expect(boxes.every((box) => !box.awake)).toBe(true);
    expectStanding(boxes, 0);
  });

  it.each([
    [15, 0],
    [15, 0.05],
    [20, 0.03],
  ])('a %i-box tower with %f stagger stands and falls asleep within 10 seconds', (height, wobble) => {
    const world = groundWorld();
    const boxes = buildTower(world, height, wobble);
    run(world, 10);
    expect(boxes.every((box) => !box.awake)).toBe(true);
    boxes.forEach((box, i) => {
      expect(Math.abs(box.position.x - (i % 2 === 0 ? 0 : wobble)), `box ${i} x`).toBeLessThan(0.1);
      expect(Math.abs(box.position.y - (0.5 + i)), `box ${i} y`).toBeLessThan(0.2);
    });
  });
});

describe('sleeping', () => {
  it('a settled 5-box tower falls asleep and stops moving', () => {
    const world = groundWorld();
    const boxes = buildTower(world, SMALL_HEIGHT);
    run(world, 5);
    expect(boxes.every((box) => !box.awake)).toBe(true);

    const frozen = boxes.map((box) => box.position.y);
    run(world, 1);
    expect(boxes.map((box) => box.position.y)).toEqual(frozen);
    expect(world.manifoldCount).toBe(0);
  });

  it('a lone resting box sleeps only after the sleep delay', () => {
    const world = groundWorld();
    const box = world.add(Body.box(1, 1, 1, 0, 0.5));
    run(world, 0.3);
    expect(box.awake).toBe(true);
    run(world, 1);
    expect(box.awake).toBe(false);
  });

  it('a body still in motion never sleeps', () => {
    const world = groundWorld();
    const ball = world.add(Body.circle(0.5, 1, 0, 30));
    run(world, 1.5);
    expect(ball.awake).toBe(true);
  });

  it('a sleeping island stays asleep while its neighbour keeps moving', () => {
    const world = groundWorld();
    const resting = world.add(Body.box(1, 1, 1, 0, 0.5));
    const moving = world.add(Body.circle(0.5, 1, 20, 100));
    run(world, 1.5);
    expect(resting.awake).toBe(false);
    expect(moving.awake).toBe(true);
  });

  it('a falling body wakes the sleeping tower, which settles and sleeps again', () => {
    const world = groundWorld();
    const boxes = buildTower(world, SMALL_HEIGHT);
    run(world, 5);
    expect(boxes.every((box) => !box.awake)).toBe(true);

    world.add(Body.circle(0.3, 0.5, 0, 15));
    let bottomWoke = false;
    for (let i = 0; i < 6 * 60; i++) {
      world.step(FIXED_DT);
      bottomWoke ||= boxes[0].awake;
    }
    expect(bottomWoke).toBe(true);

    run(world, 10);
    expect(boxes.every((box) => !box.awake)).toBe(true);
    expectStanding(boxes, 0);
  });
});
