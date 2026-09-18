import { Body } from '../src/engine/body';
import { FIXED_DT } from '../src/engine/stepper';
import { World } from '../src/engine/world';

export function run(world: World, seconds: number): void {
  const steps = Math.round(seconds / FIXED_DT);
  for (let i = 0; i < steps; i++) world.step(FIXED_DT);
}

// Static ground with its top surface at y = 0.
export function groundWorld(): World {
  const world = new World();
  world.add(Body.box(40, 1, Infinity, 0, -0.5));
  return world;
}

// Unit boxes stacked exactly touching; odd boxes shift sideways by `wobble` (metres) to avoid a perfect column.
export function buildTower(world: World, height: number, wobble = 0): Body[] {
  const boxes: Body[] = [];
  for (let i = 0; i < height; i++) {
    boxes.push(world.add(Body.box(1, 1, 1, i % 2 === 0 ? 0 : wobble, 0.5 + i)));
  }
  return boxes;
}
