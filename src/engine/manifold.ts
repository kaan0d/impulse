import type { Body } from './body';
import { Vec2 } from './vec2';

/// Contact patch between two bodies. Preallocated; `collide` overwrites it in place.
export class Manifold {
  // Unit vector from bodyA toward bodyB.
  readonly normal = new Vec2();
  // Only the first `count` entries are valid.
  readonly points = [new Vec2(), new Vec2()];
  readonly depths = [0, 0];
  count = 0;

  constructor(
    public bodyA: Body,
    public bodyB: Body,
  ) {}

  addContact(x: number, y: number, depth: number): void {
    this.points[this.count].set(x, y);
    this.depths[this.count] = depth;
    this.count++;
  }
}
