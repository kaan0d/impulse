import type { Body } from './body';
import { writeBounds } from './bounds';
import { entryFraction } from './ccd';
import { Vec2 } from './vec2';

export interface RayHit {
  body: Body;
  // Fraction of the segment from start to end where it first touches the body.
  fraction: number;
  point: Vec2;
  // Outward unit normal of the surface at `point`.
  normal: Vec2;
}

const scratchBounds = [0, 0, 0, 0];
const scratchNormal = new Vec2();

/// Appends every collidable body whose bounding box overlaps the given box to `out`, in body order.
/// Checks every body, so cost is linear in the body count.
export function queryAABB(bodies: Body[], minX: number, minY: number, maxX: number, maxY: number, out: Body[]): Body[] {
  for (const body of bodies) {
    if (!body.collidable) continue;
    writeBounds(body, scratchBounds, 0);
    const [bx0, by0, bx1, by1] = scratchBounds;
    if (bx0 <= maxX && bx1 >= minX && by0 <= maxY && by1 >= minY) out.push(body);
  }
  return out;
}

/// Closest hit of the segment (x0, y0) to (x1, y1) among collidable bodies whose category is in `mask`, or null.
/// A body that contains the start point is not hit. Checks every body, so cost is linear in the body count.
export function raycast(bodies: Body[], x0: number, y0: number, x1: number, y1: number, mask: number): RayHit | null {
  let best: RayHit | null = null;
  let bestFraction = 1;
  for (const body of bodies) {
    if (!body.collidable || (body.category & mask) === 0) continue;
    const fraction = entryFraction(body, x0, y0, x1 - x0, y1 - y0, 0, scratchNormal);
    if (fraction >= bestFraction) continue;
    bestFraction = fraction;
    best = { body, fraction, point: new Vec2(x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction), normal: scratchNormal.clone() };
  }
  return best;
}
