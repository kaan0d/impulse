import type { Manifold } from './manifold';
import type { Vec2 } from './vec2';

// A later SAT axis replaces the current one only if clearly shallower, keeping the reference face stable.
export const RELATIVE_TOL = 0.95;
export const ABSOLUTE_TOL = 0.01;

/// Finds the segment parameters t in [0,1] where offset d0 + (d1-d0)t stays within +-half. False if none.
export function clipToSlab(d0: number, d1: number, half: number, range: Vec2): boolean {
  const span = d1 - d0;
  if (span === 0) {
    range.set(0, 1);
    return Math.abs(d0) <= half;
  }
  const t0 = (-half - d0) / span;
  const t1 = (half - d0) / span;
  range.set(Math.max(0, Math.min(t0, t1)), Math.min(1, Math.max(t0, t1)));
  return range.x <= range.y;
}

/// Adds a contact for a point on or behind the reference face plane (n . p = planeOffset).
export function emitIfBelowFace(
  out: Manifold,
  px: number,
  py: number,
  rnx: number,
  rny: number,
  planeOffset: number,
  id: number,
): void {
  const separation = px * rnx + py * rny - planeOffset;
  if (separation > 0) return;
  // Midway between the point and its projection onto the face.
  out.addContact(px - (rnx * separation) / 2, py - (rny * separation) / 2, -separation, id);
}
