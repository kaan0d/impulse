import type { Body } from './body';

/// A fast body can step over thin static geometry between two frames. This sweeps the body's inscribed
/// circle along its step and stops the body where that circle first meets a static surface. The velocity is
/// kept, so the contact solver still resolves the impact next step. Static bodies only; allocation-free.
export function sweepAgainstStatics(body: Body, startX: number, startY: number, statics: Body[]): void {
  const dx = body.position.x - startX;
  const dy = body.position.y - startY;
  const core = body.coreRadius;
  if (statics.length === 0 || dx * dx + dy * dy <= core * core) return;

  let first = 1;
  for (const wall of statics) first = Math.min(first, entryFraction(wall, startX, startY, dx, dy, core));
  if (first < 1) body.position.set(startX + dx * first, startY + dy * first);
}

// Fraction of the step at which a point moving from (x0, y0) by (dx, dy) reaches the wall grown by `radius`.
// Returns 1 for no hit, and also when the point starts inside (the solver handles existing overlap).
function entryFraction(wall: Body, x0: number, y0: number, dx: number, dy: number, radius: number): number {
  const { shape } = wall;
  const relX = x0 - wall.position.x;
  const relY = y0 - wall.position.y;
  if (shape.kind === 'circle') return segmentVsCircle(relX, relY, dx, dy, shape.radius + radius);

  const cos = Math.cos(wall.angle);
  const sin = Math.sin(wall.angle);
  const lx = cos * relX + sin * relY;
  const ly = -sin * relX + cos * relY;
  const ldx = cos * dx + sin * dy;
  const ldy = -sin * dx + cos * dy;

  enter = -Infinity;
  exit = Infinity;
  if (shape.kind === 'box') {
    const hx = shape.width / 2 + radius;
    const hy = shape.height / 2 + radius;
    const inside = clipPlane(1, 0, hx, lx, ly, ldx, ldy) && clipPlane(-1, 0, hx, lx, ly, ldx, ldy) && clipPlane(0, 1, hy, lx, ly, ldx, ldy) && clipPlane(0, -1, hy, lx, ly, ldx, ldy);
    return inside ? hitFraction() : 1;
  }
  for (let i = 0; i < shape.vertices.length; i++) {
    const n = shape.normals[i];
    if (!clipPlane(n.x, n.y, n.x * shape.vertices[i].x + n.y * shape.vertices[i].y + radius, lx, ly, ldx, ldy)) return 1;
  }
  return hitFraction();
}

// Running entry and exit fractions of the segment against the planes clipped so far.
let enter = 0;
let exit = 0;

// Clips the segment against the half-space n . p <= offset. False when the segment lies entirely outside it.
function clipPlane(nx: number, ny: number, offset: number, x0: number, y0: number, dx: number, dy: number): boolean {
  const distance = nx * x0 + ny * y0 - offset;
  const rate = nx * dx + ny * dy;
  if (rate === 0) return distance <= 0;
  const t = -distance / rate;
  if (rate < 0) enter = Math.max(enter, t);
  else exit = Math.min(exit, t);
  return true;
}

function hitFraction(): number {
  return enter >= 0 && enter < 1 && enter <= exit ? enter : 1;
}

function segmentVsCircle(x0: number, y0: number, dx: number, dy: number, radius: number): number {
  const c = x0 * x0 + y0 * y0 - radius * radius;
  if (c <= 0) return 1;
  const a = dx * dx + dy * dy;
  const b = x0 * dx + y0 * dy;
  const discriminant = b * b - a * c;
  if (discriminant < 0) return 1;
  const t = (-b - Math.sqrt(discriminant)) / a;
  return t >= 0 && t < 1 ? t : 1;
}
