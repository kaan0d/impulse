import type { Body, BoxShape, CircleShape, PolygonShape } from './body';
import { ABSOLUTE_TOL, RELATIVE_TOL, clipToSlab, emitIfBelowFace } from './clip';
import { circleVsPolygon, convexVsConvex, polygonVsCircle } from './collide-convex';
import { clamp } from './math';
import { CONTACT_MARGIN, type Manifold } from './manifold';
import { Vec2 } from './vec2';

/// Fills `out` and returns true if the bodies overlap or are closer than CONTACT_MARGIN. The normal points from a to b.
/// Depth is negative for a gap. Allocation-free.
export function collide(a: Body, b: Body, out: Manifold): boolean {
  out.bodyA = a;
  out.bodyB = b;
  out.count = 0;
  const shapeA = a.shape;
  switch (shapeA.kind) {
    case 'circle':
      return collideCircleWith(a, shapeA, b, out);
    case 'box':
      return collideBoxWith(a, shapeA, b, out);
    case 'polygon':
      return collidePolygonWith(a, shapeA, b, out);
  }
}

function collideCircleWith(a: Body, sa: CircleShape, b: Body, out: Manifold): boolean {
  const sb = b.shape;
  switch (sb.kind) {
    case 'circle':
      return circleVsCircle(a, sa, b, sb, out);
    case 'box':
      return circleVsBox(a, sa, b, sb, out);
    case 'polygon':
      return circleVsPolygon(a, sa, b, sb, out);
  }
}

function collideBoxWith(a: Body, sa: BoxShape, b: Body, out: Manifold): boolean {
  const sb = b.shape;
  switch (sb.kind) {
    case 'circle':
      return boxVsCircle(a, sa, b, sb, out);
    case 'box':
      return boxVsBox(a, sa, b, sb, out);
    case 'polygon':
      return convexVsConvex(a, b, out);
  }
}

function collidePolygonWith(a: Body, sa: PolygonShape, b: Body, out: Manifold): boolean {
  const sb = b.shape;
  switch (sb.kind) {
    case 'circle':
      return polygonVsCircle(a, sa, b, sb, out);
    case 'box':
    case 'polygon':
      return convexVsConvex(a, b, out);
  }
}

function circleVsCircle(a: Body, sa: CircleShape, b: Body, sb: CircleShape, out: Manifold): boolean {
  const dx = b.position.x - a.position.x;
  const dy = b.position.y - a.position.y;
  const radii = sa.radius + sb.radius;
  const distSq = dx * dx + dy * dy;
  const reach = radii + CONTACT_MARGIN;
  if (distSq > reach * reach) return false;

  const dist = Math.sqrt(distSq);
  // Coincident centers have no direction, so pick +x.
  const nx = dist > 0 ? dx / dist : 1;
  const ny = dist > 0 ? dy / dist : 0;
  const depth = radii - dist;
  // Contact sits midway between the two surface points.
  const along = sa.radius - depth / 2;
  out.normal.set(nx, ny);
  out.addContact(a.position.x + nx * along, a.position.y + ny * along, depth, 0);
  return true;
}

function circleVsBox(circle: Body, sc: CircleShape, box: Body, sb: BoxShape, out: Manifold): boolean {
  const hit = boxVsCircle(box, sb, circle, sc, out);
  // boxVsCircle points box to circle; this pair's normal must point circle to box.
  out.normal.scale(-1);
  return hit;
}

function boxVsCircle(box: Body, sb: BoxShape, circle: Body, sc: CircleShape, out: Manifold): boolean {
  const cos = Math.cos(box.angle);
  const sin = Math.sin(box.angle);
  const dx = circle.position.x - box.position.x;
  const dy = circle.position.y - box.position.y;
  // Circle center in the box's local frame.
  const lx = cos * dx + sin * dy;
  const ly = -sin * dx + cos * dy;
  const hx = sb.width / 2;
  const hy = sb.height / 2;

  // Closest box surface point p, outward normal n, and signed gap from p to the center.
  let px = clamp(lx, -hx, hx);
  let py = clamp(ly, -hy, hy);
  let nx = lx - px;
  let ny = ly - py;
  let gap = Math.sqrt(nx * nx + ny * ny);
  if (gap > 0) {
    nx /= gap;
    ny /= gap;
  } else {
    // Center inside the box: exit through the nearest face.
    const gapX = hx - Math.abs(lx);
    const gapY = hy - Math.abs(ly);
    const nearX = gapX < gapY;
    nx = nearX ? signOf(lx) : 0;
    ny = nearX ? 0 : signOf(ly);
    px = nearX ? nx * hx : lx;
    py = nearX ? ly : ny * hy;
    gap = -(nearX ? gapX : gapY);
  }

  const depth = sc.radius - gap;
  if (depth < -CONTACT_MARGIN) return false;

  // Contact sits midway between the box surface and the circle's deepest point.
  const mx = px - (nx * depth) / 2;
  const my = py - (ny * depth) / 2;
  out.normal.set(cos * nx - sin * ny, sin * nx + cos * ny);
  out.addContact(box.position.x + cos * mx - sin * my, box.position.y + sin * mx + cos * my, depth, 0);
  return true;
}

/// Box axes and extents in world space, cached once per collide call.
class BoxFrame {
  x = 0;
  y = 0;
  ux = 1;
  uy = 0;
  vx = 0;
  vy = 1;
  hx = 0;
  hy = 0;

  load(body: Body, shape: BoxShape): void {
    const cos = Math.cos(body.angle);
    const sin = Math.sin(body.angle);
    this.x = body.position.x;
    this.y = body.position.y;
    this.ux = cos;
    this.uy = sin;
    this.vx = -sin;
    this.vy = cos;
    this.hx = shape.width / 2;
    this.hy = shape.height / 2;
  }

  // Half-width of the box projected onto the unit direction (nx, ny).
  radiusAlong(nx: number, ny: number): number {
    return this.hx * Math.abs(nx * this.ux + ny * this.uy) + this.hy * Math.abs(nx * this.vx + ny * this.vy);
  }
}

class Axis {
  // 0,1 are box A's x,y axes; 2,3 are box B's.
  index = -1;
  x = 0;
  y = 0;
}

class Segment {
  // Which of the box's four faces this is: x axis, y axis, then their negatives.
  face = 0;
  x0 = 0;
  y0 = 0;
  x1 = 0;
  y1 = 0;
}

const boxA = new BoxFrame();
const boxB = new BoxFrame();
const axis = new Axis();
const incidentEdge = new Segment();
const clipRange = new Vec2();

function boxVsBox(a: Body, sa: BoxShape, b: Body, sb: BoxShape, out: Manifold): boolean {
  boxA.load(a, sa);
  boxB.load(b, sb);
  const dx = boxB.x - boxA.x;
  const dy = boxB.y - boxA.y;
  if (!selectAxis(dx, dy)) return false;

  const toB = axis.x * dx + axis.y * dy >= 0 ? 1 : -1;
  const nx = axis.x * toB;
  const ny = axis.y * toB;
  out.normal.set(nx, ny);

  // Reference face belongs to the box that owns the chosen axis; its outward normal faces the other box.
  const refIsA = axis.index < 2;
  const ref = refIsA ? boxA : boxB;
  const inc = refIsA ? boxB : boxA;
  const rnx = refIsA ? nx : -nx;
  const rny = refIsA ? ny : -ny;
  const refAlongX = axis.index % 2 === 0;
  const faceHalf = refAlongX ? ref.hx : ref.hy;
  const sideHalf = refAlongX ? ref.hy : ref.hx;
  const faceX = ref.x + rnx * faceHalf;
  const faceY = ref.y + rny * faceHalf;

  findIncidentEdge(inc, rnx, rny, incidentEdge);
  const e = incidentEdge;
  // Signed offsets along the reference face tangent (-rny, rnx).
  const d0 = (faceX - e.x0) * rny + (e.y0 - faceY) * rnx;
  const d1 = (faceX - e.x1) * rny + (e.y1 - faceY) * rnx;
  if (!clipToSlab(d0, d1, sideHalf, clipRange)) return false;

  const planeOffset = faceX * rnx + faceY * rny;
  const edgeDx = e.x1 - e.x0;
  const edgeDy = e.y1 - e.y0;
  // Feature id: which box owns the reference face, which faces meet, and which edge end the point is.
  const refFace = (axis.index % 2) + (rnx * axis.x + rny * axis.y < 0 ? 2 : 0);
  const featureBase = (refIsA ? 32 : 0) + refFace * 8 + e.face * 2;
  emitIfBelowFace(out, e.x0 + edgeDx * clipRange.x, e.y0 + edgeDy * clipRange.x, rnx, rny, planeOffset, featureBase);
  // A clip that collapsed to one point would otherwise emit it twice.
  if (clipRange.y > clipRange.x) {
    emitIfBelowFace(out, e.x0 + edgeDx * clipRange.y, e.y0 + edgeDy * clipRange.y, rnx, rny, planeOffset, featureBase + 1);
  }
  return out.count > 0;
}

// Picks the least-penetration face normal of either box. Returns false if any axis separates them.
function selectAxis(dx: number, dy: number): boolean {
  let leastPenetration = Infinity;
  for (let i = 0; i < 4; i++) {
    const box = i < 2 ? boxA : boxB;
    const useX = i % 2 === 0;
    const nx = useX ? box.ux : box.vx;
    const ny = useX ? box.uy : box.vy;
    const penetration = boxA.radiusAlong(nx, ny) + boxB.radiusAlong(nx, ny) - Math.abs(dx * nx + dy * ny);
    if (penetration < -CONTACT_MARGIN) return false;
    if (penetration >= leastPenetration * RELATIVE_TOL - ABSOLUTE_TOL) continue;
    leastPenetration = penetration;
    axis.index = i;
    axis.x = nx;
    axis.y = ny;
  }
  return true;
}

// Writes the face of `inc` most opposed to the reference normal (rnx, rny) as a segment.
function findIncidentEdge(inc: BoxFrame, rnx: number, rny: number, edge: Segment): void {
  const alongU = rnx * inc.ux + rny * inc.uy;
  const alongV = rnx * inc.vx + rny * inc.vy;
  const useX = Math.abs(alongU) > Math.abs(alongV);
  const flip = (useX ? alongU : alongV) >= 0 ? -1 : 1;
  const nx = (useX ? inc.ux : inc.vx) * flip;
  const ny = (useX ? inc.uy : inc.vy) * flip;
  const faceHalf = useX ? inc.hx : inc.hy;
  const edgeHalf = useX ? inc.hy : inc.hx;
  const cx = inc.x + nx * faceHalf;
  const cy = inc.y + ny * faceHalf;
  // The edge runs perpendicular to the face normal.
  edge.face = (useX ? 0 : 1) + (flip < 0 ? 2 : 0);
  edge.x0 = cx + ny * edgeHalf;
  edge.y0 = cy - nx * edgeHalf;
  edge.x1 = cx - ny * edgeHalf;
  edge.y1 = cy + nx * edgeHalf;
}

function signOf(value: number): number {
  return value >= 0 ? 1 : -1;
}
