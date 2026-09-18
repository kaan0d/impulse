import { MAX_POLYGON_VERTICES, type Body, type CircleShape, type PolygonShape } from './body';
import { ABSOLUTE_TOL, RELATIVE_TOL, clipToSlab, emitIfBelowFace } from './clip';
import type { Manifold } from './manifold';
import { Vec2 } from './vec2';

/// World-space convex polygon: a box is four vertices. Edge i runs from vertex i to vertex i + 1.
class ConvexFrame {
  count = 0;
  readonly x = new Float64Array(MAX_POLYGON_VERTICES);
  readonly y = new Float64Array(MAX_POLYGON_VERTICES);
  // Outward unit normal of each edge.
  readonly nx = new Float64Array(MAX_POLYGON_VERTICES);
  readonly ny = new Float64Array(MAX_POLYGON_VERTICES);

  load(body: Body): void {
    const { shape } = body;
    const cos = Math.cos(body.angle);
    const sin = Math.sin(body.angle);
    if (shape.kind === 'box') {
      const hx = shape.width / 2;
      const hy = shape.height / 2;
      this.count = 4;
      this.set(0, -hx, -hy, 0, -1, body, cos, sin);
      this.set(1, hx, -hy, 1, 0, body, cos, sin);
      this.set(2, hx, hy, 0, 1, body, cos, sin);
      this.set(3, -hx, hy, -1, 0, body, cos, sin);
      return;
    }
    if (shape.kind !== 'polygon') throw new Error('ConvexFrame needs a box or polygon body');
    this.count = shape.vertices.length;
    for (let i = 0; i < this.count; i++) {
      this.set(i, shape.vertices[i].x, shape.vertices[i].y, shape.normals[i].x, shape.normals[i].y, body, cos, sin);
    }
  }

  // Stores vertex i and its edge normal after rotating and translating them by the body's pose.
  private set(i: number, lx: number, ly: number, lnx: number, lny: number, body: Body, cos: number, sin: number): void {
    this.x[i] = body.position.x + cos * lx - sin * ly;
    this.y[i] = body.position.y + sin * lx + cos * ly;
    this.nx[i] = cos * lnx - sin * lny;
    this.ny[i] = sin * lnx + cos * lny;
  }
}

class Separation {
  value = 0;
  edge = 0;
}

const convexA = new ConvexFrame();
const convexB = new ConvexFrame();
const separationA = new Separation();
const separationB = new Separation();
const clipRange = new Vec2();

/// SAT with reference-face clipping for any pair of boxes and polygons, at most two contacts.
export function convexVsConvex(a: Body, b: Body, out: Manifold): boolean {
  convexA.load(a);
  convexB.load(b);
  findMaxSeparation(convexA, convexB, separationA);
  if (separationA.value > 0) return false;
  findMaxSeparation(convexB, convexA, separationB);
  if (separationB.value > 0) return false;

  // Separations are negative when overlapping; B's face must be clearly shallower to become the reference.
  const refIsA = !(separationB.value > RELATIVE_TOL * separationA.value + ABSOLUTE_TOL);
  const ref = refIsA ? convexA : convexB;
  const inc = refIsA ? convexB : convexA;
  const refEdge = refIsA ? separationA.edge : separationB.edge;
  const rnx = ref.nx[refEdge];
  const rny = ref.ny[refEdge];
  out.normal.set(refIsA ? rnx : -rnx, refIsA ? rny : -rny);

  // Incident edge: the one on the other polygon whose normal opposes the reference normal most.
  let incEdge = 0;
  let mostOpposed = Infinity;
  for (let i = 0; i < inc.count; i++) {
    const alignment = rnx * inc.nx[i] + rny * inc.ny[i];
    if (alignment < mostOpposed) {
      mostOpposed = alignment;
      incEdge = i;
    }
  }
  const next = (incEdge + 1) % inc.count;
  const x0 = inc.x[incEdge];
  const y0 = inc.y[incEdge];
  const x1 = inc.x[next];
  const y1 = inc.y[next];

  // Clip the incident edge to the reference edge's extent along its tangent (-rny, rnx).
  const refNext = (refEdge + 1) % ref.count;
  const refStart = -rny * ref.x[refEdge] + rnx * ref.y[refEdge];
  const refEnd = -rny * ref.x[refNext] + rnx * ref.y[refNext];
  const center = (refStart + refEnd) / 2;
  const d0 = -rny * x0 + rnx * y0 - center;
  const d1 = -rny * x1 + rnx * y1 - center;
  if (!clipToSlab(d0, d1, (refEnd - refStart) / 2, clipRange)) return false;

  const planeOffset = rnx * ref.x[refEdge] + rny * ref.y[refEdge];
  const featureBase = (refIsA ? 1024 : 0) + refEdge * 64 + incEdge * 2;
  emitIfBelowFace(out, x0 + (x1 - x0) * clipRange.x, y0 + (y1 - y0) * clipRange.x, rnx, rny, planeOffset, featureBase);
  // A clip that collapsed to one point would otherwise emit it twice.
  if (clipRange.y > clipRange.x) {
    emitIfBelowFace(out, x0 + (x1 - x0) * clipRange.y, y0 + (y1 - y0) * clipRange.y, rnx, rny, planeOffset, featureBase + 1);
  }
  return out.count > 0;
}

// For each edge of `a`, the deepest reach of `b` behind it; keeps the edge whose separation is largest.
function findMaxSeparation(a: ConvexFrame, b: ConvexFrame, result: Separation): void {
  let best = -Infinity;
  let bestEdge = 0;
  for (let i = 0; i < a.count; i++) {
    let least = Infinity;
    for (let j = 0; j < b.count; j++) {
      const separation = a.nx[i] * (b.x[j] - a.x[i]) + a.ny[i] * (b.y[j] - a.y[i]);
      if (separation < least) least = separation;
    }
    if (least > best) {
      best = least;
      bestEdge = i;
    }
  }
  result.value = best;
  result.edge = bestEdge;
}

export function circleVsPolygon(circle: Body, sc: CircleShape, poly: Body, sp: PolygonShape, out: Manifold): boolean {
  const hit = polygonVsCircle(poly, sp, circle, sc, out);
  // polygonVsCircle points polygon to circle; this pair's normal must point circle to polygon.
  out.normal.scale(-1);
  return hit;
}

export function polygonVsCircle(poly: Body, sp: PolygonShape, circle: Body, sc: CircleShape, out: Manifold): boolean {
  const cos = Math.cos(poly.angle);
  const sin = Math.sin(poly.angle);
  const dx = circle.position.x - poly.position.x;
  const dy = circle.position.y - poly.position.y;
  // Circle center in the polygon's local frame.
  const lx = cos * dx + sin * dy;
  const ly = -sin * dx + cos * dy;
  const { vertices, normals } = sp;

  let edge = 0;
  let gap = -Infinity;
  for (let i = 0; i < vertices.length; i++) {
    const separation = normals[i].x * (lx - vertices[i].x) + normals[i].y * (ly - vertices[i].y);
    if (separation > gap) {
      gap = separation;
      edge = i;
    }
  }
  if (gap > sc.radius) return false;

  // Closest polygon surface point p and outward normal n; `gap` becomes the signed distance from p to the center.
  const v1 = vertices[edge];
  const v2 = vertices[(edge + 1) % vertices.length];
  const alongEdge = (lx - v1.x) * (v2.x - v1.x) + (ly - v1.y) * (v2.y - v1.y);
  const alongEdgeFromEnd = (lx - v2.x) * (v1.x - v2.x) + (ly - v2.y) * (v1.y - v2.y);
  const cornerFrom = gap >= 0 && alongEdge <= 0 ? v1 : gap >= 0 && alongEdgeFromEnd <= 0 ? v2 : null;

  let px = lx - normals[edge].x * gap;
  let py = ly - normals[edge].y * gap;
  let nx = normals[edge].x;
  let ny = normals[edge].y;
  if (cornerFrom) {
    const cx = lx - cornerFrom.x;
    const cy = ly - cornerFrom.y;
    const distance = Math.sqrt(cx * cx + cy * cy);
    if (distance > sc.radius) return false;
    px = cornerFrom.x;
    py = cornerFrom.y;
    // A center exactly on the corner keeps the edge normal.
    if (distance > 0) {
      nx = cx / distance;
      ny = cy / distance;
    }
    gap = distance;
  }

  const depth = sc.radius - gap;
  // Contact sits midway between the polygon surface and the circle's deepest point.
  const mx = px - (nx * depth) / 2;
  const my = py - (ny * depth) / 2;
  out.normal.set(cos * nx - sin * ny, sin * nx + cos * ny);
  out.addContact(poly.position.x + cos * mx - sin * my, poly.position.y + sin * mx + cos * my, depth, 0);
  return true;
}
