import { Vec2 } from './vec2';

export interface CircleShape {
  kind: 'circle';
  radius: number;
}

export interface BoxShape {
  kind: 'box';
  width: number;
  height: number;
}

/// Convex polygon in the body's local frame. The origin is the centroid, vertices wind counter-clockwise,
/// and `normals[i]` is the outward unit normal of the edge from `vertices[i]` to `vertices[i + 1]`.
export interface PolygonShape {
  kind: 'polygon';
  vertices: Vec2[];
  normals: Vec2[];
}

export type Shape = CircleShape | BoxShape | PolygonShape;

// Collision frames use fixed-size buffers, so polygons are capped.
export const MAX_POLYGON_VERTICES = 16;

/// Rigid body in SI units, y up. Mass `Infinity` makes it static (zero inverse mass and inertia).
export class Body {
  readonly position: Vec2;
  readonly velocity = new Vec2();
  angle = 0;
  angularVelocity = 0;
  friction = 0.5;
  restitution = 0;
  // Fraction of speed lost per second, linear and spinning; zero means none.
  linearDamping = 0;
  angularDamping = 0;
  // Lever arm in metres of the torque that resists rolling: a rolling coefficient times the radius.
  rollingResistance = 0;
  // False bodies pass through everything; used for markers such as a mouse-joint cursor.
  collidable = true;
  // Bodies meet only if each mask contains the other's category bit.
  category = 1;
  mask = 0xffffffff;
  // A shared non-zero group beats the masks: positive always collides, negative never.
  group = 0;
  // Index in `world.bodies`, assigned by `World.add`.
  id = -1;
  awake = true;
  // Seconds spent below the sleep speed thresholds.
  sleepTime = 0;
  // Where the step began and how far the body has turned since, as cosine and sine; kept by the solver.
  startX = 0;
  startY = 0;
  deltaCos = 1;
  deltaSin = 0;

  readonly invMass: number;
  readonly inertia: number;
  readonly invInertia: number;
  // Radius of the largest circle around the center that fits inside the shape.
  readonly coreRadius: number;

  constructor(
    readonly shape: Shape,
    readonly mass: number,
    x: number,
    y: number,
  ) {
    if (!(mass > 0)) throw new RangeError(`mass must be > 0, got ${mass}`);
    this.position = new Vec2(x, y);
    this.invMass = 1 / mass;
    this.inertia = computeInertia(shape, mass);
    this.invInertia = 1 / this.inertia;
    this.coreRadius = computeCoreRadius(shape);
  }

  // False for static and sleeping bodies: they are skipped by integration and the solver.
  get isSimulated(): boolean {
    return this.invMass !== 0 && this.awake;
  }

  resetMotion(): void {
    this.startX = this.position.x;
    this.startY = this.position.y;
    this.deltaCos = 1;
    this.deltaSin = 0;
  }

  // Adds gravity for one substep, then damping, which is stable at any rate.
  integrateVelocity(gravity: Vec2, dt: number): void {
    this.velocity.addScaled(gravity, dt);
    if (this.linearDamping > 0) this.velocity.scale(1 / (1 + dt * this.linearDamping));
    if (this.angularDamping > 0) this.angularVelocity /= 1 + dt * this.angularDamping;
  }

  // Semi-implicit Euler position update that also tracks the turn for the solver's contact separation.
  advance(dt: number): void {
    this.position.addScaled(this.velocity, dt);
    const turn = this.angularVelocity * dt;
    this.angle += turn;
    // Small-angle rotation, renormalized: much cheaper than trig every substep.
    const cos = this.deltaCos - turn * this.deltaSin;
    const sin = this.deltaSin + turn * this.deltaCos;
    const scale = 1 / Math.sqrt(cos * cos + sin * sin);
    this.deltaCos = cos * scale;
    this.deltaSin = sin * scale;
  }

  canCollideWith(other: Body): boolean {
    if (this.group !== 0 && this.group === other.group) return this.group > 0;
    return (this.mask & other.category) !== 0 && (other.mask & this.category) !== 0;
  }

  // True if the world-space point lies inside the shape (edges count as inside).
  containsPoint(x: number, y: number): boolean {
    const cos = Math.cos(this.angle);
    const sin = Math.sin(this.angle);
    const dx = x - this.position.x;
    const dy = y - this.position.y;
    // Point in the body's local frame.
    const lx = cos * dx + sin * dy;
    const ly = -sin * dx + cos * dy;
    const { shape } = this;
    switch (shape.kind) {
      case 'circle':
        return lx * lx + ly * ly <= shape.radius * shape.radius;
      case 'box':
        return Math.abs(lx) <= shape.width / 2 && Math.abs(ly) <= shape.height / 2;
      case 'polygon':
        return shape.vertices.every((vertex, i) => shape.normals[i].x * (lx - vertex.x) + shape.normals[i].y * (ly - vertex.y) <= 0);
    }
  }

  static circle(radius: number, mass: number, x: number, y: number): Body {
    return new Body({ kind: 'circle', radius }, mass, x, y);
  }

  static box(width: number, height: number, mass: number, x: number, y: number): Body {
    return new Body({ kind: 'box', width, height }, mass, x, y);
  }

  // `points` is any-winding convex outline in any frame; the body's position becomes its centroid.
  static polygon(points: Vec2[], mass: number, x: number, y: number): Body {
    return new Body(createPolygon(points), mass, x, y);
  }

  // Corner 0 points up; `radius` is the distance from the center to each corner.
  static regularPolygon(sides: number, radius: number, mass: number, x: number, y: number): Body {
    const points: Vec2[] = [];
    for (let i = 0; i < sides; i++) {
      const angle = Math.PI / 2 + (i * 2 * Math.PI) / sides;
      points.push(new Vec2(radius * Math.cos(angle), radius * Math.sin(angle)));
    }
    return Body.polygon(points, mass, x, y);
  }
}

// Moment of inertia about the center of mass.
function computeInertia(shape: Shape, mass: number): number {
  switch (shape.kind) {
    case 'circle':
      return 0.5 * mass * shape.radius ** 2;
    case 'box':
      return (mass * (shape.width ** 2 + shape.height ** 2)) / 12;
    case 'polygon':
      return polygonInertia(shape.vertices, mass);
  }
}

function computeCoreRadius(shape: Shape): number {
  switch (shape.kind) {
    case 'circle':
      return shape.radius;
    case 'box':
      return Math.min(shape.width, shape.height) / 2;
    case 'polygon':
      // The polygon is centred on its centroid, so n . v is the distance from the center to that edge's line.
      return Math.min(...shape.vertices.map((vertex, i) => shape.normals[i].dot(vertex)));
  }
}

// Sum over triangles fanned from the origin; exact for a polygon centred on its centroid.
function polygonInertia(vertices: Vec2[], mass: number): number {
  let crossSum = 0;
  let inertiaSum = 0;
  for (let i = 0; i < vertices.length; i++) {
    const v1 = vertices[i];
    const v2 = vertices[(i + 1) % vertices.length];
    const cross = v1.cross(v2);
    crossSum += cross;
    inertiaSum += cross * (v1.dot(v1) + v1.dot(v2) + v2.dot(v2));
  }
  return (mass * inertiaSum) / (6 * crossSum);
}

function createPolygon(points: Vec2[]): PolygonShape {
  if (points.length < 3 || points.length > MAX_POLYGON_VERTICES) {
    throw new RangeError(`polygon needs 3 to ${MAX_POLYGON_VERTICES} vertices, got ${points.length}`);
  }
  let vertices = points.map((point) => point.clone());
  if (signedArea(vertices) < 0) vertices = vertices.reverse();
  if (!(signedArea(vertices) > 1e-9)) throw new RangeError('polygon has no area');
  if (!isConvex(vertices)) throw new RangeError('polygon must be convex');

  const centroid = centroidOf(vertices);
  for (const vertex of vertices) vertex.sub(centroid);
  const normals = vertices.map((vertex, i) => {
    const next = vertices[(i + 1) % vertices.length];
    return new Vec2(next.y - vertex.y, vertex.x - next.x).scale(1 / Math.hypot(next.x - vertex.x, next.y - vertex.y));
  });
  return { kind: 'polygon', vertices, normals };
}

function signedArea(vertices: Vec2[]): number {
  let twice = 0;
  for (let i = 0; i < vertices.length; i++) twice += vertices[i].cross(vertices[(i + 1) % vertices.length]);
  return twice / 2;
}

// Every turn must go the same way; collinear vertices are allowed.
function isConvex(vertices: Vec2[]): boolean {
  const count = vertices.length;
  for (let i = 0; i < count; i++) {
    const a = vertices[i];
    const b = vertices[(i + 1) % count];
    const c = vertices[(i + 2) % count];
    if ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) < -1e-9) return false;
  }
  return true;
}

function centroidOf(vertices: Vec2[]): Vec2 {
  const centroid = new Vec2();
  let twiceArea = 0;
  for (let i = 0; i < vertices.length; i++) {
    const v1 = vertices[i];
    const v2 = vertices[(i + 1) % vertices.length];
    const cross = v1.cross(v2);
    twiceArea += cross;
    centroid.x += (v1.x + v2.x) * cross;
    centroid.y += (v1.y + v2.y) * cross;
  }
  return centroid.scale(1 / (3 * twiceArea));
}
