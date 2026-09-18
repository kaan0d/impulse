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

export type Shape = CircleShape | BoxShape;

/// Rigid body in SI units, y up. Mass `Infinity` makes it static (zero inverse mass and inertia).
export class Body {
  readonly position: Vec2;
  readonly velocity = new Vec2();
  angle = 0;
  angularVelocity = 0;
  friction = 0.5;
  restitution = 0;
  // Index in `world.bodies`, assigned by `World.add`.
  id = -1;
  awake = true;
  // Seconds spent below the sleep speed thresholds.
  sleepTime = 0;

  readonly invMass: number;
  readonly inertia: number;
  readonly invInertia: number;

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
  }

  // False for static and sleeping bodies: they are skipped by integration and the solver.
  get isSimulated(): boolean {
    return this.invMass !== 0 && this.awake;
  }

  static circle(radius: number, mass: number, x: number, y: number): Body {
    return new Body({ kind: 'circle', radius }, mass, x, y);
  }

  static box(width: number, height: number, mass: number, x: number, y: number): Body {
    return new Body({ kind: 'box', width, height }, mass, x, y);
  }
}

// Moment of inertia about the center of mass.
function computeInertia(shape: Shape, mass: number): number {
  if (shape.kind === 'circle') return 0.5 * mass * shape.radius ** 2;
  return (mass * (shape.width ** 2 + shape.height ** 2)) / 12;
}
