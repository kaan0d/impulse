import type { Body } from './body';
import { clamp } from './math';
import type { Manifold } from './manifold';

const VELOCITY_ITERATIONS = 10;
// Slower impacts are treated as inelastic, so resting contacts do not jitter.
const RESTITUTION_THRESHOLD = 1;
// Overlap tolerated before position correction starts, keeps contacts alive at rest.
const PENETRATION_SLOP = 0.005;
// Fraction of the excess overlap removed per step.
const CORRECTION_FACTOR = 0.2;

/// Sequential-impulse solver for contact points, after Box2D Lite. Pools its constraints; no per-step allocation.
export class ContactSolver {
  private readonly constraints: ContactConstraint[] = [];
  private count = 0;

  // Changes body velocities so contacts stop approaching, then pushes overlapping bodies apart.
  solve(manifolds: Manifold[], manifoldCount: number): void {
    this.prepare(manifolds, manifoldCount);
    const { constraints, count } = this;
    for (let iteration = 0; iteration < VELOCITY_ITERATIONS; iteration++) {
      for (let i = 0; i < count; i++) {
        constraints[i].solveFriction();
        constraints[i].solveNormal();
      }
    }
    for (let i = 0; i < count; i++) constraints[i].correctPosition();
  }

  private prepare(manifolds: Manifold[], manifoldCount: number): void {
    let count = 0;
    for (let i = 0; i < manifoldCount; i++) {
      const manifold = manifolds[i];
      for (let j = 0; j < manifold.count; j++) {
        let constraint: ContactConstraint | undefined = this.constraints[count];
        if (!constraint) {
          constraint = new ContactConstraint(manifold.bodyA, manifold.bodyB);
          this.constraints.push(constraint);
        }
        constraint.prepare(manifold, j);
        count++;
      }
    }
    this.count = count;
  }
}

/// One contact point: a non-penetration constraint plus a Coulomb friction constraint.
class ContactConstraint {
  // Normal points from bodyA toward bodyB; the friction tangent is (ny, -nx).
  private nx = 0;
  private ny = 0;
  private depth = 0;
  private friction = 0;
  // Contact offsets from each body's center of mass.
  private rAx = 0;
  private rAy = 0;
  private rBx = 0;
  private rBy = 0;
  private normalMass = 0;
  private tangentMass = 0;
  // Target normal speed after the solve: bounce speed, or 0 for inelastic contact.
  private restitutionBias = 0;
  // Accumulated over the iterations of one step and clamped, never the per-iteration delta.
  private normalImpulse = 0;
  private tangentImpulse = 0;

  constructor(
    private bodyA: Body,
    private bodyB: Body,
  ) {}

  prepare(manifold: Manifold, pointIndex: number): void {
    const a = manifold.bodyA;
    const b = manifold.bodyB;
    const point = manifold.points[pointIndex];
    this.bodyA = a;
    this.bodyB = b;
    this.nx = manifold.normal.x;
    this.ny = manifold.normal.y;
    this.depth = manifold.depths[pointIndex];
    this.friction = Math.sqrt(a.friction * b.friction);
    this.rAx = point.x - a.position.x;
    this.rAy = point.y - a.position.y;
    this.rBx = point.x - b.position.x;
    this.rBy = point.y - b.position.y;
    this.normalImpulse = 0;
    this.tangentImpulse = 0;

    const tx = this.ny;
    const ty = -this.nx;
    this.normalMass = this.effectiveMass(this.nx, this.ny);
    this.tangentMass = this.effectiveMass(tx, ty);

    const approachSpeed = this.velocityAlong(this.nx, this.ny);
    const bounces = approachSpeed < -RESTITUTION_THRESHOLD;
    this.restitutionBias = bounces ? -Math.max(a.restitution, b.restitution) * approachSpeed : 0;
  }

  solveFriction(): void {
    const tx = this.ny;
    const ty = -this.nx;
    const impulse = -this.tangentMass * this.velocityAlong(tx, ty);
    const limit = this.friction * this.normalImpulse;
    const next = clamp(this.tangentImpulse + impulse, -limit, limit);
    const applied = next - this.tangentImpulse;
    this.tangentImpulse = next;
    this.applyImpulse(tx * applied, ty * applied);
  }

  solveNormal(): void {
    const impulse = -this.normalMass * (this.velocityAlong(this.nx, this.ny) - this.restitutionBias);
    const next = Math.max(this.normalImpulse + impulse, 0);
    const applied = next - this.normalImpulse;
    this.normalImpulse = next;
    this.applyImpulse(this.nx * applied, this.ny * applied);
  }

  // Moves the bodies apart along the normal without touching velocity, so no energy is added.
  correctPosition(): void {
    const { bodyA: a, bodyB: b } = this;
    const excess = this.depth - PENETRATION_SLOP;
    if (excess <= 0) return;
    const shift = (CORRECTION_FACTOR * excess) / (a.invMass + b.invMass);
    a.position.x -= this.nx * shift * a.invMass;
    a.position.y -= this.ny * shift * a.invMass;
    b.position.x += this.nx * shift * b.invMass;
    b.position.y += this.ny * shift * b.invMass;
  }

  // Inverse of the constraint's response: how much impulse along (dx, dy) gives unit speed change.
  private effectiveMass(dx: number, dy: number): number {
    const { bodyA: a, bodyB: b } = this;
    const crossA = this.rAx * dy - this.rAy * dx;
    const crossB = this.rBx * dy - this.rBy * dx;
    return 1 / (a.invMass + b.invMass + a.invInertia * crossA * crossA + b.invInertia * crossB * crossB);
  }

  // Speed of bodyB relative to bodyA at the contact point, projected on (dx, dy).
  private velocityAlong(dx: number, dy: number): number {
    const { bodyA: a, bodyB: b } = this;
    const relX = b.velocity.x - b.angularVelocity * this.rBy - (a.velocity.x - a.angularVelocity * this.rAy);
    const relY = b.velocity.y + b.angularVelocity * this.rBx - (a.velocity.y + a.angularVelocity * this.rAx);
    return relX * dx + relY * dy;
  }

  private applyImpulse(px: number, py: number): void {
    const { bodyA: a, bodyB: b } = this;
    a.velocity.x -= a.invMass * px;
    a.velocity.y -= a.invMass * py;
    a.angularVelocity -= a.invInertia * (this.rAx * py - this.rAy * px);
    b.velocity.x += b.invMass * px;
    b.velocity.y += b.invMass * py;
    b.angularVelocity += b.invInertia * (this.rBx * py - this.rBy * px);
  }
}
