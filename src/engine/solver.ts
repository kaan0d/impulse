import type { Body } from './body';
import type { Joint } from './joint';
import { clamp } from './math';
import type { Manifold } from './manifold';

// Box2D Lite default; the 2-point block solver keeps tall stacks converging at this count.
const VELOCITY_ITERATIONS = 10;
// Above this ratio a 2-point block is too ill-conditioned to invert, so its points are solved one by one.
const MAX_BLOCK_CONDITION = 1000;
// Slower impacts are treated as inelastic, so resting contacts do not jitter.
const RESTITUTION_THRESHOLD = 1;
// Overlap tolerated before position correction starts, keeps contacts alive at rest.
const PENETRATION_SLOP = 0.005;
// Fraction of the excess overlap removed per position pass.
const CORRECTION_FACTOR = 0.2;
const POSITION_PASSES = 3;

/// Sequential-impulse solver for contacts and joints, after Box2D Lite. Pools its constraints; no per-step allocation.
export class Solver {
  private readonly constraints: ContactConstraint[] = [];
  private count = 0;

  // Changes body velocities so joints hold and contacts stop approaching, then pushes overlapping bodies apart.
  // Accumulated impulses persist in the manifolds and joints and warm start the next step.
  solve(manifolds: Manifold[], manifoldCount: number, joints: Joint[], invDt: number): void {
    for (const joint of joints) joint.prepare(invDt);
    this.prepare(manifolds, manifoldCount);
    const { constraints, count } = this;
    // All biases read the incoming velocities, so warm start only after every constraint is prepared.
    for (const joint of joints) joint.warmStart();
    for (let i = 0; i < count; i++) constraints[i].warmStart();
    for (let iteration = 0; iteration < VELOCITY_ITERATIONS; iteration++) {
      for (const joint of joints) joint.solve();
      for (let i = 0; i < count; i++) {
        const constraint = constraints[i];
        if (constraint.isBlockFollower) continue;
        constraint.solveFriction();
        const partner = constraint.blockPartner;
        if (partner) {
          partner.solveFriction();
          constraint.solveNormalBlock(partner);
        } else {
          constraint.solveNormal();
        }
      }
    }
    for (let i = 0; i < count; i++) constraints[i].storeImpulses();
    // Contact by contact, each pass sees the shifts of the previous ones, so a stack's shared compression resolves.
    for (let pass = 0; pass < POSITION_PASSES; pass++) {
      for (let i = 0; i < count; i++) constraints[i].correctPosition();
    }
  }

  private prepare(manifolds: Manifold[], manifoldCount: number): void {
    let count = 0;
    for (let i = 0; i < manifoldCount; i++) {
      const manifold = manifolds[i];
      for (let j = 0; j < manifold.count; j++) {
        let constraint: ContactConstraint | undefined = this.constraints[count];
        if (!constraint) {
          constraint = new ContactConstraint(manifold);
          this.constraints.push(constraint);
        }
        constraint.prepare(manifold, j);
        count++;
      }
      if (manifold.count === 2) this.constraints[count - 2].linkBlock(this.constraints[count - 1]);
    }
    this.count = count;
  }
}

/// One contact point: a non-penetration constraint plus a Coulomb friction constraint.
class ContactConstraint {
  private bodyA: Body;
  private bodyB: Body;
  private pointIndex = 0;
  // Normal points from bodyA toward bodyB; the friction tangent is (ny, -nx).
  private nx = 0;
  private ny = 0;
  private depth = 0;
  // Body positions when the contact was prepared, to tell how far corrections have already moved them.
  private startAx = 0;
  private startAy = 0;
  private startBx = 0;
  private startBy = 0;
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

  // A 2-point manifold solves both normal impulses together: the first point leads, the second follows.
  blockPartner: ContactConstraint | null = null;
  isBlockFollower = false;
  // Normal effective-mass matrix of the block and its inverse (symmetric).
  private k11 = 0;
  private k12 = 0;
  private k22 = 0;
  private inverseK11 = 0;
  private inverseK12 = 0;
  private inverseK22 = 0;

  constructor(private manifold: Manifold) {
    this.bodyA = manifold.bodyA;
    this.bodyB = manifold.bodyB;
  }

  prepare(manifold: Manifold, pointIndex: number): void {
    const a = manifold.bodyA;
    const b = manifold.bodyB;
    const point = manifold.points[pointIndex];
    this.manifold = manifold;
    this.pointIndex = pointIndex;
    this.bodyA = a;
    this.bodyB = b;
    this.nx = manifold.normal.x;
    this.ny = manifold.normal.y;
    this.depth = manifold.depths[pointIndex];
    this.startAx = a.position.x;
    this.startAy = a.position.y;
    this.startBx = b.position.x;
    this.startBy = b.position.y;
    this.friction = Math.sqrt(a.friction * b.friction);
    this.rAx = point.x - a.position.x;
    this.rAy = point.y - a.position.y;
    this.rBx = point.x - b.position.x;
    this.rBy = point.y - b.position.y;
    this.normalImpulse = manifold.normalImpulses[pointIndex];
    this.tangentImpulse = manifold.tangentImpulses[pointIndex];
    this.blockPartner = null;
    this.isBlockFollower = false;

    this.normalMass = this.effectiveMass(this.nx, this.ny);
    this.tangentMass = this.effectiveMass(this.ny, -this.nx);

    const approachSpeed = this.velocityAlong(this.nx, this.ny);
    const bounces = approachSpeed < -RESTITUTION_THRESHOLD;
    this.restitutionBias = bounces ? -Math.max(a.restitution, b.restitution) * approachSpeed : 0;
  }

  // Re-applies last step's impulses so the solver starts near the answer.
  warmStart(): void {
    const tx = this.ny;
    const ty = -this.nx;
    this.applyImpulse(this.nx * this.normalImpulse + tx * this.tangentImpulse, this.ny * this.normalImpulse + ty * this.tangentImpulse);
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

  // Pairs this point with the manifold's second point, unless their coupling is too ill-conditioned to invert.
  linkBlock(other: ContactConstraint): void {
    const { bodyA: a, bodyB: b } = this;
    const crossA1 = this.rAx * this.ny - this.rAy * this.nx;
    const crossB1 = this.rBx * this.ny - this.rBy * this.nx;
    const crossA2 = other.rAx * this.ny - other.rAy * this.nx;
    const crossB2 = other.rBx * this.ny - other.rBy * this.nx;
    const mass = a.invMass + b.invMass;
    this.k11 = mass + a.invInertia * crossA1 * crossA1 + b.invInertia * crossB1 * crossB1;
    this.k22 = mass + a.invInertia * crossA2 * crossA2 + b.invInertia * crossB2 * crossB2;
    this.k12 = mass + a.invInertia * crossA1 * crossA2 + b.invInertia * crossB1 * crossB2;
    const det = this.k11 * this.k22 - this.k12 * this.k12;
    if (this.k11 * this.k11 >= MAX_BLOCK_CONDITION * det) return;
    this.inverseK11 = this.k22 / det;
    this.inverseK12 = -this.k12 / det;
    this.inverseK22 = this.k11 / det;
    this.blockPartner = other;
    other.isBlockFollower = true;
  }

  // Solves both points' normal impulses at once as a small LCP: try both active, then each alone, then neither.
  solveNormalBlock(other: ContactConstraint): void {
    const a1 = this.normalImpulse;
    const a2 = other.normalImpulse;
    // Velocity error with the accumulated impulses taken out.
    const b1 = this.velocityAlong(this.nx, this.ny) - this.restitutionBias - (this.k11 * a1 + this.k12 * a2);
    const b2 = other.velocityAlong(this.nx, this.ny) - other.restitutionBias - (this.k12 * a1 + this.k22 * a2);

    const bothA = -(this.inverseK11 * b1 + this.inverseK12 * b2);
    const bothB = -(this.inverseK12 * b1 + this.inverseK22 * b2);
    if (bothA >= 0 && bothB >= 0) return this.applyBlock(other, bothA, bothB);

    const onlyA = -b1 / this.k11;
    if (onlyA >= 0 && this.k12 * onlyA + b2 >= 0) return this.applyBlock(other, onlyA, 0);

    const onlyB = -b2 / this.k22;
    if (onlyB >= 0 && this.k12 * onlyB + b1 >= 0) return this.applyBlock(other, 0, onlyB);

    if (b1 >= 0 && b2 >= 0) this.applyBlock(other, 0, 0);
  }

  storeImpulses(): void {
    this.manifold.normalImpulses[this.pointIndex] = this.normalImpulse;
    this.manifold.tangentImpulses[this.pointIndex] = this.tangentImpulse;
  }

  // Moves the bodies apart along the normal without touching velocity, so no energy is added.
  correctPosition(): void {
    const { bodyA: a, bodyB: b } = this;
    // Corrections only translate, so the current overlap is the detected one minus the relative shift along n.
    const shifted = (b.position.x - this.startBx - (a.position.x - this.startAx)) * this.nx + (b.position.y - this.startBy - (a.position.y - this.startAy)) * this.ny;
    const excess = this.depth - shifted - PENETRATION_SLOP;
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

  private applyBlock(other: ContactConstraint, impulseA: number, impulseB: number): void {
    const deltaA = impulseA - this.normalImpulse;
    const deltaB = impulseB - other.normalImpulse;
    this.normalImpulse = impulseA;
    other.normalImpulse = impulseB;
    this.applyImpulse(this.nx * deltaA, this.ny * deltaA);
    other.applyImpulse(this.nx * deltaB, this.ny * deltaB);
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
