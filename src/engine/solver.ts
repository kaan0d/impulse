import type { Body } from './body';
import type { Joint } from './joint';
import { clamp } from './math';
import type { Manifold } from './manifold';
import type { Vec2 } from './vec2';

// Each step is split into this many substeps; contact impulses are stored per substep.
export const SUBSTEPS = 4;
// Slower impacts are treated as inelastic, so resting contacts do not jitter.
const RESTITUTION_THRESHOLD = 1;
// Two-point contacts need a few sweeps for the bounce to reach the requested speed.
const RESTITUTION_PASSES = 6;
// Contact spring: stiff enough to hold stacks, soft enough to absorb one step of overlap smoothly.
const CONTACT_HERTZ = 40;
const CONTACT_DAMPING_RATIO = 10;
// Static geometry cannot give way, so its contacts are stiffer.
const STATIC_STIFFNESS_SCALE = 2;
// Fastest speed at which overlap is pushed apart, so deep overlaps do not explode.
const MAX_PUSH_SPEED = 3;

interface Softness {
  biasRate: number;
  massScale: number;
  impulseScale: number;
}

/// Soft-step solver for contacts and joints: substeps of gravity, warm start, biased solve, position update
/// and an unbiased relax pass, then restitution. Pools its constraints; no per-step allocation.
export class Solver {
  private readonly constraints: ContactConstraint[] = [];
  private count = 0;
  private readonly dynamicSoftness: Softness = { biasRate: 0, massScale: 1, impulseScale: 0 };
  private readonly staticSoftness: Softness = { biasRate: 0, massScale: 1, impulseScale: 0 };

  // Reads the incoming velocities into the contacts; call before `solve`, so restitution sees the impact speed.
  prepare(bodies: Body[], manifolds: Manifold[], manifoldCount: number, dt: number): void {
    const substep = dt / SUBSTEPS;
    for (const body of bodies) body.resetMotion();
    setSoftness(this.dynamicSoftness, CONTACT_HERTZ, CONTACT_DAMPING_RATIO, substep);
    setSoftness(this.staticSoftness, CONTACT_HERTZ * STATIC_STIFFNESS_SCALE, CONTACT_DAMPING_RATIO, substep);

    let count = 0;
    for (let i = 0; i < manifoldCount; i++) {
      const manifold = manifolds[i];
      for (let j = 0; j < manifold.count; j++) {
        let constraint: ContactConstraint | undefined = this.constraints[count];
        if (!constraint) {
          constraint = new ContactConstraint(manifold);
          this.constraints.push(constraint);
        }
        const isStatic = manifold.bodyA.invMass === 0 || manifold.bodyB.invMass === 0;
        constraint.prepare(manifold, j, isStatic ? this.staticSoftness : this.dynamicSoftness, 1 / substep);
        count++;
      }
    }
    this.count = count;
  }

  // Advances velocities and positions by `dt` with gravity, joints and contacts. Ends with the bounce pass.
  // Accumulated impulses persist in the manifolds and joints and warm start the next step.
  solve(bodies: Body[], gravity: Vec2, joints: Joint[], dt: number): void {
    const substep = dt / SUBSTEPS;
    const { constraints, count } = this;
    for (let i = 0; i < SUBSTEPS; i++) {
      for (const joint of joints) joint.prepare(1 / substep);
      for (const body of bodies) if (body.isSimulated) body.integrateVelocity(gravity, substep);
      for (const joint of joints) joint.warmStart();
      for (let c = 0; c < count; c++) constraints[c].warmStart();
      this.iterate(joints, true);

      for (const body of bodies) if (body.isSimulated) body.advance(substep);
      // Joint anchors moved with the bodies, so re-read them before the relax pass.
      for (const joint of joints) joint.prepare(1 / substep);
      this.iterate(joints, false);
    }
    for (let pass = 0; pass < RESTITUTION_PASSES; pass++) {
      for (let c = 0; c < count; c++) constraints[c].applyRestitution();
    }
    for (let c = 0; c < count; c++) constraints[c].storeImpulses();
  }

  // With `useBias` the solve also pushes overlap and joint error out; without it bias energy is removed.
  private iterate(joints: Joint[], useBias: boolean): void {
    const { constraints, count } = this;
    for (const joint of joints) joint.solve(useBias);
    for (let c = 0; c < count; c++) constraints[c].solve(useBias);
  }
}

// Soft-constraint coefficients for a spring of the given stiffness and damping at substep length `h`.
function setSoftness(out: Softness, hertz: number, dampingRatio: number, h: number): void {
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * dampingRatio + h * omega;
  const a2 = h * omega * a1;
  const a3 = 1 / (1 + a2);
  out.biasRate = omega / a1;
  out.massScale = a2 * a3;
  out.impulseScale = a3;
}

/// One contact point: a soft non-penetration constraint plus a Coulomb friction constraint.
class ContactConstraint {
  private bodyA: Body;
  private bodyB: Body;
  private pointIndex = 0;
  // Normal points from bodyA toward bodyB; the friction tangent is (ny, -nx).
  private nx = 0;
  private ny = 0;
  // Overlap when the step began; the current separation follows from how far the anchors have moved since.
  private depth = 0;
  private friction = 0;
  private restitution = 0;
  // Largest torque per unit of normal impulse that may resist relative spin.
  private rollingResistance = 0;
  private rollingMass = 0;
  private rollingImpulse = 0;
  // Contact offsets from each body's center of mass, fixed for the step.
  private rAx = 0;
  private rAy = 0;
  private rBx = 0;
  private rBy = 0;
  // A circle's contact point stays under its center as it spins, so its anchor must not turn with it.
  private turnsA = true;
  private turnsB = true;
  private normalMass = 0;
  private tangentMass = 0;
  // Normal speed before gravity and solving; negative means approaching. Drives the bounce.
  private approachSpeed = 0;
  private invSubstep = 0;
  private softness!: Softness;
  // Accumulated over the substeps and clamped, never the per-iteration delta.
  private normalImpulse = 0;
  private tangentImpulse = 0;
  // Largest normal impulse seen this step; zero means the bodies never truly touched.
  private maxNormalImpulse = 0;

  constructor(private manifold: Manifold) {
    this.bodyA = manifold.bodyA;
    this.bodyB = manifold.bodyB;
  }

  prepare(manifold: Manifold, pointIndex: number, softness: Softness, invSubstep: number): void {
    const a = manifold.bodyA;
    const b = manifold.bodyB;
    const point = manifold.points[pointIndex];
    this.manifold = manifold;
    this.pointIndex = pointIndex;
    this.bodyA = a;
    this.bodyB = b;
    this.softness = softness;
    this.invSubstep = invSubstep;
    this.nx = manifold.normal.x;
    this.ny = manifold.normal.y;
    this.depth = manifold.depths[pointIndex];
    this.turnsA = a.shape.kind !== 'circle';
    this.turnsB = b.shape.kind !== 'circle';
    this.friction = Math.sqrt(a.friction * b.friction);
    this.restitution = Math.max(a.restitution, b.restitution);
    this.rollingResistance = Math.max(a.rollingResistance, b.rollingResistance);
    this.rollingMass = 1 / (a.invInertia + b.invInertia);
    this.rollingImpulse = manifold.rollingImpulses[pointIndex];
    this.rAx = point.x - a.position.x;
    this.rAy = point.y - a.position.y;
    this.rBx = point.x - b.position.x;
    this.rBy = point.y - b.position.y;
    this.normalImpulse = manifold.normalImpulses[pointIndex];
    this.tangentImpulse = manifold.tangentImpulses[pointIndex];
    this.maxNormalImpulse = 0;

    this.normalMass = this.effectiveMass(this.nx, this.ny);
    this.tangentMass = this.effectiveMass(this.ny, -this.nx);
    this.approachSpeed = this.velocityAlong(this.nx, this.ny);
  }

  // Re-applies last substep's impulses so the solver starts near the answer.
  warmStart(): void {
    this.applySpin(this.rollingImpulse);
    const tx = this.ny;
    const ty = -this.nx;
    this.applyImpulse(this.nx * this.normalImpulse + tx * this.tangentImpulse, this.ny * this.normalImpulse + ty * this.tangentImpulse);
  }

  solve(useBias: boolean): void {
    if (this.rollingResistance > 0) this.solveRolling();
    this.solveFriction();
    this.solveNormal(useBias);
  }

  // Lets a bounce happen once the substeps have settled the contact, using the speed from before the solve.
  applyRestitution(): void {
    if (this.restitution === 0 || this.approachSpeed > -RESTITUTION_THRESHOLD || this.maxNormalImpulse === 0) return;
    const impulse = -this.normalMass * (this.velocityAlong(this.nx, this.ny) + this.restitution * this.approachSpeed);
    const next = Math.max(this.normalImpulse + impulse, 0);
    const applied = next - this.normalImpulse;
    this.normalImpulse = next;
    this.applyImpulse(this.nx * applied, this.ny * applied);
  }

  storeImpulses(): void {
    this.manifold.normalImpulses[this.pointIndex] = this.normalImpulse;
    this.manifold.tangentImpulses[this.pointIndex] = this.tangentImpulse;
    this.manifold.rollingImpulses[this.pointIndex] = this.rollingImpulse;
  }

  // Brakes relative spin with a torque capped by the normal force, like a wheel sinking slightly into the ground.
  private solveRolling(): void {
    const { bodyA: a, bodyB: b } = this;
    const impulse = -this.rollingMass * (b.angularVelocity - a.angularVelocity);
    const limit = this.rollingResistance * this.normalImpulse;
    const next = clamp(this.rollingImpulse + impulse, -limit, limit);
    const applied = next - this.rollingImpulse;
    this.rollingImpulse = next;
    this.applySpin(applied);
  }

  private applySpin(impulse: number): void {
    this.bodyA.angularVelocity -= this.bodyA.invInertia * impulse;
    this.bodyB.angularVelocity += this.bodyB.invInertia * impulse;
  }

  private solveFriction(): void {
    const tx = this.ny;
    const ty = -this.nx;
    const impulse = -this.tangentMass * this.velocityAlong(tx, ty);
    const limit = this.friction * this.normalImpulse;
    const next = clamp(this.tangentImpulse + impulse, -limit, limit);
    const applied = next - this.tangentImpulse;
    this.tangentImpulse = next;
    this.applyImpulse(tx * applied, ty * applied);
  }

  private solveNormal(useBias: boolean): void {
    const separation = this.separation();
    let bias = 0;
    let massScale = 1;
    let impulseScale = 0;
    if (separation > 0) {
      // Still apart: allow the bodies to close the gap this substep but not to cross it.
      bias = separation * this.invSubstep;
    } else if (useBias) {
      bias = Math.max(this.softness.biasRate * separation, -MAX_PUSH_SPEED);
      massScale = this.softness.massScale;
      impulseScale = this.softness.impulseScale;
    }
    const impulse = -this.normalMass * massScale * (this.velocityAlong(this.nx, this.ny) + bias) - impulseScale * this.normalImpulse;
    const next = Math.max(this.normalImpulse + impulse, 0);
    const applied = next - this.normalImpulse;
    this.normalImpulse = next;
    this.maxNormalImpulse = Math.max(this.maxNormalImpulse, next);
    this.applyImpulse(this.nx * applied, this.ny * applied);
  }

  // Gap along the normal: negative when overlapping. Anchors move with each body's position and turn.
  private separation(): number {
    const { bodyA: a, bodyB: b } = this;
    let movedX = b.position.x - b.startX - (a.position.x - a.startX);
    let movedY = b.position.y - b.startY - (a.position.y - a.startY);
    if (this.turnsB) {
      movedX += b.deltaCos * this.rBx - b.deltaSin * this.rBy - this.rBx;
      movedY += b.deltaSin * this.rBx + b.deltaCos * this.rBy - this.rBy;
    }
    if (this.turnsA) {
      movedX -= a.deltaCos * this.rAx - a.deltaSin * this.rAy - this.rAx;
      movedY -= a.deltaSin * this.rAx + a.deltaCos * this.rAy - this.rAy;
    }
    return -this.depth + movedX * this.nx + movedY * this.ny;
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
