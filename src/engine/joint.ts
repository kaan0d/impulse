import type { Body } from './body';
import { clamp } from './math';
import { Vec2 } from './vec2';

// Fraction of the anchor error removed per step, fed back through the velocity bias (Baumgarte).
const BIAS_FACTOR = 0.2;

/// Two-anchor constraint between bodies, solved by sequential impulses next to the contacts.
/// Anchors are given in world space at creation and stored in each body's local frame.
/// Bodies must already belong to the world. Pair a static body with a dynamic one to pin it in place.
export abstract class Joint {
  // Skipped for the step when neither body is simulated.
  protected active = false;
  // 1 / step length, set by `beginStep`.
  protected invDt = 0;
  // Anchor offsets from each body's center in world axes, refreshed every step.
  protected rAx = 0;
  protected rAy = 0;
  protected rBx = 0;
  protected rBy = 0;
  // Anchor B minus anchor A, and the anchors' relative velocity; refreshed by the helpers below.
  protected readonly gap = new Vec2();
  protected readonly relativeVelocity = new Vec2();
  private readonly localA = new Vec2();
  private readonly localB = new Vec2();

  protected constructor(
    readonly bodyA: Body,
    readonly bodyB: Body,
    worldAnchorA: Vec2,
    worldAnchorB: Vec2,
  ) {
    toLocal(bodyA, worldAnchorA, this.localA);
    toLocal(bodyB, worldAnchorB, this.localB);
  }

  // Once per step, before any impulse is applied. `invDt` is 1 / step length.
  abstract prepare(invDt: number): void;
  // Re-applies last step's accumulated impulse.
  abstract warmStart(): void;
  // One iteration. Without `useBias` error-correcting bias is left out, so the relax pass adds no energy.
  abstract solve(useBias: boolean): void;

  anchorA(out: Vec2): Vec2 {
    return toWorld(this.bodyA, this.localA, out);
  }

  anchorB(out: Vec2): Vec2 {
    return toWorld(this.bodyB, this.localB, out);
  }

  // Sets `active` and the anchor offsets; returns false if the joint sits this step out.
  protected beginStep(invDt: number): boolean {
    const { bodyA: a, bodyB: b } = this;
    this.invDt = invDt;
    this.active = a.isSimulated || b.isSimulated;
    if (!this.active) return false;
    this.rAx = Math.cos(a.angle) * this.localA.x - Math.sin(a.angle) * this.localA.y;
    this.rAy = Math.sin(a.angle) * this.localA.x + Math.cos(a.angle) * this.localA.y;
    this.rBx = Math.cos(b.angle) * this.localB.x - Math.sin(b.angle) * this.localB.y;
    this.rBy = Math.sin(b.angle) * this.localB.x + Math.cos(b.angle) * this.localB.y;
    return true;
  }

  protected updateGap(): void {
    const { bodyA: a, bodyB: b } = this;
    this.gap.set(b.position.x + this.rBx - (a.position.x + this.rAx), b.position.y + this.rBy - (a.position.y + this.rAy));
  }

  protected updateRelativeVelocity(): void {
    const { bodyA: a, bodyB: b } = this;
    this.relativeVelocity.set(
      b.velocity.x - b.angularVelocity * this.rBy - (a.velocity.x - a.angularVelocity * this.rAy),
      b.velocity.y + b.angularVelocity * this.rBx - (a.velocity.y + a.angularVelocity * this.rAx),
    );
  }

  // Pushes bodyB along (px, py) and bodyA the opposite way, at the anchors.
  protected applyImpulse(px: number, py: number): void {
    const { bodyA: a, bodyB: b } = this;
    a.velocity.x -= a.invMass * px;
    a.velocity.y -= a.invMass * py;
    a.angularVelocity -= a.invInertia * (this.rAx * py - this.rAy * px);
    b.velocity.x += b.invMass * px;
    b.velocity.y += b.invMass * py;
    b.angularVelocity += b.invInertia * (this.rBx * py - this.rBy * px);
  }
}

/// Pin: both anchors are held at the same world point, the bodies may rotate freely around it.
/// Optional angle limits and a motor act on the relative rotation of the two bodies.
export class RevoluteJoint extends Joint {
  // Relative rotation when the joint was made; `angle` is measured from it.
  private readonly referenceAngle: number;
  private limitsEnabled = false;
  private lowerAngle = 0;
  private upperAngle = 0;
  private motorEnabled = false;
  private motorSpeed = 0;
  private maxMotorTorque = 0;
  private axialMass = 0;
  private currentAngle = 0;
  private maxMotorImpulse = 0;
  // Accumulated angular impulses; the limit ones are never negative.
  private motorImpulse = 0;
  private lowerImpulse = 0;
  private upperImpulse = 0;
  // Inverse of the 2x2 effective mass matrix of the pin (symmetric).
  private inverseMass11 = 0;
  private inverseMass12 = 0;
  private inverseMass22 = 0;
  private biasX = 0;
  private biasY = 0;
  private impulseX = 0;
  private impulseY = 0;

  constructor(bodyA: Body, bodyB: Body, worldAnchor: Vec2) {
    super(bodyA, bodyB, worldAnchor, worldAnchor);
    this.referenceAngle = bodyB.angle - bodyA.angle;
  }

  // Rotation of bodyB relative to bodyA, in radians, zero at creation.
  get angle(): number {
    return this.bodyB.angle - this.bodyA.angle - this.referenceAngle;
  }

  // Keeps `angle` within [lower, upper]. The bodies may cross the limit briefly at high speed.
  setLimits(lower: number, upper: number): void {
    if (lower > upper) throw new RangeError(`lower limit ${lower} is above upper limit ${upper}`);
    this.limitsEnabled = true;
    this.lowerAngle = lower;
    this.upperAngle = upper;
  }

  clearLimits(): void {
    this.limitsEnabled = false;
    this.lowerImpulse = 0;
    this.upperImpulse = 0;
  }

  // Drives the relative spin toward `speed` (rad/s) with at most `maxTorque` (N m).
  setMotor(speed: number, maxTorque: number): void {
    this.motorEnabled = true;
    this.motorSpeed = speed;
    this.maxMotorTorque = maxTorque;
  }

  clearMotor(): void {
    this.motorEnabled = false;
    this.motorImpulse = 0;
  }

  prepare(invDt: number): void {
    if (!this.beginStep(invDt)) return;
    const { bodyA: a, bodyB: b, rAx, rAy, rBx, rBy } = this;
    const mass = a.invMass + b.invMass;
    const k11 = mass + a.invInertia * rAy * rAy + b.invInertia * rBy * rBy;
    const k12 = -a.invInertia * rAx * rAy - b.invInertia * rBx * rBy;
    const k22 = mass + a.invInertia * rAx * rAx + b.invInertia * rBx * rBx;
    const det = k11 * k22 - k12 * k12;
    this.inverseMass11 = k22 / det;
    this.inverseMass12 = -k12 / det;
    this.inverseMass22 = k11 / det;

    this.updateGap();
    this.biasX = BIAS_FACTOR * invDt * this.gap.x;
    this.biasY = BIAS_FACTOR * invDt * this.gap.y;

    const inertia = a.invInertia + b.invInertia;
    this.axialMass = inertia > 0 ? 1 / inertia : 0;
    this.currentAngle = this.angle;
    this.maxMotorImpulse = this.maxMotorTorque / invDt;
  }

  warmStart(): void {
    if (!this.active) return;
    this.applySpinImpulse(this.motorImpulse + this.lowerImpulse - this.upperImpulse);
    this.applyImpulse(this.impulseX, this.impulseY);
  }

  solve(useBias: boolean): void {
    if (!this.active) return;
    if (this.motorEnabled) this.solveMotor();
    if (this.limitsEnabled) {
      this.solveLowerLimit(useBias);
      this.solveUpperLimit(useBias);
    }
    this.solvePin(useBias);
  }

  private solveMotor(): void {
    const impulse = -this.axialMass * (this.relativeSpin() - this.motorSpeed);
    const next = clamp(this.motorImpulse + impulse, -this.maxMotorImpulse, this.maxMotorImpulse);
    this.applySpinImpulse(next - this.motorImpulse);
    this.motorImpulse = next;
  }

  // Inside the limit the bias lets the bodies close up to it; past the limit it pushes them back.
  private solveLowerLimit(useBias: boolean): void {
    const error = this.currentAngle - this.lowerAngle;
    const bias = (Math.max(error, 0) + (useBias ? BIAS_FACTOR * Math.min(error, 0) : 0)) * this.invDt;
    const impulse = -this.axialMass * (this.relativeSpin() + bias);
    const next = Math.max(this.lowerImpulse + impulse, 0);
    this.applySpinImpulse(next - this.lowerImpulse);
    this.lowerImpulse = next;
  }

  private solveUpperLimit(useBias: boolean): void {
    const error = this.upperAngle - this.currentAngle;
    const bias = (Math.max(error, 0) + (useBias ? BIAS_FACTOR * Math.min(error, 0) : 0)) * this.invDt;
    const impulse = -this.axialMass * (-this.relativeSpin() + bias);
    const next = Math.max(this.upperImpulse + impulse, 0);
    this.applySpinImpulse(-(next - this.upperImpulse));
    this.upperImpulse = next;
  }

  private solvePin(useBias: boolean): void {
    this.updateRelativeVelocity();
    const targetX = -(this.relativeVelocity.x + (useBias ? this.biasX : 0));
    const targetY = -(this.relativeVelocity.y + (useBias ? this.biasY : 0));
    const px = this.inverseMass11 * targetX + this.inverseMass12 * targetY;
    const py = this.inverseMass12 * targetX + this.inverseMass22 * targetY;
    this.impulseX += px;
    this.impulseY += py;
    this.applyImpulse(px, py);
  }

  private relativeSpin(): number {
    return this.bodyB.angularVelocity - this.bodyA.angularVelocity;
  }

  // Spins bodyB one way and bodyA the other, without moving their centers.
  private applySpinImpulse(impulse: number): void {
    this.bodyA.angularVelocity -= this.bodyA.invInertia * impulse;
    this.bodyB.angularVelocity += this.bodyB.invInertia * impulse;
  }
}

/// Keeps the two anchors a fixed distance apart, the bodies may rotate freely about the anchors.
/// Rigid rod by default; `setSpring` softens it and `setRope` lets it go slack.
export class DistanceJoint extends Joint {
  // Unit vector from anchor A to anchor B, refreshed each step.
  private ux = 1;
  private uy = 0;
  private frequency = 0;
  private dampingRatio = 0;
  private isRope = false;
  private effectiveMass = 0;
  // Softness terms of the spring; zero for a rigid joint.
  private gamma = 0;
  private bias = 0;
  // Spring and slack-rope bias is physical, not error correction, so the relax pass keeps it.
  private biasIsPhysical = false;
  private impulse = 0;

  // `length` defaults to the anchors' current separation.
  constructor(
    bodyA: Body,
    bodyB: Body,
    worldAnchorA: Vec2,
    worldAnchorB: Vec2,
    private readonly length = worldAnchorB.clone().sub(worldAnchorA).length(),
  ) {
    super(bodyA, bodyB, worldAnchorA, worldAnchorB);
  }

  // A spring instead of a rod: oscillates at `frequencyHz`, `dampingRatio` 0 is undamped and 1 is critical.
  setSpring(frequencyHz: number, dampingRatio: number): void {
    this.frequency = frequencyHz;
    this.dampingRatio = dampingRatio;
  }

  // Zero frequency makes the joint rigid again.
  clearSpring(): void {
    this.frequency = 0;
  }

  // A rope only pulls: it does nothing while the anchors are closer than `length`.
  setRope(enabled = true): void {
    this.isRope = enabled;
    if (!enabled) return;
    // Impulses stored while it acted as a rod would push, which a rope never does.
    this.impulse = Math.min(this.impulse, 0);
  }

  prepare(invDt: number): void {
    if (!this.beginStep(invDt)) return;
    const { bodyA: a, bodyB: b, rAx, rAy, rBx, rBy } = this;
    this.updateGap();
    const distance = this.gap.length();
    // Coincident anchors have no direction, so pick +x.
    this.ux = distance > 0 ? this.gap.x / distance : 1;
    this.uy = distance > 0 ? this.gap.y / distance : 0;

    const crossA = rAx * this.uy - rAy * this.ux;
    const crossB = rBx * this.uy - rBy * this.ux;
    const inverseMass = a.invMass + b.invMass + a.invInertia * crossA * crossA + b.invInertia * crossB * crossB;
    const stretch = distance - this.length;
    const slack = this.isRope && stretch < 0;

    if (this.frequency > 0 && !slack) {
      // Soft constraint: gamma and the bias follow from the spring's stiffness and damping at this step size.
      const dt = 1 / invDt;
      const mass = 1 / inverseMass;
      const omega = 2 * Math.PI * this.frequency;
      const stiffness = mass * omega * omega;
      const damping = 2 * mass * this.dampingRatio * omega;
      const softness = dt * (damping + dt * stiffness);
      this.gamma = softness > 0 ? 1 / softness : 0;
      this.bias = stretch * dt * stiffness * this.gamma;
      this.biasIsPhysical = true;
    } else {
      this.gamma = 0;
      // A slack rope may close the gap this step but not overshoot it; otherwise pull the error out gradually.
      this.bias = slack ? stretch * invDt : BIAS_FACTOR * invDt * stretch;
      this.biasIsPhysical = slack;
    }
    this.effectiveMass = 1 / (inverseMass + this.gamma);
  }

  warmStart(): void {
    if (this.active) this.applyImpulse(this.ux * this.impulse, this.uy * this.impulse);
  }

  solve(useBias: boolean): void {
    if (!this.active) return;
    this.updateRelativeVelocity();
    const speed = this.relativeVelocity.x * this.ux + this.relativeVelocity.y * this.uy;
    const bias = useBias || this.biasIsPhysical ? this.bias : 0;
    const lambda = -this.effectiveMass * (speed + bias + this.gamma * this.impulse);
    const next = this.isRope ? Math.min(this.impulse + lambda, 0) : this.impulse + lambda;
    const applied = next - this.impulse;
    this.impulse = next;
    this.applyImpulse(this.ux * applied, this.uy * applied);
  }
}

/// Drags a body's anchor toward a target with a soft spring, capped at `maxForce`, for grabbing with a pointer.
/// `cursor` is a static body that marks the target: move it with `setTarget`. Give it `collidable = false`
/// so it does not push other bodies.
export class MouseJoint extends Joint {
  private gamma = 0;
  private inverseMass11 = 0;
  private inverseMass12 = 0;
  private inverseMass22 = 0;
  private biasX = 0;
  private biasY = 0;
  private maxImpulse = 0;
  private impulseX = 0;
  private impulseY = 0;

  constructor(
    cursor: Body,
    body: Body,
    worldAnchor: Vec2,
    private readonly maxForce: number,
    private readonly frequency = 5,
    private readonly dampingRatio = 0.7,
  ) {
    super(cursor, body, cursor.position, worldAnchor);
    this.wakeBody();
  }

  setTarget(x: number, y: number): void {
    this.bodyA.position.set(x, y);
    this.wakeBody();
  }

  prepare(invDt: number): void {
    if (!this.beginStep(invDt)) return;
    const { bodyA: a, bodyB: b, rAx, rAy, rBx, rBy } = this;
    const dt = 1 / invDt;
    const omega = 2 * Math.PI * this.frequency;
    const stiffness = b.mass * omega * omega;
    const damping = 2 * b.mass * this.dampingRatio * omega;
    const softness = dt * (damping + dt * stiffness);
    this.gamma = softness > 0 ? 1 / softness : 0;
    const beta = dt * stiffness * this.gamma;

    const mass = a.invMass + b.invMass;
    const k11 = mass + a.invInertia * rAy * rAy + b.invInertia * rBy * rBy + this.gamma;
    const k12 = -a.invInertia * rAx * rAy - b.invInertia * rBx * rBy;
    const k22 = mass + a.invInertia * rAx * rAx + b.invInertia * rBx * rBx + this.gamma;
    const det = k11 * k22 - k12 * k12;
    this.inverseMass11 = k22 / det;
    this.inverseMass12 = -k12 / det;
    this.inverseMass22 = k11 / det;

    this.updateGap();
    this.biasX = beta * this.gap.x;
    this.biasY = beta * this.gap.y;
    this.maxImpulse = this.maxForce * dt;
  }

  warmStart(): void {
    if (this.active) this.applyImpulse(this.impulseX, this.impulseY);
  }

  // A spring: its bias is physical, so the relax pass does not change it.
  solve(): void {
    if (!this.active) return;
    this.updateRelativeVelocity();
    const targetX = -(this.relativeVelocity.x + this.biasX + this.gamma * this.impulseX);
    const targetY = -(this.relativeVelocity.y + this.biasY + this.gamma * this.impulseY);
    let nextX = this.impulseX + this.inverseMass11 * targetX + this.inverseMass12 * targetY;
    let nextY = this.impulseY + this.inverseMass12 * targetX + this.inverseMass22 * targetY;
    const size = Math.hypot(nextX, nextY);
    if (size > this.maxImpulse) {
      nextX *= this.maxImpulse / size;
      nextY *= this.maxImpulse / size;
    }
    this.applyImpulse(nextX - this.impulseX, nextY - this.impulseY);
    this.impulseX = nextX;
    this.impulseY = nextY;
  }

  private wakeBody(): void {
    this.bodyB.awake = true;
    this.bodyB.sleepTime = 0;
  }
}

function toLocal(body: Body, world: Vec2, out: Vec2): void {
  const dx = world.x - body.position.x;
  const dy = world.y - body.position.y;
  const cos = Math.cos(body.angle);
  const sin = Math.sin(body.angle);
  out.set(cos * dx + sin * dy, -sin * dx + cos * dy);
}

function toWorld(body: Body, local: Vec2, out: Vec2): Vec2 {
  const cos = Math.cos(body.angle);
  const sin = Math.sin(body.angle);
  return out.set(body.position.x + cos * local.x - sin * local.y, body.position.y + sin * local.x + cos * local.y);
}
