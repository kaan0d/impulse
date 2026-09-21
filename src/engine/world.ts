import type { Body } from './body';
import { PAIR_KEY_STRIDE, SpatialHash } from './broadphase';
import { sweepAgainstStatics } from './ccd';
import { collide } from './collide';
import type { Joint } from './joint';
import { Manifold } from './manifold';
import { queryAABB, raycast, type RayHit } from './query';
import { Sleeper } from './sleeper';
import { Solver } from './solver';
import { Vec2 } from './vec2';

// Grid cell edge in metres; about twice the size of a typical body.
const BROADPHASE_CELL_SIZE = 2;

export class World {
  readonly gravity = new Vec2(0, -9.81);
  readonly bodies: Body[] = [];
  readonly joints: Joint[] = [];
  // Contacts of the last detection; only the first `manifoldCount` entries are valid.
  readonly manifolds: Manifold[] = [];
  manifoldCount = 0;
  // Stops fast bodies from tunneling through static ones; costs one sweep per fast body per step.
  continuousCollision = true;

  private readonly staticBodies: Body[] = [];
  private readonly solver = new Solver();
  private readonly sleeper = new Sleeper();
  private readonly broadphase = new SpatialHash(BROADPHASE_CELL_SIZE);
  // Manifolds of touching pairs live here between steps so their impulses can warm start the solver.
  private readonly pairs = new Map<number, Manifold>();
  // Pair keys of jointed bodies with how many joints join each pair; these pairs never collide, so links can overlap.
  private readonly jointedPairs = new Map<number, number>();
  private readonly freeManifolds: Manifold[] = [];
  // Manifold offered to the next unknown pair; adopted into `pairs` only if that pair touches.
  private spare: Manifold | undefined;

  add(body: Body): Body {
    body.id = this.bodies.length;
    this.bodies.push(body);
    if (body.invMass === 0) this.staticBodies.push(body);
    return body;
  }

  // Topmost (most recently added) movable body under the point, or null. Static bodies are ignored.
  bodyAt(x: number, y: number): Body | null {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      const body = this.bodies[i];
      if (body.invMass !== 0 && body.containsPoint(x, y)) return body;
    }
    return null;
  }

  // Collidable bodies whose bounding box overlaps the given box, static ones included.
  boundsOverlapping(minX: number, minY: number, maxX: number, maxY: number, out: Body[] = []): Body[] {
    return queryAABB(this.bodies, minX, minY, maxX, maxY, out);
  }

  // Closest body the segment crosses whose category is in `mask`, or null. Bodies holding the start are skipped.
  raycast(x0: number, y0: number, x1: number, y1: number, mask = 0xffffffff): RayHit | null {
    return raycast(this.bodies, x0, y0, x1, y1, mask);
  }

  // Removes every body and joint. Settings such as gravity and continuousCollision are kept.
  clear(): void {
    this.bodies.length = 0;
    this.joints.length = 0;
    this.staticBodies.length = 0;
    this.manifoldCount = 0;
    this.pairs.clear();
    this.jointedPairs.clear();
    this.spare = undefined;
  }

  // Both bodies must already be in this world.
  addJoint(joint: Joint): Joint {
    this.joints.push(joint);
    const key = jointPairKey(joint);
    this.jointedPairs.set(key, (this.jointedPairs.get(key) ?? 0) + 1);
    return joint;
  }

  // The bodies wake, since whatever the joint was holding up is gone, and may collide with each other again.
  removeJoint(joint: Joint): void {
    const index = this.joints.indexOf(joint);
    if (index < 0) return;
    this.joints.splice(index, 1);
    wake(joint.bodyA);
    wake(joint.bodyB);
    const key = jointPairKey(joint);
    const remaining = (this.jointedPairs.get(key) ?? 1) - 1;
    if (remaining > 0) this.jointedPairs.set(key, remaining);
    else this.jointedPairs.delete(key);
  }

  // Detects contacts once, lets the solver run its substeps, then catches fast bodies and updates sleep.
  step(dt: number): void {
    this.detectCollisions();
    this.solver.prepare(this.bodies, this.manifolds, this.manifoldCount, dt);
    this.solver.solve(this.bodies, this.gravity, this.joints, dt);
    if (this.continuousCollision) {
      for (const body of this.bodies) {
        if (body.isSimulated) sweepAgainstStatics(body, body.startX, body.startY, this.staticBodies);
      }
    }
    this.sleeper.update(this.bodies, this.manifolds, this.manifoldCount, this.joints, dt);
  }

  detectCollisions(): void {
    // A body woken mid-pass may touch neighbors the pass already skipped, so repeat until nobody wakes.
    do this.wakeJointedBodies();
    while (this.detectPass());
  }

  // A sleeping body follows its joint partner, and so does a chain of them.
  private wakeJointedBodies(): void {
    let changed = true;
    while (changed) {
      changed = false;
      for (const { bodyA, bodyB } of this.joints) {
        if (bodyA.isSimulated) changed = wake(bodyB) || changed;
        else if (bodyB.isSimulated) changed = wake(bodyA) || changed;
      }
    }
  }

  // Narrowphase over the broadphase pairs, in ascending (idA, idB) order. Returns true if a body woke.
  private detectPass(): boolean {
    const { bodies, manifolds, pairs } = this;
    const pairCount = this.broadphase.findPairs(bodies);
    const keys = this.broadphase.pairKeys;
    let count = 0;
    let woke = false;
    for (let k = 0; k < pairCount; k++) {
      const key = keys[k];
      if (this.jointedPairs.has(key)) continue;
      const idB = key % PAIR_KEY_STRIDE;
      const a = bodies[(key - idB) / PAIR_KEY_STRIDE];
      const b = bodies[idB];
      if (!a.collidable || !b.collidable || !a.canCollideWith(b)) continue;

      const known = pairs.get(key);
      const manifold = known ?? this.takeSpare(a, b);
      known?.savePrevious();
      if (!collide(a, b, manifold)) {
        if (known) this.release(key, known);
        continue;
      }
      if (!known) {
        pairs.set(key, manifold);
        this.spare = undefined;
      }
      manifold.carryImpulses();
      const wokeA = wake(a);
      const wokeB = wake(b);
      woke = woke || wokeA || wokeB;
      manifolds[count++] = manifold;
    }
    this.manifoldCount = count;
    return woke;
  }

  private takeSpare(a: Body, b: Body): Manifold {
    this.spare ??= this.freeManifolds.pop() ?? new Manifold(a, b);
    return this.spare;
  }

  private release(key: number, manifold: Manifold): void {
    this.pairs.delete(key);
    manifold.reset();
    this.freeManifolds.push(manifold);
  }
}

function jointPairKey({ bodyA, bodyB }: Joint): number {
  return Math.min(bodyA.id, bodyB.id) * PAIR_KEY_STRIDE + Math.max(bodyA.id, bodyB.id);
}

function wake(body: Body): boolean {
  if (body.awake) return false;
  body.awake = true;
  body.sleepTime = 0;
  return true;
}
