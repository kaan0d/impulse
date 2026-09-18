import type { Body } from './body';
import { PAIR_KEY_STRIDE, SpatialHash } from './broadphase';
import { collide } from './collide';
import type { Joint } from './joint';
import { Manifold } from './manifold';
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

  private readonly solver = new Solver();
  private readonly sleeper = new Sleeper();
  private readonly broadphase = new SpatialHash(BROADPHASE_CELL_SIZE);
  // Manifolds of touching pairs live here between steps so their impulses can warm start the solver.
  private readonly pairs = new Map<number, Manifold>();
  // Pair keys of jointed bodies; those pairs never collide, so links can overlap.
  private readonly jointedPairs = new Set<number>();
  private readonly freeManifolds: Manifold[] = [];
  // Manifold offered to the next unknown pair; adopted into `pairs` only if that pair touches.
  private spare: Manifold | undefined;

  add(body: Body): Body {
    body.id = this.bodies.length;
    this.bodies.push(body);
    return body;
  }

  // Both bodies must already be in this world.
  addJoint(joint: Joint): Joint {
    this.joints.push(joint);
    const idA = joint.bodyA.id;
    const idB = joint.bodyB.id;
    this.jointedPairs.add(Math.min(idA, idB) * PAIR_KEY_STRIDE + Math.max(idA, idB));
    return joint;
  }

  // Semi-implicit Euler: contacts, gravity into velocity, contact solve, then position from the new velocity.
  step(dt: number): void {
    this.detectCollisions();
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      body.velocity.addScaled(this.gravity, dt);
    }
    this.solver.solve(this.manifolds, this.manifoldCount, this.joints, 1 / dt);
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      body.position.addScaled(body.velocity, dt);
      body.angle += body.angularVelocity * dt;
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

function wake(body: Body): boolean {
  if (body.awake) return false;
  body.awake = true;
  body.sleepTime = 0;
  return true;
}
