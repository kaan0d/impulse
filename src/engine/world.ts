import type { Body } from './body';
import { collide } from './collide';
import { Manifold } from './manifold';
import { Sleeper } from './sleeper';
import { ContactSolver } from './solver';
import { Vec2 } from './vec2';

// Pair key = idA * stride + idB. 2^26 keeps the key an exact integer for up to 67M bodies.
const PAIR_KEY_STRIDE = 2 ** 26;

export class World {
  readonly gravity = new Vec2(0, -9.81);
  readonly bodies: Body[] = [];
  // Contacts of the last detection; only the first `manifoldCount` entries are valid.
  readonly manifolds: Manifold[] = [];
  manifoldCount = 0;

  private readonly solver = new ContactSolver();
  private readonly sleeper = new Sleeper();
  // Manifolds of touching pairs live here between steps so their impulses can warm start the solver.
  private readonly pairs = new Map<number, Manifold>();
  private readonly freeManifolds: Manifold[] = [];
  // Manifold offered to the next unknown pair; adopted into `pairs` only if that pair touches.
  private spare: Manifold | undefined;

  add(body: Body): Body {
    body.id = this.bodies.length;
    this.bodies.push(body);
    return body;
  }

  // Semi-implicit Euler: contacts, gravity into velocity, contact solve, then position from the new velocity.
  step(dt: number): void {
    this.detectCollisions();
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      body.velocity.addScaled(this.gravity, dt);
    }
    this.solver.solve(this.manifolds, this.manifoldCount);
    for (const body of this.bodies) {
      if (!body.isSimulated) continue;
      body.position.addScaled(body.velocity, dt);
      body.angle += body.angularVelocity * dt;
    }
    this.sleeper.update(this.bodies, this.manifolds, this.manifoldCount, dt);
  }

  detectCollisions(): void {
    // A body woken mid-pass may touch neighbors the pass already skipped, so repeat until nobody wakes.
    while (this.detectPass()) continue;
  }

  // Brute-force O(n^2) pairs; the stage 5 broadphase replaces this loop. Returns true if a body woke.
  private detectPass(): boolean {
    const { bodies, manifolds, pairs } = this;
    let count = 0;
    let woke = false;
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const a = bodies[i];
        const b = bodies[j];
        if (!a.isSimulated && !b.isSimulated) continue;

        const key = a.id * PAIR_KEY_STRIDE + b.id;
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
