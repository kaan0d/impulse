import type { Body } from './body';
import type { Joint } from './joint';
import type { Manifold } from './manifold';

const SLEEP_LINEAR_SPEED = 0.02; // m/s
const SLEEP_ANGULAR_SPEED = 0.035; // rad/s
const TIME_TO_SLEEP = 0.5; // seconds a whole island must stay slow

/// Puts islands (groups of touching dynamic bodies) to sleep once every member has been still long enough.
export class Sleeper {
  // Union-find over body ids; grows only when bodies are added.
  private parent = new Int32Array(0);
  private islandStillTime = new Float64Array(0);

  // Call once per step after integration, with the step's active manifolds. Joints link bodies into islands too.
  update(bodies: Body[], manifolds: Manifold[], manifoldCount: number, joints: Joint[], dt: number): void {
    this.ensureCapacity(bodies.length);
    for (const body of bodies) {
      if (!body.isSimulated) continue;
      const still =
        body.velocity.lengthSq() < SLEEP_LINEAR_SPEED ** 2 && body.angularVelocity ** 2 < SLEEP_ANGULAR_SPEED ** 2;
      body.sleepTime = still ? body.sleepTime + dt : 0;
      this.parent[body.id] = body.id;
      this.islandStillTime[body.id] = Infinity;
    }

    for (let i = 0; i < manifoldCount; i++) {
      const { bodyA, bodyB } = manifolds[i];
      if (bodyA.isSimulated && bodyB.isSimulated) this.union(bodyA.id, bodyB.id);
    }
    for (const { bodyA, bodyB } of joints) {
      if (bodyA.isSimulated && bodyB.isSimulated) this.union(bodyA.id, bodyB.id);
    }

    for (const body of bodies) {
      if (!body.isSimulated) continue;
      const root = this.find(body.id);
      this.islandStillTime[root] = Math.min(this.islandStillTime[root], body.sleepTime);
    }

    for (const body of bodies) {
      if (!body.isSimulated) continue;
      if (this.islandStillTime[this.find(body.id)] < TIME_TO_SLEEP) continue;
      body.awake = false;
      body.velocity.set(0, 0);
      body.angularVelocity = 0;
    }
  }

  private ensureCapacity(size: number): void {
    if (this.parent.length >= size) return;
    this.parent = new Int32Array(size * 2);
    this.islandStillTime = new Float64Array(size * 2);
  }

  private find(id: number): number {
    const { parent } = this;
    while (parent[id] !== id) {
      parent[id] = parent[parent[id]];
      id = parent[id];
    }
    return id;
  }

  private union(idA: number, idB: number): void {
    const rootA = this.find(idA);
    const rootB = this.find(idB);
    if (rootA !== rootB) this.parent[rootA] = rootB;
  }
}
