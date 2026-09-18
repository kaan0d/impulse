import type { Body } from './body';
import { Vec2 } from './vec2';

/// Contact patch between two bodies. Persistent per touching pair; `collide` overwrites it in place.
export class Manifold {
  // Unit vector from bodyA toward bodyB.
  readonly normal = new Vec2();
  // Only the first `count` entries of each per-contact array are valid.
  readonly points = [new Vec2(), new Vec2()];
  readonly depths = [0, 0];
  // Which geometric feature made the contact, so impulses can follow it between steps.
  readonly ids = [0, 0];
  // Accumulated solver impulses, reused as the next step's warm start.
  readonly normalImpulses = [0, 0];
  readonly tangentImpulses = [0, 0];
  count = 0;

  private prevCount = 0;
  private readonly prevIds = [0, 0];
  private readonly prevNormalImpulses = [0, 0];
  private readonly prevTangentImpulses = [0, 0];

  constructor(
    public bodyA: Body,
    public bodyB: Body,
  ) {}

  addContact(x: number, y: number, depth: number, id: number): void {
    this.points[this.count].set(x, y);
    this.depths[this.count] = depth;
    this.ids[this.count] = id;
    this.normalImpulses[this.count] = 0;
    this.tangentImpulses[this.count] = 0;
    this.count++;
  }

  // Call before `collide` refreshes the contacts.
  savePrevious(): void {
    this.prevCount = this.count;
    for (let i = 0; i < this.count; i++) {
      this.prevIds[i] = this.ids[i];
      this.prevNormalImpulses[i] = this.normalImpulses[i];
      this.prevTangentImpulses[i] = this.tangentImpulses[i];
    }
  }

  // Call after `collide`: a contact with the same feature id inherits the saved impulses.
  carryImpulses(): void {
    for (let i = 0; i < this.count; i++) {
      for (let j = 0; j < this.prevCount; j++) {
        if (this.ids[i] !== this.prevIds[j]) continue;
        this.normalImpulses[i] = this.prevNormalImpulses[j];
        this.tangentImpulses[i] = this.prevTangentImpulses[j];
        break;
      }
    }
  }

  // Forgets all contact history so the manifold can serve a new pair.
  reset(): void {
    this.count = 0;
    this.prevCount = 0;
  }
}
