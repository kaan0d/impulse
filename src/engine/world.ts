import type { Body } from './body';
import { collide } from './collide';
import { Manifold } from './manifold';
import { ContactSolver } from './solver';
import { Vec2 } from './vec2';

export class World {
  readonly gravity = new Vec2(0, -9.81);
  readonly bodies: Body[] = [];
  // Reused pool; only the first `manifoldCount` entries are valid after a step.
  readonly manifolds: Manifold[] = [];
  manifoldCount = 0;
  private readonly solver = new ContactSolver();

  add(body: Body): Body {
    this.bodies.push(body);
    return body;
  }

  // Semi-implicit Euler: contacts, gravity into velocity, contact solve, then position from the new velocity.
  step(dt: number): void {
    this.detectCollisions();
    for (const body of this.bodies) {
      if (body.invMass === 0) continue;
      body.velocity.addScaled(this.gravity, dt);
    }
    this.solver.solve(this.manifolds, this.manifoldCount);
    for (const body of this.bodies) {
      if (body.invMass === 0) continue;
      body.position.addScaled(body.velocity, dt);
      body.angle += body.angularVelocity * dt;
    }
  }

  // Brute-force O(n^2) pairs; the stage 5 broadphase replaces this loop.
  detectCollisions(): void {
    const { bodies, manifolds } = this;
    let count = 0;
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const a = bodies[i];
        const b = bodies[j];
        if (a.invMass === 0 && b.invMass === 0) continue;
        let manifold: Manifold | undefined = manifolds[count];
        if (!manifold) {
          manifold = new Manifold(a, b);
          manifolds.push(manifold);
        }
        if (collide(a, b, manifold)) count++;
      }
    }
    this.manifoldCount = count;
  }
}
