# Impulse

A 2D rigid body physics engine written from scratch in TypeScript. No physics libraries. Design reference: Box2D Lite (Erin Catto).

Live demo: https://kaan0d.github.io/impulse/ (goes live in Stage 7)

## Rules

- Fixed timestep (1/60 s) with an accumulator. Render dt never reaches physics.
- Deterministic: no `Math.random`, same input gives bit-identical output.
- No allocations in hot paths (step, narrowphase, solver).
- The engine (`src/engine/`) never touches DOM or Canvas.

## Commands

| Command | What it does |
| --- | --- |
| `npm run dev` | Start the demo |
| `npm test` | Run unit tests (Vitest) |
| `npm run typecheck` | `tsc --noEmit`, strict mode |
| `npm run build` | Production build under the `/impulse/` base path |

## Structure

```
src/engine/   pure physics (Vec2, Body, World, FixedStepper, collide, Manifold, ContactSolver, Sleeper, SpatialHash)
src/render/   Canvas drawing and contact debug overlay
src/demo/     demo scenes
tests/        Vitest unit tests
```

## Roadmap

One commit per stage: `stage N: <summary>`. A stage is done when its tests are green and its demo works.

| Stage | Adds | Tests |
| --- | --- | --- |
| 1. Bodies and integration | Vec2, circle and box bodies, semi-implicit Euler, gravity, fixed-timestep loop, Canvas renderer, drag-and-drop spawning | Free fall matches the analytic result; fixed timestep is deterministic |
| 2. Collision detection | SAT for box/box, circle/circle, circle/box; contact manifold | Known overlaps give the expected normal, depth and contact points; no contact when apart |
| 3. Impulse solver | Sequential impulses, friction, restitution, accumulated impulse clamping | Box falls and comes to rest on the ground (velocity ~0, penetration < 0.01); elastic collisions conserve energy |
| 4. Stacking stability | Warm starting, sleeping | 10-box tower stands for 10 s without toppling; same scene twice is bit-identical |
| 5. Broadphase | Spatial hash | Same pair set as brute force |
| 6. Joints (optional) | Distance / revolute constraints | Joint length and anchor stay within tolerance under load |
| 7. Benchmark and deploy | Benchmark page (`bench/`), results table below, GitHub Actions deploy to Pages | Build works under the Pages base path |

## Status

- [x] Stage 1
- [x] Stage 2
- [x] Stage 3
- [x] Stage 4
- [x] Stage 5
- [ ] Stage 6 (optional)
- [ ] Stage 7

## Benchmark results

Added in Stage 7.
