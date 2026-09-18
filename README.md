# Impulse

A 2D rigid body physics engine written from scratch in TypeScript. No physics libraries. Design reference: Box2D Lite (Erin Catto).

Live demo: https://kaan0d.github.io/impulse/ (live once the deploy workflow has run, see Deployment)

Benchmark: https://kaan0d.github.io/impulse/bench/

## What it does

- **Bodies:** circles, boxes and convex polygons (up to 16 vertices), dynamic or static (`mass = Infinity`).
- **Collision:** SAT with reference-face clipping for boxes and polygons (up to two contacts), closest-feature tests for circles, and a spatial-hash broadphase.
- **Contacts:** sequential impulses with friction, restitution, accumulated impulse clamping, warm starting, and a 2-point block solver for stable stacks.
- **Sleeping:** touching bodies form islands that fall asleep together and wake on contact or when a joint partner wakes.
- **Continuous collision:** fast bodies cannot tunnel through static geometry.
- **Joints:** pin (with angle limits and a motor), rod, spring, rope, and a mouse joint for grabbing.
- **Demo:** several scenes, drag-and-drop spawning, grabbing with the mouse, pause, step, reset, speed control, and debug overlays.

## Rules

- Fixed timestep (1/60 s) with an accumulator. Render dt never reaches physics.
- Deterministic: no `Math.random`, same input gives bit-identical output.
- No allocations in hot paths (step, narrowphase, solver). This is a design goal that no test enforces.
- The engine (`src/engine/`) never touches DOM or Canvas.

## Commands

| Command | What it does |
| --- | --- |
| `npm run dev` | Start the demo |
| `npm test` | Run unit tests (Vitest) |
| `npm run e2e` | Run browser tests (Playwright, installed Google Chrome). Builds and serves the site itself |
| `npm run typecheck` | `tsc --noEmit`, strict mode |
| `npm run build` | Production build under the `/impulse/` base path |
| `npm run bench` | Open the benchmark page |

## Structure

```
src/engine/   pure physics: Vec2, Body, World, FixedStepper, collide (+ collide-convex, clip),
              Manifold, Solver, Sleeper, SpatialHash, Joint, continuous collision (ccd)
src/render/   Canvas drawing and debug overlays
src/demo/     demo scenes and the page controller
bench/        benchmark page (separate Vite entry)
tests/        Vitest unit tests
e2e/          Playwright browser tests
.githooks/    pre-commit hook
.github/      CI and Pages deploy workflows
```

## What is tested, and what is not

Unit tests (Vitest) and browser tests (Playwright) run against the engine and the built site.

| Claim | Evidence |
| --- | --- |
| Integration is correct | Free-fall velocity matches the analytic result, position matches the exact semi-implicit Euler sum, and stays within the first-order error of the continuous formula |
| Physics never sees frame time | Different frame rates give the same state as stepping by hand |
| Deterministic | Reruns are bit-identical for towers, polygon piles, joint chains and all joint features together, sleep flags included |
| Collision is right | Known overlaps give the expected normal, depth and points for every shape pair. Polygons agree with the box-box path on 4000 random pairs, and circle-polygon depth matches an independent distance computation |
| Solver behaves physically | Bodies rest with penetration under 0.01, elastic collisions conserve energy and momentum to 1e-9, friction decelerates at μg, resting impulses equal the weight, impulses never pull |
| Stacks stand and sleep | 10-, 15- and 20-box towers stand, and they fall asleep within seconds (a 10-box tower in under 3 s) |
| Broadphase is exact | The spatial hash finds the same pairs as brute force across five cell sizes, with polygons included |
| Joints hold | A pin pendulum's period is within 2% of the physics formula, limits and motor torque caps hold, a spring's deflection and frequency match `g/ω²` and `f`, ropes go slack and taut |
| No tunneling | Fast circles, boxes and polygons stop at thin static walls, and pass through them when continuous collision is off |
| Demo works in a browser | Drag-and-drop spawning, mouse grabbing, pause, step, reset, scene switching, overlays and the benchmark page run in Chrome with no console errors |
| Deploy base path | Every URL in the built pages is relative or under `/impulse/` |

Several tests were also checked by breaking the code on purpose and confirming they fail.

Not covered:

- Determinism was only checked on one machine's Node and Chrome. Other browsers or CPUs may differ in `Math.sin` and `Math.cos`.
- A pile of 500 mixed bodies never falls asleep in the 8 s measured, so sleeping helps stacks and pyramids, not big chaotic piles.
- Continuous collision only stops bodies at static geometry. Two fast dynamic bodies can still pass through each other.
- A ball bouncing under gravity comes back about 3.5% too high at 1/60 s, because gravity is added before the impact solve.
- The deploy workflow has not run yet, and the benchmark numbers are from one machine.

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
| 8. Convex polygons | Polygon bodies, general SAT with clipping, circle-polygon, polygon spawning in the demo | Mass properties match closed forms; polygons agree with the box path and an independent distance oracle; polygons rest flat |
| 9. Stability | 2-point block solver, sequential position correction, continuous collision against statics | 15- and 20-box towers stand and sleep; bullets stop at thin walls |
| 10. Joint upgrades | Angle limits, motors, springs, ropes, mouse joint, joint removal | Limits and torque caps hold, spring frequency and deflection match theory, rope slack behaviour |
| 11. Interactive demo | Scene picker, mouse grabbing, pause, step, reset, speed, overlay toggles, more scenes | Point queries and world reset; every scene simulates cleanly |
| 12. Automation | Pre-commit hook, CI on pull requests, Playwright browser tests | 14 browser tests against the built site |

## Status

- [x] Stage 1
- [x] Stage 2
- [x] Stage 3
- [x] Stage 4
- [x] Stage 5
- [x] Stage 6 (optional)
- [x] Stage 7
- [x] Stage 8
- [x] Stage 9
- [x] Stage 10
- [x] Stage 11
- [x] Stage 12

## Benchmark results

`npm run bench` opens the benchmark page (`bench/index.html`, also built and served at `/impulse/bench/`). It runs deterministic scenes through the full `World.step` (contacts, 10 solver iterations, integration, sleeping) and times the broadphase against a brute-force O(n²) test.

Measured on the production build (`vite build` then `vite preview`) in headless Chrome 153 driven by Playwright, Windows 11, AMD Ryzen 7 5800X. Timer resolution is 0.1 ms, and two runs differed by under 10%. Numbers from your machine will differ.

### Step time

| Scene | Bodies | Steps | Avg ms | p95 ms | Max ms | Peak contacts | Awake at end |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Pyramid, 210 boxes | 210 | 240 | 0.86 | 1.10 | 1.50 | 403 | 210 |
| Pyramid, 210 boxes, sleeping on | 210 | 240 | 0.29 | 0.90 | 1.20 | 403 | 0 |
| Pile, 500 bodies | 500 | 480 | 1.47 | 1.80 | 2.50 | 986 | 500 |
| Pile, 500 bodies, sleeping on | 500 | 480 | 1.39 | 1.70 | 2.00 | 986 | 500 |
| Pile, 1000 bodies | 1000 | 300 | 2.59 | 3.90 | 4.70 | 2116 | 1000 |

- Scenes without "sleeping on" keep every body awake, so they measure the solver under load. The 1/60 s budget is 16.7 ms.
- The pyramid falls fully asleep and its cost drops by about 3x. The 500-body pile never falls asleep in the 8 s measured, so sleeping does not help there.
- The first 20 steps of each scene are untimed warm-up.
- An earlier version of this table (30 solver iterations, headed Chrome, before the block solver) showed 3.3 ms for the pyramid and 10.4 ms for 1000 bodies. Both the iteration count and the browser mode changed, so the difference is not attributable to one cause.

### Broadphase

| Bodies | Overlapping pairs | Spatial hash ms | Brute force ms | Speedup | Same pairs |
| --- | --- | --- | --- | --- | --- |
| 250 | 123 | 0.03 | 0.12 | 4.4x | yes |
| 500 | 251 | 0.05 | 0.38 | 7.7x | yes |
| 1000 | 488 | 0.10 | 1.48 | 14.8x | yes |
| 2000 | 980 | 0.27 | 6.05 | 22.4x | yes |
| 4000 | 2064 | 0.60 | 25.70 | 42.5x | yes |

Spatial hash time grows about linearly with body count, brute force about quadratically. The comparison uses axis-aligned unit boxes at constant density, and the brute-force baseline is a bare bounding-box loop, not the full narrowphase.

## Automation

- **Pre-commit hook** (`.githooks/pre-commit`): runs the typecheck and unit tests before every commit. `npm install` points git at the folder through the `prepare` script. Skip it once with `git commit --no-verify`. The work is done by `pre-commit.mjs` in plain Node, so it needs no bash: on Windows, npm's own launcher is a bash script, and committing from an IDE or Git GUI without bash on its PATH fails with `/usr/bin/env: 'bash': No such file or directory`. On Linux or macOS the `pre-commit` file must be executable (`git update-index --chmod=+x .githooks/pre-commit` if it is not).
- **CI** (`.github/workflows/ci.yml`): on every pull request, runs the typecheck, unit tests, build and the browser tests, and uploads Playwright traces if something fails.
- **Deploy** (`.github/workflows/deploy.yml`): on every push to `main`, runs install, tests, typecheck and build, then publishes `dist/`.

## Deployment

The site is published by `.github/workflows/deploy.yml` with the official Pages actions. To enable it once, set **Settings → Pages → Source** to **GitHub Actions** in the repository. The build uses the base path `/impulse/`, so the demo is at `https://kaan0d.github.io/impulse/` and the benchmark at `https://kaan0d.github.io/impulse/bench/`.
