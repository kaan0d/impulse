import { Body } from '../engine/body';
import { DistanceJoint, RevoluteJoint } from '../engine/joint';
import { Vec2 } from '../engine/vec2';
import type { World } from '../engine/world';

export interface Scene {
  name: string;
  build: (world: World) => void;
}

// The canvas shows 18 x 12 m. The ground's top surface is at y = 1.
const GROUND_TOP = 1;

function addGround(world: World): void {
  world.add(Body.box(18, 1, Infinity, 9, GROUND_TOP - 0.5));
}

function addWalls(world: World): void {
  world.add(Body.box(1, 12, Infinity, 0.5, 6));
  world.add(Body.box(1, 12, Infinity, 17.5, 6));
}

// Boxes pinned end to end from a fixed bar; the last one is flicked sideways so the chain swings.
function addSwingingChain(world: World, x: number, top: number): void {
  const linkLength = 0.8;
  let previous = world.add(Body.box(1.2, 0.2, Infinity, x, top + 0.1));
  for (let i = 0; i < 6; i++) {
    const pinY = top - i * linkLength;
    const link = world.add(Body.box(0.25, linkLength, 1, x, pinY - linkLength / 2));
    world.addJoint(new RevoluteJoint(previous, link, new Vec2(x, pinY)));
    previous = link;
  }
  previous.velocity.set(4, 0);
}

// A ball on a rigid rod, released level with its pivot.
function addRodPendulum(world: World, pivotX: number, pivotY: number): void {
  const rodLength = 3;
  const pivot = world.add(Body.box(0.4, 0.4, Infinity, pivotX, pivotY));
  const bob = world.add(Body.circle(0.4, 1, pivotX + rodLength, pivotY));
  world.addJoint(new DistanceJoint(pivot, bob, new Vec2(pivotX, pivotY), new Vec2(pivotX + rodLength, pivotY)));
}

function playground(world: World): void {
  addGround(world);
  addSwingingChain(world, 4, 10.4);
  addRodPendulum(world, 13, 11);
}

function pyramid(world: World): void {
  addGround(world);
  const rows = 10;
  const size = 0.8;
  for (let row = 0; row < rows; row++) {
    for (let i = 0; i < rows - row; i++) {
      const x = 9 + (i - (rows - row - 1) / 2) * (size + 0.01);
      world.add(Body.box(size, size, 1, x, GROUND_TOP + size / 2 + row * size));
    }
  }
}

// A tall tower and a wrecking ball on a rope that swings in from the right.
function towerAndWreckingBall(world: World): void {
  addGround(world);
  const size = 0.7;
  for (let i = 0; i < 10; i++) world.add(Body.box(size, size, 1, 4.8, GROUND_TOP + size / 2 + i * size));

  const anchor = world.add(Body.box(0.4, 0.4, Infinity, 11, 10));
  const ball = world.add(Body.circle(0.6, 8, 17, 10));
  const rope = new DistanceJoint(anchor, ball, new Vec2(11, 10), new Vec2(17, 10));
  rope.setRope();
  world.addJoint(rope);
}

// A mixed handful of shapes dropped into a walled container.
function polygonPile(world: World): void {
  addGround(world);
  addWalls(world);
  for (let i = 0; i < 48; i++) {
    const x = 2 + (i % 8) * 1.7;
    const y = 3 + Math.floor(i / 8) * 1.4;
    const body = makePileBody(i, x, y);
    body.angle = i * 0.61;
    world.add(body);
  }
}

function makePileBody(i: number, x: number, y: number): Body {
  switch (i % 4) {
    case 0:
      return Body.circle(0.4, 1, x, y);
    case 1:
      return Body.box(0.7, 0.7, 1, x, y);
    case 2:
      return Body.regularPolygon(3, 0.6, 1, x, y);
    default:
      return Body.regularPolygon(6, 0.5, 1, x, y);
  }
}

// A motor-driven paddle, a hinge with angle limits, and a spring. Poke them with the mouse.
function jointShowcase(world: World): void {
  addGround(world);

  const axle = world.add(Body.box(0.3, 0.3, Infinity, 3.5, 6));
  const paddle = world.add(Body.box(3.4, 0.25, 2, 3.5, 6));
  const windmill = new RevoluteJoint(axle, paddle, new Vec2(3.5, 6));
  windmill.setMotor(1.5, 300);
  world.addJoint(windmill);

  const pivot = world.add(Body.box(0.3, 0.3, Infinity, 9, 10));
  const arm = world.add(Body.box(2.4, 0.25, 1, 10.2, 10));
  const hinge = new RevoluteJoint(pivot, arm, new Vec2(9, 10));
  hinge.setLimits(-0.9, 0.6);
  world.addJoint(hinge);

  const hook = world.add(Body.box(0.3, 0.3, Infinity, 14, 11));
  const bob = world.add(Body.circle(0.4, 1, 14, 8));
  const spring = new DistanceJoint(hook, bob, new Vec2(14, 11), new Vec2(14, 8), 2.2);
  spring.setSpring(1.2, 0.1);
  world.addJoint(spring);
}

// Fast bullets aimed at a thin wall. Untick "Continuous collision" and reset to watch them tunnel through.
function bullets(world: World): void {
  addGround(world);
  world.add(Body.box(0.05, 6, Infinity, 13, GROUND_TOP + 3));
  for (let i = 0; i < 4; i++) {
    const bullet = i % 2 === 0 ? Body.circle(0.15, 1, 2, 2 + i) : Body.box(0.3, 0.3, 1, 2, 2 + i);
    bullet.velocity.set(200, 0);
    world.add(bullet);
  }
}

function empty(world: World): void {
  addGround(world);
}

export const scenes: Scene[] = [
  { name: 'Playground', build: playground },
  { name: 'Pyramid', build: pyramid },
  { name: 'Tower and wrecking ball', build: towerAndWreckingBall },
  { name: 'Polygon pile', build: polygonPile },
  { name: 'Joints', build: jointShowcase },
  { name: 'Bullets (continuous collision)', build: bullets },
  { name: 'Empty', build: empty },
];
