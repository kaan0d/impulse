import { Body } from '../engine/body';
import { DistanceJoint, RevoluteJoint } from '../engine/joint';
import { FixedStepper } from '../engine/stepper';
import { Vec2 } from '../engine/vec2';
import { World } from '../engine/world';
import { Renderer } from '../render/renderer';

const SPAWN_ANGULAR_VELOCITY = 1;

const spawners: Record<string, (x: number, y: number) => Body> = {
  circle: (x, y) => Body.circle(0.5, 1, x, y),
  box: (x, y) => Body.box(1, 1, 1, x, y),
};

// Six boxes pinned end to end from a fixed bar, flicked sideways so the chain swings.
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

// A ball on a fixed-length rod, released level with its pivot.
function addRodPendulum(world: World, pivotX: number, pivotY: number): void {
  const rodLength = 3;
  const pivot = world.add(Body.box(0.4, 0.4, Infinity, pivotX, pivotY));
  const bob = world.add(Body.circle(0.4, 1, pivotX + rodLength, pivotY));
  world.addJoint(new DistanceJoint(pivot, bob, new Vec2(pivotX, pivotY), new Vec2(pivotX + rodLength, pivotY)));
}

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`#${id} missing from index.html`);
  return element as T;
}

const canvas = byId<HTMLCanvasElement>('view');
const world = new World();
world.add(Body.box(18, 1, Infinity, 9, 0.5));
addSwingingChain(world, 4, 10.4);
addRodPendulum(world, 13, 11);
const renderer = new Renderer(canvas, world);
const stepper = new FixedStepper(world);

for (const item of byId('palette').querySelectorAll<HTMLElement>('.item')) {
  item.addEventListener('dragstart', (event) => {
    event.dataTransfer?.setData('text/plain', item.dataset.shape ?? '');
  });
}

canvas.addEventListener('dragover', (event) => event.preventDefault());
canvas.addEventListener('drop', (event) => {
  event.preventDefault();
  const spawn = spawners[event.dataTransfer?.getData('text/plain') ?? ''];
  if (!spawn) return;
  const at = renderer.clientToWorld(event.clientX, event.clientY);
  const body = spawn(at.x, at.y);
  body.angularVelocity = SPAWN_ANGULAR_VELOCITY;
  world.add(body);
});

let lastFrameMs = performance.now();
function frame(nowMs: number): void {
  stepper.advance(Math.max(0, nowMs - lastFrameMs) / 1000);
  lastFrameMs = nowMs;
  renderer.draw();
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
