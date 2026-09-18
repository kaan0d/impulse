import { Body } from '../engine/body';
import { FixedStepper } from '../engine/stepper';
import { World } from '../engine/world';
import { Renderer } from '../render/renderer';

const SPAWN_ANGULAR_VELOCITY = 1;

const spawners: Record<string, (x: number, y: number) => Body> = {
  circle: (x, y) => Body.circle(0.5, 1, x, y),
  box: (x, y) => Body.box(1, 1, 1, x, y),
};

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`#${id} missing from index.html`);
  return element as T;
}

const canvas = byId<HTMLCanvasElement>('view');
const world = new World();
world.add(Body.box(18, 1, Infinity, 9, 0.5));
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
