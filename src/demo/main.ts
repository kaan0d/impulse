import { Body } from '../engine/body';
import { MouseJoint } from '../engine/joint';
import { FIXED_DT, FixedStepper } from '../engine/stepper';
import { World } from '../engine/world';
import { Renderer } from '../render/renderer';
import { scenes } from './scenes';

const SPAWN_ANGULAR_VELOCITY = 1;
// A held body can be pulled with this many newtons per kilogram.
const GRAB_FORCE_PER_KG = 1000;

const spawners: Record<string, (x: number, y: number) => Body> = {
  circle: (x, y) => Body.circle(0.5, 1, x, y),
  box: (x, y) => Body.box(1, 1, 1, x, y),
  triangle: (x, y) => Body.regularPolygon(3, 0.7, 1, x, y),
  diamond: (x, y) => Body.regularPolygon(4, 0.7, 1, x, y),
  pentagon: (x, y) => Body.regularPolygon(5, 0.6, 1, x, y),
  hexagon: (x, y) => Body.regularPolygon(6, 0.55, 1, x, y),
};

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`#${id} missing from index.html`);
  return element as T;
}

const canvas = byId<HTMLCanvasElement>('view');
const world = new World();
const renderer = new Renderer(canvas, world);
const stepper = new FixedStepper(world);

// Read-only handle for the browser test and for poking around in the console.
declare global {
  interface Window {
    impulse: { world: World; renderer: Renderer };
  }
}
window.impulse = { world, renderer };

// --- scene loading ---

const sceneSelect = byId<HTMLSelectElement>('scene');
scenes.forEach((scene, index) => sceneSelect.add(new Option(scene.name, String(index))));

// The mouse-joint marker body and the joint in use; both die with the scene.
let cursor: Body | null = null;
let grab: MouseJoint | null = null;

function loadScene(): void {
  world.clear();
  cursor = null;
  grab = null;
  scenes[Number(sceneSelect.value)].build(world);
}

sceneSelect.addEventListener('change', loadScene);
byId('reset').addEventListener('click', loadScene);
loadScene();

// --- time controls ---

const pauseButton = byId<HTMLButtonElement>('pause');
const speedSelect = byId<HTMLSelectElement>('speed');
let paused = false;

function setPaused(value: boolean): void {
  paused = value;
  pauseButton.textContent = paused ? 'Resume' : 'Pause';
}

pauseButton.addEventListener('click', () => setPaused(!paused));
byId('step').addEventListener('click', () => {
  setPaused(true);
  world.step(FIXED_DT);
});

// --- overlays and options ---

for (const [id, apply] of [
  ['show-contacts', (on: boolean) => (renderer.showContacts = on)],
  ['show-joints', (on: boolean) => (renderer.showJoints = on)],
  ['continuous', (on: boolean) => (world.continuousCollision = on)],
] as const) {
  const box = byId<HTMLInputElement>(id);
  apply(box.checked);
  box.addEventListener('change', () => apply(box.checked));
}

// --- drag and drop spawning ---

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

// --- grabbing bodies with the pointer ---

canvas.addEventListener('pointerdown', (event) => {
  const at = renderer.clientToWorld(event.clientX, event.clientY);
  const body = world.bodyAt(at.x, at.y);
  if (!body || grab) return;
  if (!cursor) {
    cursor = world.add(Body.circle(0.01, Infinity, at.x, at.y));
    cursor.collidable = false;
  }
  cursor.position.set(at.x, at.y);
  grab = world.addJoint(new MouseJoint(cursor, body, at, GRAB_FORCE_PER_KG * body.mass)) as MouseJoint;
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener('pointermove', (event) => {
  if (!grab) return;
  const at = renderer.clientToWorld(event.clientX, event.clientY);
  grab.setTarget(at.x, at.y);
});

function release(): void {
  if (!grab) return;
  world.removeJoint(grab);
  grab = null;
}
canvas.addEventListener('pointerup', release);
canvas.addEventListener('pointercancel', release);

// --- main loop ---

const status = byId('status');
let lastFrameMs = performance.now();
let lastStatusMs = 0;
let stepMs = 0;

function frame(nowMs: number): void {
  const frameSeconds = Math.max(0, nowMs - lastFrameMs) / 1000;
  lastFrameMs = nowMs;
  if (!paused) {
    const start = performance.now();
    // The speed multiplier only changes how much time is fed in; physics still steps by FIXED_DT.
    stepper.advance(frameSeconds * Number(speedSelect.value));
    stepMs = performance.now() - start;
  }
  renderer.draw();

  if (nowMs - lastStatusMs > 250) {
    lastStatusMs = nowMs;
    const awake = world.bodies.filter((body) => body.isSimulated).length;
    status.textContent = `${world.bodies.length} bodies, ${awake} awake, ${world.manifoldCount} contacts, ${stepMs.toFixed(1)} ms per frame of physics`;
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
