import { broadphaseSizes, runBroadphase, runScene, scenes, type BroadphaseResult, type SceneResult } from './scenes';

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`#${id} missing from bench/index.html`);
  return element as T;
}

function ms(value: number): string {
  return value.toFixed(2);
}

function addRow(table: HTMLTableElement, cells: string[]): void {
  const row = table.tBodies[0].insertRow();
  for (const text of cells) row.insertCell().textContent = text;
}

const runButton = byId<HTMLButtonElement>('run');
const status = byId('status');
const sceneTable = byId<HTMLTableElement>('scenes');
const broadphaseTable = byId<HTMLTableElement>('broadphase');
const json = byId('json');

byId('env').textContent = `${navigator.userAgent} | ${navigator.hardwareConcurrency} logical cores`;

// Yields to the event loop so the page repaints between the heavy runs.
function nextFrame(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

async function runAll(): Promise<void> {
  runButton.disabled = true;
  document.body.dataset.done = 'false';
  sceneTable.tBodies[0].replaceChildren();
  broadphaseTable.tBodies[0].replaceChildren();
  const sceneResults: SceneResult[] = [];
  const broadphaseResults: BroadphaseResult[] = [];

  for (const scene of scenes) {
    status.textContent = `Running ${scene.name}...`;
    await nextFrame();
    const result = runScene(scene);
    sceneResults.push(result);
    addRow(sceneTable, [
      result.name,
      String(result.bodies),
      String(result.steps),
      ms(result.avgMs),
      ms(result.p95Ms),
      ms(result.maxMs),
      String(result.peakContacts),
      String(result.awakeAtEnd),
    ]);
  }

  for (const size of broadphaseSizes) {
    status.textContent = `Running broadphase with ${size} bodies...`;
    await nextFrame();
    const result = runBroadphase(size);
    broadphaseResults.push(result);
    addRow(broadphaseTable, [
      String(result.bodies),
      String(result.pairs),
      ms(result.spatialHashMs),
      ms(result.bruteForceMs),
      `${(result.bruteForceMs / result.spatialHashMs).toFixed(1)}x`,
      result.samePairs ? 'yes' : 'NO',
    ]);
  }

  json.textContent = JSON.stringify({ scenes: sceneResults, broadphase: broadphaseResults }, null, 2);
  status.textContent = 'Done.';
  document.body.dataset.done = 'true';
  runButton.disabled = false;
}

runButton.addEventListener('click', () => void runAll());
void runAll();
