import type { Body } from './body';

// Pair key = smaller id * stride + larger id. 2^26 keeps the key an exact integer for up to 67M bodies.
export const PAIR_KEY_STRIDE = 2 ** 26;

const MIN_BUCKETS = 64;

/// Uniform-grid broadphase. Reports every pair of bodies with overlapping bounding boxes where at least
/// one body is simulated, as sorted unique pair keys. Rebuilt from scratch on each call, allocation-free
/// once its buffers have grown. A body much larger than `cellSize` covers many cells: slower, still correct.
export class SpatialHash {
  // minX, minY, maxX, maxY per body id.
  private bounds = new Float64Array(0);
  // Bucket b owns entries[bucketStart[b] .. bucketStart[b + 1]); the extra slot holds the total.
  private bucketStart = new Int32Array(1);
  private entries = new Int32Array(0);
  private keys = new Float64Array(0);
  // Slots below this may hold last call's keys; the rest are Infinity so a full-array sort keeps keys in front.
  private staleKeyCount = 0;
  private bucketMask = 0;
  // Cell range of the body most recently passed to `loadCells`.
  private cellX0 = 0;
  private cellY0 = 0;
  private cellX1 = 0;
  private cellY1 = 0;

  constructor(private readonly cellSize: number) {}

  // Valid up to the count returned by `findPairs`; read it after the call because the array can be replaced.
  get pairKeys(): Float64Array {
    return this.keys;
  }

  // Bodies must have ids equal to their index in `bodies`. Returns the number of pair keys.
  findPairs(bodies: Body[]): number {
    this.updateBounds(bodies);
    this.buildGrid(bodies.length);
    return this.sortUnique(this.collectPairs(bodies));
  }

  private updateBounds(bodies: Body[]): void {
    if (this.bounds.length < bodies.length * 4) this.bounds = new Float64Array(bodies.length * 8);
    const { bounds } = this;
    for (const body of bodies) {
      const { shape } = body;
      let halfX: number;
      let halfY: number;
      if (shape.kind === 'circle') {
        halfX = shape.radius;
        halfY = shape.radius;
      } else {
        const cos = Math.abs(Math.cos(body.angle));
        const sin = Math.abs(Math.sin(body.angle));
        halfX = (shape.width * cos + shape.height * sin) / 2;
        halfY = (shape.width * sin + shape.height * cos) / 2;
      }
      const at = body.id * 4;
      bounds[at] = body.position.x - halfX;
      bounds[at + 1] = body.position.y - halfY;
      bounds[at + 2] = body.position.x + halfX;
      bounds[at + 3] = body.position.y + halfY;
    }
  }

  // Counting sort of (cell, body) entries into hash buckets, keeping body ids ascending inside each bucket.
  private buildGrid(bodyCount: number): void {
    this.ensureBuckets(bodyCount);
    const { bucketStart, bucketMask } = this;
    const bucketCount = bucketMask + 1;
    bucketStart.fill(0);

    let total = 0;
    for (let id = 0; id < bodyCount; id++) {
      this.loadCells(id);
      for (let cy = this.cellY0; cy <= this.cellY1; cy++) {
        for (let cx = this.cellX0; cx <= this.cellX1; cx++) {
          bucketStart[bucketOf(cx, cy, bucketMask)]++;
          total++;
        }
      }
    }
    for (let b = 1; b < bucketCount; b++) bucketStart[b] += bucketStart[b - 1];
    bucketStart[bucketCount] = total;
    if (this.entries.length < total) this.entries = new Int32Array(total * 2);

    // Filling from the last body down leaves each bucket ascending and bucketStart[b] at its first slot.
    for (let id = bodyCount - 1; id >= 0; id--) {
      this.loadCells(id);
      for (let cy = this.cellY0; cy <= this.cellY1; cy++) {
        for (let cx = this.cellX0; cx <= this.cellX1; cx++) {
          this.entries[--bucketStart[bucketOf(cx, cy, bucketMask)]] = id;
        }
      }
    }
  }

  // Appends a key for every overlapping pair sharing a bucket; may repeat keys. Returns the raw count.
  private collectPairs(bodies: Body[]): number {
    const { bucketStart, entries, bucketMask } = this;
    let count = 0;
    for (let b = 0; b <= bucketMask; b++) {
      const end = bucketStart[b + 1];
      for (let p = bucketStart[b]; p < end - 1; p++) {
        const idA = entries[p];
        for (let q = p + 1; q < end; q++) {
          const idB = entries[q];
          if (idA === idB || !this.boundsOverlap(idA, idB)) continue;
          if (!bodies[idA].isSimulated && !bodies[idB].isSimulated) continue;
          if (count === this.keys.length) this.growKeys(count);
          this.keys[count++] = idA * PAIR_KEY_STRIDE + idB;
        }
      }
    }
    return count;
  }

  private sortUnique(rawCount: number): number {
    const { keys } = this;
    keys.fill(Infinity, rawCount, this.staleKeyCount);
    keys.sort();
    let unique = 0;
    for (let k = 0; k < rawCount; k++) {
      if (unique > 0 && keys[k] === keys[unique - 1]) continue;
      keys[unique++] = keys[k];
    }
    this.staleKeyCount = rawCount;
    return unique;
  }

  private loadCells(id: number): void {
    const { bounds, cellSize } = this;
    this.cellX0 = Math.floor(bounds[id * 4] / cellSize);
    this.cellY0 = Math.floor(bounds[id * 4 + 1] / cellSize);
    this.cellX1 = Math.floor(bounds[id * 4 + 2] / cellSize);
    this.cellY1 = Math.floor(bounds[id * 4 + 3] / cellSize);
  }

  private boundsOverlap(idA: number, idB: number): boolean {
    const { bounds } = this;
    const a = idA * 4;
    const b = idB * 4;
    return bounds[a] <= bounds[b + 2] && bounds[b] <= bounds[a + 2] && bounds[a + 1] <= bounds[b + 3] && bounds[b + 1] <= bounds[a + 3];
  }

  // Power-of-two bucket count, at least two per body, so hash collisions stay rare.
  private ensureBuckets(bodyCount: number): void {
    let buckets = MIN_BUCKETS;
    while (buckets < bodyCount * 2) buckets *= 2;
    if (buckets - 1 === this.bucketMask) return;
    this.bucketMask = buckets - 1;
    this.bucketStart = new Int32Array(buckets + 1);
  }

  private growKeys(count: number): void {
    const grown = new Float64Array(Math.max(256, count * 2)).fill(Infinity);
    grown.set(this.keys.subarray(0, count));
    this.keys = grown;
  }
}

function bucketOf(cellX: number, cellY: number, mask: number): number {
  return (Math.imul(cellX, 73856093) ^ Math.imul(cellY, 19349663)) & mask;
}
