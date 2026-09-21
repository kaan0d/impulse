import type { Body } from './body';

/// Writes the world-space bounding box of `body` as minX, minY, maxX, maxY at `out[at..at + 3]`. No allocation.
export function writeBounds(body: Body, out: Float64Array | number[], at: number): void {
  const { shape, position } = body;
  if (shape.kind === 'circle') {
    out[at] = position.x - shape.radius;
    out[at + 1] = position.y - shape.radius;
    out[at + 2] = position.x + shape.radius;
    out[at + 3] = position.y + shape.radius;
    return;
  }
  const cos = Math.cos(body.angle);
  const sin = Math.sin(body.angle);
  if (shape.kind === 'box') {
    const halfX = (shape.width * Math.abs(cos) + shape.height * Math.abs(sin)) / 2;
    const halfY = (shape.width * Math.abs(sin) + shape.height * Math.abs(cos)) / 2;
    out[at] = position.x - halfX;
    out[at + 1] = position.y - halfY;
    out[at + 2] = position.x + halfX;
    out[at + 3] = position.y + halfY;
    return;
  }
  // Polygon: extremes of the rotated vertices.
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const vertex of shape.vertices) {
    const x = cos * vertex.x - sin * vertex.y;
    const y = sin * vertex.x + cos * vertex.y;
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  out[at] = position.x + minX;
  out[at + 1] = position.y + minY;
  out[at + 2] = position.x + maxX;
  out[at + 3] = position.y + maxY;
}
