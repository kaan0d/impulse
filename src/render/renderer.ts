import type { Body, Shape } from '../engine/body';
import { Vec2 } from '../engine/vec2';
import type { World } from '../engine/world';

const PIXELS_PER_METER = 50;
const TWO_PI = Math.PI * 2;
// Palette of kaandinc.com: dark metal faces with amber edges, like its faceted solids.
const METAL = '#2a2620';
const EDGE_COLOR = '#e8963f';
const SLEEPING_FILL = '#1a1a18';
const SLEEPING_EDGE = '#6b4a26';
const STATIC_FILL = '#1f1e1b';
const STATIC_EDGE = '#3a3833';
const GRID_COLOR = 'rgba(235, 232, 226, 0.045)';
const FACET_ALPHA = 0.3;
const CONTACT_COLOR = '#ebe8e2';
const CONTACT_DOT_RADIUS = 0.06;
const NORMAL_LENGTH = 0.4;
const JOINT_COLOR = '#b9b5ac';
const JOINT_DOT_RADIUS = 0.07;

/// Draws a World on a canvas. World origin is the bottom-left corner, y up.
export class Renderer {
  private readonly ctx: CanvasRenderingContext2D;
  // Debug overlays, switchable at any time.
  showContacts = true;
  showJoints = true;
  private readonly anchorA = new Vec2();
  private readonly anchorB = new Vec2();

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly world: World,
  ) {
    const ctx = canvas.getContext('2d');
    if (!ctx) throw new Error('Canvas 2D context unavailable');
    this.ctx = ctx;
  }

  // Converts viewport coordinates (e.g. a mouse event) to world meters.
  clientToWorld(clientX: number, clientY: number): Vec2 {
    const rect = this.canvas.getBoundingClientRect();
    const pixelX = ((clientX - rect.left) * this.canvas.width) / rect.width;
    const pixelY = ((clientY - rect.top) * this.canvas.height) / rect.height;
    return new Vec2(pixelX / PIXELS_PER_METER, (this.canvas.height - pixelY) / PIXELS_PER_METER);
  }

  draw(): void {
    const { ctx, canvas } = this;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Flip y so world units are meters with y up.
    ctx.setTransform(PIXELS_PER_METER, 0, 0, -PIXELS_PER_METER, 0, canvas.height);
    ctx.lineWidth = 2 / PIXELS_PER_METER;
    this.drawGrid();
    // Markers such as a mouse-joint cursor are not drawn.
    for (const body of this.world.bodies) if (body.collidable) this.drawBody(body);
    if (this.showJoints) this.drawJoints();
    if (this.showContacts) this.drawContacts();
  }

  // One line per meter, so scale is readable.
  private drawGrid(): void {
    const { ctx, canvas } = this;
    ctx.strokeStyle = GRID_COLOR;
    ctx.beginPath();
    for (let x = 0; x <= canvas.width / PIXELS_PER_METER; x++) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height / PIXELS_PER_METER);
    }
    for (let y = 0; y <= canvas.height / PIXELS_PER_METER; y++) {
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width / PIXELS_PER_METER, y);
    }
    ctx.stroke();
  }

  // Debug overlay: a dot on each anchor, joined by a line (a rod shows as a line, a pin as one dot).
  private drawJoints(): void {
    const { ctx, world, anchorA, anchorB } = this;
    ctx.strokeStyle = JOINT_COLOR;
    ctx.fillStyle = JOINT_COLOR;
    for (const joint of world.joints) {
      joint.anchorA(anchorA);
      joint.anchorB(anchorB);
      ctx.beginPath();
      ctx.moveTo(anchorA.x, anchorA.y);
      ctx.lineTo(anchorB.x, anchorB.y);
      ctx.stroke();
      for (const anchor of [anchorA, anchorB]) {
        ctx.beginPath();
        ctx.arc(anchor.x, anchor.y, JOINT_DOT_RADIUS, 0, TWO_PI);
        ctx.fill();
      }
    }
  }

  // Debug overlay: a dot per contact point and a line along the contact normal.
  private drawContacts(): void {
    const { ctx, world } = this;
    ctx.strokeStyle = CONTACT_COLOR;
    ctx.fillStyle = CONTACT_COLOR;
    for (let i = 0; i < world.manifoldCount; i++) {
      const manifold = world.manifolds[i];
      for (let j = 0; j < manifold.count; j++) {
        const point = manifold.points[j];
        ctx.beginPath();
        ctx.arc(point.x, point.y, CONTACT_DOT_RADIUS, 0, TWO_PI);
        ctx.fill();
        ctx.beginPath();
        ctx.moveTo(point.x, point.y);
        ctx.lineTo(point.x + manifold.normal.x * NORMAL_LENGTH, point.y + manifold.normal.y * NORMAL_LENGTH);
        ctx.stroke();
      }
    }
  }

  private drawBody(body: Body): void {
    const { ctx } = this;
    const { shape } = body;
    const fixed = body.invMass === 0;
    ctx.fillStyle = fixed ? STATIC_FILL : body.awake ? METAL : SLEEPING_FILL;
    ctx.strokeStyle = fixed ? STATIC_EDGE : body.awake ? EDGE_COLOR : SLEEPING_EDGE;
    ctx.save();
    ctx.translate(body.position.x, body.position.y);
    ctx.rotate(body.angle);
    ctx.beginPath();
    if (shape.kind === 'circle') {
      ctx.arc(0, 0, shape.radius, 0, TWO_PI);
      // Radius line makes rotation visible.
      ctx.moveTo(0, 0);
      ctx.lineTo(shape.radius, 0);
    } else if (shape.kind === 'box') {
      ctx.rect(-shape.width / 2, -shape.height / 2, shape.width, shape.height);
    } else {
      shape.vertices.forEach((vertex, i) => (i === 0 ? ctx.moveTo(vertex.x, vertex.y) : ctx.lineTo(vertex.x, vertex.y)));
      ctx.closePath();
    }
    ctx.fill();
    ctx.stroke();
    if (!fixed) this.drawFacets(shape);
    ctx.restore();
  }

  // Faint lines from the center to each corner, echoing the faceted solids on kaandinc.com.
  // Circles get their radius line in drawBody.
  private drawFacets(shape: Shape): void {
    if (shape.kind === 'circle') return;
    const { ctx } = this;
    ctx.globalAlpha = FACET_ALPHA;
    ctx.beginPath();
    if (shape.kind === 'box') {
      // Diagonals are the spokes of a box.
      const hw = shape.width / 2;
      const hh = shape.height / 2;
      ctx.moveTo(-hw, -hh);
      ctx.lineTo(hw, hh);
      ctx.moveTo(hw, -hh);
      ctx.lineTo(-hw, hh);
    } else {
      for (const corner of shape.vertices) {
        ctx.moveTo(0, 0);
        ctx.lineTo(corner.x, corner.y);
      }
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
}
