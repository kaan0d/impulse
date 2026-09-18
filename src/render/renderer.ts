import type { Body } from '../engine/body';
import { Vec2 } from '../engine/vec2';
import type { World } from '../engine/world';

const PIXELS_PER_METER = 50;
const TWO_PI = Math.PI * 2;
const DYNAMIC_COLOR = '#4f8cff';
const SLEEPING_COLOR = '#2d4a80';
const STATIC_COLOR = '#5c6370';
const CONTACT_COLOR = '#ff5c5c';
const CONTACT_DOT_RADIUS = 0.06;
const NORMAL_LENGTH = 0.4;
const JOINT_COLOR = '#7ee0a1';
const JOINT_DOT_RADIUS = 0.07;

/// Draws a World on a canvas. World origin is the bottom-left corner, y up.
export class Renderer {
  private readonly ctx: CanvasRenderingContext2D;
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
    ctx.strokeStyle = '#dbe6ff';
    for (const body of this.world.bodies) this.drawBody(body);
    this.drawJoints();
    this.drawContacts();
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
    ctx.fillStyle = body.invMass === 0 ? STATIC_COLOR : body.awake ? DYNAMIC_COLOR : SLEEPING_COLOR;
    ctx.save();
    ctx.translate(body.position.x, body.position.y);
    ctx.rotate(body.angle);
    ctx.beginPath();
    if (shape.kind === 'circle') {
      ctx.arc(0, 0, shape.radius, 0, TWO_PI);
      // Radius line makes rotation visible.
      ctx.moveTo(0, 0);
      ctx.lineTo(shape.radius, 0);
    } else {
      ctx.rect(-shape.width / 2, -shape.height / 2, shape.width, shape.height);
    }
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}
