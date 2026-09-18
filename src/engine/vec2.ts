/// Mutable 2D vector. Mutating methods return `this` and never allocate.
export class Vec2 {
  constructor(
    public x = 0,
    public y = 0,
  ) {}

  set(x: number, y: number): this {
    this.x = x;
    this.y = y;
    return this;
  }

  copy(other: Vec2): this {
    return this.set(other.x, other.y);
  }

  clone(): Vec2 {
    return new Vec2(this.x, this.y);
  }

  add(other: Vec2): this {
    return this.set(this.x + other.x, this.y + other.y);
  }

  sub(other: Vec2): this {
    return this.set(this.x - other.x, this.y - other.y);
  }

  scale(factor: number): this {
    return this.set(this.x * factor, this.y * factor);
  }

  // Fused this += other * factor; the integrator's inner operation.
  addScaled(other: Vec2, factor: number): this {
    return this.set(this.x + other.x * factor, this.y + other.y * factor);
  }

  dot(other: Vec2): number {
    return this.x * other.x + this.y * other.y;
  }

  // Z component of the 3D cross product.
  cross(other: Vec2): number {
    return this.x * other.y - this.y * other.x;
  }

  lengthSq(): number {
    return this.dot(this);
  }

  length(): number {
    return Math.sqrt(this.lengthSq());
  }
}
