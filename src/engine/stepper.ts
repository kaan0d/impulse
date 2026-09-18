import type { World } from './world';

export const FIXED_DT = 1 / 60;

// Caps a long frame (tab switch, breakpoint) so catch-up cannot spiral.
const MAX_FRAME_SECONDS = 0.25;

/// Turns variable frame times into whole fixed steps. Physics only ever sees FIXED_DT.
export class FixedStepper {
  private accumulator = 0;

  constructor(private readonly world: World) {}

  // Returns how many physics steps ran for this frame.
  advance(frameSeconds: number): number {
    this.accumulator += Math.min(frameSeconds, MAX_FRAME_SECONDS);
    let steps = 0;
    while (this.accumulator >= FIXED_DT) {
      this.world.step(FIXED_DT);
      this.accumulator -= FIXED_DT;
      steps++;
    }
    return steps;
  }
}
