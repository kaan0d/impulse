import { describe, expect, it } from 'vitest';
import { Body } from '../src/engine/body';
import { Manifold } from '../src/engine/manifold';

function manifold(): Manifold {
  return new Manifold(Body.circle(1, 1, 0, 0), Body.circle(1, 1, 1, 0));
}

// Simulates one step: remember the old contacts, refresh them with the given ids, then carry impulses over.
function refresh(m: Manifold, ids: number[]): void {
  m.savePrevious();
  m.count = 0;
  for (const id of ids) m.addContact(0, 0, 0.1, id);
  m.carryImpulses();
}

describe('Manifold warm start', () => {
  it('a contact keeps its impulses when its feature id survives', () => {
    const m = manifold();
    refresh(m, [10, 11]);
    m.normalImpulses.splice(0, 2, 1.5, 2.5);
    m.tangentImpulses.splice(0, 2, 0.1, -0.2);

    refresh(m, [10, 11]);
    expect(m.normalImpulses).toEqual([1.5, 2.5]);
    expect(m.tangentImpulses).toEqual([0.1, -0.2]);
  });

  it('impulses follow the feature id even when the contact order changes', () => {
    const m = manifold();
    refresh(m, [10, 11]);
    m.normalImpulses.splice(0, 2, 1.5, 2.5);

    refresh(m, [11, 10]);
    expect(m.normalImpulses).toEqual([2.5, 1.5]);
  });

  it('a new feature id starts from zero', () => {
    const m = manifold();
    refresh(m, [10, 11]);
    m.normalImpulses.splice(0, 2, 1.5, 2.5);

    refresh(m, [10, 12]);
    expect(m.normalImpulses).toEqual([1.5, 0]);
  });

  it('reset forgets the history', () => {
    const m = manifold();
    refresh(m, [10]);
    m.normalImpulses[0] = 3;
    m.reset();
    m.savePrevious();
    m.addContact(0, 0, 0.1, 10);
    m.carryImpulses();
    expect(m.normalImpulses[0]).toBe(0);
  });
});
