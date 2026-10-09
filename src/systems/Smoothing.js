// Time-constant based exponential smoothing, so the response does not depend on the headset's refresh rate
// (72, 90 or 120 Hz).

/** Fraction of the way to move toward the target this frame for a first-order filter with time constant tau. */
export function smoothingAlpha(tau, dt) {
  return tau > 0 && dt > 0 ? 1 - Math.exp(-dt / tau) : 1
}

export class QuaternionSmoother {
  constructor(tau = 0.02) {
    this.tau = tau
    this.value = null
  }

  update(target, dt) {
    if (this.value === null) {
      this.value = target.clone()
      return this.value
    }
    this.value.slerp(target, smoothingAlpha(this.tau, dt))
    return this.value
  }

  reset() { this.value = null }
}
