import * as THREE from 'three'

/**
 * One Euro filter (Casiez et al. 2012) for 3D positions: heavy smoothing when the hand is still (kills
 * tracking jitter), little lag when it moves fast.
 */
export class OneEuroVector3 {
  constructor({ minCutoff = 1.5, beta = 4, dCutoff = 1 } = {}) {
    this.minCutoff = minCutoff
    this.beta = beta
    this.dCutoff = dCutoff
    this.value = new THREE.Vector3()
    this.deriv = new THREE.Vector3()
    this._raw = new THREE.Vector3()
    this.ready = false
  }

  static alpha(cutoff, dt) {
    const tau = 1 / (2 * Math.PI * cutoff)
    return 1 / (1 + tau / dt)
  }

  update(x, dt) {
    if (!this.ready || !(dt > 0)) {
      this.value.copy(x)
      this.deriv.set(0, 0, 0)
      this.ready = true
      return this.value
    }
    this._raw.copy(x).sub(this.value).divideScalar(dt)
    this.deriv.lerp(this._raw, OneEuroVector3.alpha(this.dCutoff, dt))
    const cutoff = this.minCutoff + this.beta * this.deriv.length()
    this.value.lerp(x, OneEuroVector3.alpha(cutoff, dt))
    return this.value
  }

  reset() { this.ready = false }
}
