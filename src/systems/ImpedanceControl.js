import * as THREE from 'three'

// ─── First-Order Exponential Smoother ────────────────────────────────────────
// Simple low-pass filter. alpha=0 = frozen, alpha=1 = instant snap.
// alpha ~0.08 gives good "heavy arm" feel matching G1 motor dynamics.

export class ExponentialSmoother {
  constructor(alpha = 0.08) {
    this.alpha = alpha
    this.value = null
  }

  update(target) {
    if (this.value === null) {
      this.value = target.clone()
      return this.value
    }
    this.value.lerp(target, this.alpha)
    return this.value
  }

  reset(value) {
    this.value = value ? value.clone() : null
  }
}

export class QuaternionSmoother {
  constructor(alpha = 0.12) {
    this.alpha = alpha
    this.value = null
  }

  update(target) {
    if (this.value === null) {
      this.value = target.clone()
      return this.value
    }
    this.value.slerp(target, this.alpha)
    return this.value
  }

  reset(value) {
    this.value = value ? value.clone() : null
  }
}

// ─── Second-Order Spring-Damper ───────────────────────────────────────────────
// Models the G1 arm as a mass-spring-damper system.
// M*x'' + C*x' + K*(x - x_target) = 0
//
// Tuned to approximate G1 joint torque limits (400W max output).
// omega_n = 8 rad/s (~1.27 Hz natural frequency), zeta = 0.9 (overdamped)
// Prevents instant velocity spikes that would saturate real motors.

export class SpringDamper {
  constructor(options = {}) {
    const {
      mass    = 1.0,
      stiffness = 64.0,    // K  — higher = snappier
      damping   = 14.4,    // C  — 2*sqrt(K*M)*zeta, zeta≈0.9
    } = options

    this.M = mass
    this.K = stiffness
    this.C = damping

    this.pos = null      // THREE.Vector3 current position
    this.vel = new THREE.Vector3()  // current velocity
  }

  update(target, dt) {
    if (!this.pos) {
      this.pos = target.clone()
      return this.pos
    }

    dt = Math.min(dt, 0.05)  // cap timestep to prevent blow-up

    // F = -K*(pos - target) - C*vel
    const force = new THREE.Vector3()
    force.copy(this.pos).sub(target).multiplyScalar(-this.K)
    force.addScaledVector(this.vel, -this.C)
    force.divideScalar(this.M)

    this.vel.addScaledVector(force, dt)
    this.pos.addScaledVector(this.vel, dt)

    return this.pos
  }

  reset(pos) {
    this.pos = pos ? pos.clone() : null
    this.vel.set(0, 0, 0)
  }
}

// ─── Per-Hand Impedance State ─────────────────────────────────────────────────
// One instance per hand. Manages smooth position + orientation tracking.

export class HandImpedance {
  constructor() {
    this.wristPos  = new SpringDamper({ stiffness: 64, damping: 14.4 })
    this.wristQuat = new QuaternionSmoother(0.10)
  }

  update(rawPos, rawQuat, dt) {
    const smoothPos  = this.wristPos.update(rawPos, dt)
    const smoothQuat = this.wristQuat.update(rawQuat)
    return { position: smoothPos, quaternion: smoothQuat }
  }

  reset() {
    this.wristPos.reset(null)
    this.wristQuat.reset(null)
  }
}
