// Damped least-squares IK for one 7-DoF G1 arm, solved on MuJoCo's own kinematics.
// Pure JS (no three.js) so it runs inside the physics worker and in Node tests.

export const ARM_JOINTS = [
  'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
  'wrist_roll', 'wrist_pitch', 'wrist_yaw',
]

const N = 7

// MuJoCo quaternions are [w, x, y, z]
export function mat2quat(m, o, out) {
  const m00 = m[o], m01 = m[o + 1], m02 = m[o + 2]
  const m10 = m[o + 3], m11 = m[o + 4], m12 = m[o + 5]
  const m20 = m[o + 6], m21 = m[o + 7], m22 = m[o + 8]
  const tr = m00 + m11 + m22
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2
    out[0] = 0.25 * s; out[1] = (m21 - m12) / s; out[2] = (m02 - m20) / s; out[3] = (m10 - m01) / s
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2
    out[0] = (m21 - m12) / s; out[1] = 0.25 * s; out[2] = (m01 + m10) / s; out[3] = (m02 + m20) / s
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2
    out[0] = (m02 - m20) / s; out[1] = (m01 + m10) / s; out[2] = 0.25 * s; out[3] = (m12 + m21) / s
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2
    out[0] = (m10 - m01) / s; out[1] = (m02 + m20) / s; out[2] = (m12 + m21) / s; out[3] = 0.25 * s
  }
  return out
}

// Rotation error (axis * angle, world frame) taking orientation `cur` to `target`
function rotationError(target, cur, out, o) {
  // q = target * conj(cur)
  const [tw, tx, ty, tz] = target
  const cw = cur[0], cx = -cur[1], cy = -cur[2], cz = -cur[3]
  let w = tw * cw - tx * cx - ty * cy - tz * cz
  let x = tw * cx + tx * cw + ty * cz - tz * cy
  let y = tw * cy - tx * cz + ty * cw + tz * cx
  let z = tw * cz + tx * cy - ty * cx + tz * cw
  if (w < 0) { w = -w; x = -x; y = -y; z = -z }
  const s = Math.hypot(x, y, z)
  const k = s > 1e-9 ? 2 * Math.atan2(s, w) / s : 2
  out[o] = x * k; out[o + 1] = y * k; out[o + 2] = z * k
}

// Solves A x = b in place for a symmetric positive-definite 7x7 A (Cholesky)
function cholSolve(A, b) {
  for (let j = 0; j < N; j++) {
    let s = A[j * N + j]
    for (let k = 0; k < j; k++) s -= A[j * N + k] * A[j * N + k]
    const d = Math.sqrt(Math.max(s, 1e-12))
    A[j * N + j] = d
    for (let i = j + 1; i < N; i++) {
      let t = A[i * N + j]
      for (let k = 0; k < j; k++) t -= A[i * N + k] * A[j * N + k]
      A[i * N + j] = t / d
    }
  }
  for (let i = 0; i < N; i++) {
    let t = b[i]
    for (let k = 0; k < i; k++) t -= A[i * N + k] * b[k]
    b[i] = t / A[i * N + i]
  }
  for (let i = N - 1; i >= 0; i--) {
    let t = b[i]
    for (let k = i + 1; k < N; k++) t -= A[k * N + i] * b[k]
    b[i] = t / A[i * N + i]
  }
  return b
}

export class ArmIK {
  constructor(mj, m, side, posture) {
    const id = (type, name) => {
      const i = mj.mj_name2id(m, type.value, name)
      if (i < 0) throw new Error(`IK: missing ${name}`)
      return i
    }
    this.m = m
    this.site = id(mj.mjtObj.mjOBJ_SITE, `${side}_palm`)
    this.jnt = Int32Array.from(ARM_JOINTS, n => id(mj.mjtObj.mjOBJ_JOINT, `${side}_${n}_joint`))
    this.qadr = Int32Array.from(this.jnt, j => m.jnt_qposadr[j])
    this.lo = Float64Array.from(this.jnt, j => m.jnt_range[2 * j])
    this.hi = Float64Array.from(this.jnt, j => m.jnt_range[2 * j + 1])
    this.posture = Float64Array.from(posture)

    this.e = new Float64Array(6)
    this.J = new Float64Array(6 * N)
    this.A = new Float64Array(N * N)
    this.b = new Float64Array(N)
    this.qc = new Float64Array(4)
  }

  /**
   * Moves the arm joints in `d.qpos` toward the palm target. `d` is a scratch MjData that holds the commanded
   * configuration (not the physical state), so contact never drags the IK solution around.
   */
  solve(mj, d, targetPos, targetQuat, {
    iterations = 4, rotWeight = 0.35, damping = 1e-3, postureWeight = 3e-3, maxStep = 0.3,
  } = {}) {
    const { m, e, J, A, b, qc, qadr, jnt } = this
    for (let it = 0; it < iterations; it++) {
      mj.mj_kinematics(m, d)
      const sp = d.site_xpos, s3 = 3 * this.site
      const px = sp[s3], py = sp[s3 + 1], pz = sp[s3 + 2]
      e[0] = targetPos[0] - px; e[1] = targetPos[1] - py; e[2] = targetPos[2] - pz
      mat2quat(d.site_xmat, 9 * this.site, qc)
      rotationError(targetQuat, qc, e, 3)
      e[3] *= rotWeight; e[4] *= rotWeight; e[5] *= rotWeight

      const ax = d.xaxis, an = d.xanchor
      for (let k = 0; k < N; k++) {
        const j3 = 3 * jnt[k]
        const a0 = ax[j3], a1 = ax[j3 + 1], a2 = ax[j3 + 2]
        const r0 = px - an[j3], r1 = py - an[j3 + 1], r2 = pz - an[j3 + 2]
        J[k] = a1 * r2 - a2 * r1
        J[N + k] = a2 * r0 - a0 * r2
        J[2 * N + k] = a0 * r1 - a1 * r0
        J[3 * N + k] = a0 * rotWeight
        J[4 * N + k] = a1 * rotWeight
        J[5 * N + k] = a2 * rotWeight
      }

      // (J^T J + (damping + postureWeight) I) dq = J^T e + postureWeight (posture - q)
      const qpos = d.qpos
      for (let i = 0; i < N; i++) {
        let bi = 0
        for (let r = 0; r < 6; r++) bi += J[r * N + i] * e[r]
        b[i] = bi + postureWeight * (this.posture[i] - qpos[qadr[i]])
        for (let k = 0; k <= i; k++) {
          let s = 0
          for (let r = 0; r < 6; r++) s += J[r * N + i] * J[r * N + k]
          A[i * N + k] = s
          A[k * N + i] = s
        }
        A[i * N + i] += damping + postureWeight
      }
      cholSolve(A, b)

      for (let i = 0; i < N; i++) {
        const dq = Math.max(-maxStep, Math.min(maxStep, b[i]))
        qpos[qadr[i]] = Math.max(this.lo[i], Math.min(this.hi[i], qpos[qadr[i]] + dq))
      }
    }
  }
}
