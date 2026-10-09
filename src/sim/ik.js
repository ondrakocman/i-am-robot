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
    this.w = { Jp: new Float64Array(3 * N), Jr: new Float64Array(3 * N), Jpi: new Float64Array(3 * N), N1: new Float64Array(N * N), JrN: new Float64Array(3 * N), JrNi: new Float64Array(3 * N), Jpn: new Float64Array(3 * N), N1n: new Float64Array(N * N), JrNm: new Float64Array(3 * N), JrNmi: new Float64Array(3 * N), p1: new Float64Array(N), dq: new Float64Array(N), p: new Float64Array(N), tmp: new Float64Array(6), M: new Float64Array(9), locked: new Uint8Array(N), scale: new Float64Array(N) }
    this.qc = new Float64Array(4)
  }

  /**
   * Moves the arm joints in `d.qpos` toward the palm target. `d` is a scratch MjData that holds the commanded
   * configuration (not the physical state), so contact never drags the IK solution around.
   *
   * Each iteration: a damped least-squares step on the palm position, the palm orientation solved in the
   * null space of the position (an orientation the wrist cannot reach then costs orientation error only, it
   * never drags the palm off the operator's hand), and a step toward the rest posture in the null space of
   * both, which chooses where the elbow goes without moving the palm. Joints at a limit are locked out of
   * the step; shoulder and elbow are made expensive so orientation changes go to the wrist first.
   */
  solve(mj, d, targetPos, targetQuat, {
    iterations = 4, rotWeight = 0.35, damping = 1e-3, postureGain = 0.2, maxStep = 0.3, jointScale = null,
  } = {}) {
    const { m, e, J, qc, jnt } = this
    this.w.scale.set(jointScale ?? DEFAULT_JOINT_SCALE)
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
      this.prioritySolve(damping, postureGain, maxStep, d.qpos)
    }
  }

  /**
   * One iteration of the two-priority step on the current e and J:
   *   dq = Jp+ ep  +  N1 (Jr N1)+ (er - Jr Jp+ ep)  +  N (posture - q) * gain
   * with damped pseudo-inverses (3x3 inversions), N1 the null space of the position rows, N of all rows.
   */
  prioritySolve(damping, postureGain, maxStep, qpos) {
    const { J, qadr, w } = this
    const { Jp, Jr, dq, locked } = w
    locked.fill(0)
    // A joint at its limit that the step would push further out is removed from the solve (its columns
    // zeroed) and the step recomputed, so the other joints take over instead of the residual being lost
    for (let pass = 0; pass < 3; pass++) {
      for (let i = 0; i < 3 * N; i++) { Jp[i] = J[i]; Jr[i] = J[3 * N + i] }
      for (let k = 0; k < N; k++) if (locked[k]) for (let r = 0; r < 3; r++) { Jp[r * N + k] = 0; Jr[r * N + k] = 0 }
      this.priorityStep(damping, postureGain, qpos)
      let newlyLocked = false
      for (let k = 0; k < N; k++) {
        if (locked[k]) { dq[k] = 0; continue }
        const q = qpos[qadr[k]]
        if ((q <= this.lo[k] && dq[k] < 0) || (q >= this.hi[k] && dq[k] > 0)) { locked[k] = 1; newlyLocked = true }
      }
      if (!newlyLocked) break
    }
    for (let i = 0; i < N; i++) {
      const step = Math.max(-maxStep, Math.min(maxStep, dq[i]))
      qpos[qadr[i]] = Math.max(this.lo[i], Math.min(this.hi[i], qpos[qadr[i]] + step))
    }
  }

  // dq for the current Jp/Jr (locked columns already zeroed). Solved in a scaled joint space (column k of J
  // times scale[k]): a joint with a small scale is expensive, so the minimum-norm step prefers the wrist for
  // orientation changes instead of swinging the shoulder and elbow out.
  priorityStep(damping, postureGain, qpos) {
    const { e, qadr, w } = this
    const { Jp, Jr, Jpi, N1, JrN, JrNi, dq, tmp, M, locked, scale } = w
    for (let k = 0; k < N; k++) for (let r = 0; r < 3; r++) { Jp[r * N + k] *= scale[k]; Jr[r * N + k] *= scale[k] }
    pinv3(Jp, damping, Jpi, M)                          // Jpi: 7x3
    for (let i = 0; i < N; i++) dq[i] = Jpi[3 * i] * e[0] + Jpi[3 * i + 1] * e[1] + Jpi[3 * i + 2] * e[2]
    // N1 = I - Jpi Jp
    for (let i = 0; i < N; i++) for (let k = 0; k < N; k++) {
      let s = i === k ? 1 : 0
      for (let r = 0; r < 3; r++) s -= Jpi[3 * i + r] * Jp[r * N + k]
      N1[i * N + k] = s
    }
    // orientation residual after the position step, solved inside N1
    for (let r = 0; r < 3; r++) {
      let s = e[3 + r]
      for (let k = 0; k < N; k++) s -= Jr[r * N + k] * dq[k]
      tmp[r] = s
      for (let k = 0; k < N; k++) { let t = 0; for (let j = 0; j < N; j++) t += Jr[r * N + j] * N1[j * N + k]; JrN[r * N + k] = t }
    }
    pinv3(JrN, damping, JrNi, M)
    for (let i = 0; i < N; i++) {
      let s = 0
      for (let k = 0; k < N; k++) s += N1[i * N + k] * (JrNi[3 * k] * tmp[0] + JrNi[3 * k + 1] * tmp[1] + JrNi[3 * k + 2] * tmp[2])
      dq[i] += s
    }
    // Posture in the null space of both tasks: p1 = (I - Jp+ Jp) p, then p2 = p1 - N1 (Jr N1)+ (Jr p1). These
    // projectors use a far smaller damping than the task steps: with the task damping the projection leaks
    // near a stretched arm and the posture pull would settle the palm millimetres off the target.
    const { Jpn, N1n, JrNm, JrNmi, p, p1 } = w
    pinv3(Jp, NULL_DAMPING, Jpn, M)
    for (let i = 0; i < N; i++) for (let k = 0; k < N; k++) {
      let s = i === k ? 1 : 0
      for (let r = 0; r < 3; r++) s -= Jpn[3 * i + r] * Jp[r * N + k]
      N1n[i * N + k] = s
    }
    for (let r = 0; r < 3; r++) for (let k = 0; k < N; k++) { let t = 0; for (let j = 0; j < N; j++) t += Jr[r * N + j] * N1n[j * N + k]; JrNm[r * N + k] = t }
    pinv3(JrNm, NULL_DAMPING, JrNmi, M)
    for (let i = 0; i < N; i++) p[i] = locked[i] ? 0 : postureGain * (this.posture[i] - qpos[qadr[i]]) / scale[i]
    for (let i = 0; i < N; i++) { let s = 0; for (let k = 0; k < N; k++) s += N1n[i * N + k] * p[k]; p1[i] = s }
    for (let r = 0; r < 3; r++) { let s = 0; for (let k = 0; k < N; k++) s += Jr[r * N + k] * p1[k]; tmp[r] = s }
    for (let i = 0; i < N; i++) {
      let s = p1[i]
      for (let k = 0; k < N; k++) s -= N1n[i * N + k] * (JrNmi[3 * k] * tmp[0] + JrNmi[3 * k + 1] * tmp[1] + JrNmi[3 * k + 2] * tmp[2])
      dq[i] += s
    }
    for (let i = 0; i < N; i++) dq[i] *= scale[i]
  }
}

// shoulder 0.35, elbow 0.5, wrist 1: see priorityStep
const DEFAULT_JOINT_SCALE = [0.35, 0.35, 0.35, 0.5, 1, 1, 1]
const NULL_DAMPING = 1e-8 // for the null-space projectors (the task steps use the `damping` option)

// Damped pseudo-inverse of a 3x7 matrix: out (7x3) = A^T (A A^T + damping I)^-1; M is 3x3 scratch
function pinv3(A, damping, out, M) {
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    let s = r === c ? damping : 0
    for (let k = 0; k < N; k++) s += A[r * N + k] * A[c * N + k]
    M[3 * r + c] = s
  }
  const [a, b, c, d, e, f, g, h, i] = M
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
  const inv = [(e * i - f * h), (c * h - b * i), (b * f - c * e), (f * g - d * i), (a * i - c * g), (c * d - a * f), (d * h - e * g), (b * g - a * h), (a * e - b * d)].map(x => x / det)
  for (let k = 0; k < N; k++) for (let c2 = 0; c2 < 3; c2++) {
    let s = 0
    for (let r = 0; r < 3; r++) s += A[r * N + k] * inv[3 * r + c2]
    out[3 * k + c2] = s
  }
}
