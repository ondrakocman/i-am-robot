// Tube-into-box task on the fixed-base G1 + Dex3 model. Owns the MuJoCo state, turns operator input into
// actuator targets (IK for the arms, retargeted curls for the fingers), checks task success and records
// episodes. Pure JS: runs in the physics worker and in Node (scripts/sim-check.mjs).

import { ArmIK, ARM_JOINTS } from './ik.js'
import { EpisodeRecorder } from './episode.js'

export const CONTROL_HZ = 50
export const SIDES = ['left', 'right']
export const FINGER_JOINTS = ['thumb_0', 'thumb_1', 'thumb_2', 'index_0', 'index_1', 'middle_0', 'middle_1']

// Per-hand operator input: tracked, palm position (3), palm quaternion w,x,y,z (4), finger commands (7):
// thumb rotation in [-1, 1], then curls in [0, 1] for thumb_1, thumb_2, index_0, index_1, middle_0, middle_1.
// Positions/orientations are in the MuJoCo world frame (x forward, y left, z up).
export const HAND_INPUT = 15
export const INPUT_SIZE = 2 * HAND_INPUT
// Raw operator data, recorded for re-retargeting later: head pose (pos 3 + quat wxyz 4), then for each hand
// the 25 WebXR joints (pos 3 + quat wxyz 4), all in the MuJoCo world frame.
export const RAW_SIZE = 7 + 2 * 25 * 7

export const TASK = {
  name: 'tube_box',
  instruction: 'Put the orange tube into the blue box',
  tubeHome: [0.31, 0.15],
  tubeJitter: 0.025,
  boxInner: [0.09, 0.1],        // half extents of the box interior (x, y)
  boxMaxZ: 0.89,                // tube center must be below this (box rim is at 0.86)
  restSpeed: 0.05,              // m/s
  successHold: 0.5,             // s resting in the box with both hands off the tube
  dropZ: 0.68,                  // tube center below this = fell off the table
  timeout: 60,                  // s
  endHold: 1.5,                 // s to show the outcome before the next episode
  resetButton: [0.18, 0.36, 1.06],
  resetRadius: 0.06,
  resetHold: 0.6,
  // Physics randomization per episode (logged in the header)
  tubeMassRange: [0.2, 0.4],      // kg
  tubeFrictionRange: [0.5, 0.9],
  // Episodes whose arm joints exceed this are flagged 'fast_motion' (normal teleop stays < 3 rad/s)
  fastMotion: 6,                  // rad/s
}

// Start posture: hands raised near the chest, clear of the objects (grip ~(0.22, +-0.13, 1.03))
const READY = {
  shoulder_pitch: 0.3, shoulder_roll: 0.3, shoulder_yaw: 0, elbow: -0.8,
  wrist_roll: 0, wrist_pitch: 0, wrist_yaw: 0,
}
// IK redundancy bias: elbow down and slightly out
const POSTURE = {
  shoulder_pitch: 0, shoulder_roll: 0.3, shoulder_yaw: 0, elbow: 0,
  wrist_roll: 0, wrist_pitch: 0, wrist_yaw: 0,
}
const MIRRORED = new Set(['shoulder_roll', 'shoulder_yaw', 'wrist_roll', 'wrist_yaw'])
// Joint speed limits applied to the commanded targets (rad/s)
const ARM_SPEED = [3, 3, 3, 3, 5, 5, 5]
// Anti-windup: the commanded target may lead the measured joint by at most this much (rad). When the hand
// is blocked by contact the target stops running ahead, so the motor pushes with a bounded force and the
// arm doesn't whip when it comes free. With kp=80 this caps the extra torque at ~10 Nm (wrist kp=40: ~5 Nm).
const ARM_LEAD = 0.12

function mulberry32(seed) {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6D2B79F5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

const smooth = (a, b, u) => { u = Math.max(0, Math.min(1, u)); u = u * u * (3 - 2 * u); return a + (b - a) * u }

export class TubeBoxSim {
  /**
   * @param mj      loaded MuJoCo module
   * @param m       MjModel of public/mujoco/tube_box.xml
   * @param opts.onEpisode  called with { header, frames: Float32Array } whenever an episode ends
   * @param opts.autopilot  drive the left hand with a scripted pick-and-place (testing / desktop demo)
   */
  constructor(mj, m, { seed = (Math.random() * 2 ** 31) >>> 0, autopilot = false, onEpisode = null, meta = {}, armLead = ARM_LEAD } = {}) {
    this.armLead = armLead
    this.mj = mj
    this.m = m
    this.d = new mj.MjData(m)
    this.ikData = new mj.MjData(m)
    this.seed = seed
    this.autopilot = autopilot
    this.onEpisode = onEpisode
    this.meta = meta

    const opt = m.opt
    this.dt = opt.timestep
    this.stepsPerControl = Math.max(1, Math.round(1 / (CONTROL_HZ * this.dt)))
    this.controlDt = this.stepsPerControl * this.dt
    this.steps = 0

    const id = (type, name) => {
      const i = mj.mj_name2id(m, mj.mjtObj[type].value, name)
      if (i < 0) throw new Error(`model is missing ${name}`)
      return i
    }
    this.name = (type, i) => mj.mj_id2name(m, mj.mjtObj[type].value, i) ?? ''

    this.tubeBody = id('mjOBJ_BODY', 'tube')
    this.tubeGeom = id('mjOBJ_GEOM', 'tube')
    this.tubeMass0 = m.body_mass[this.tubeBody]
    this.tubeInertia0 = Array.from(m.body_inertia.slice(3 * this.tubeBody, 3 * this.tubeBody + 3))
    const tubeJnt = id('mjOBJ_JOINT', 'tube_free')
    this.tubeQ = m.jnt_qposadr[tubeJnt]
    this.tubeV = m.jnt_dofadr[tubeJnt]
    this.boxBody = id('mjOBJ_BODY', 'box')
    this.eyeSite = id('mjOBJ_SITE', 'eye')

    const mirror = (pose, side) => ARM_JOINTS.map(n => (MIRRORED.has(n) && side === 'right' ? -1 : 1) * pose[n])
    this.arms = SIDES.map(side => {
      const posture = mirror(POSTURE, side)
      const fingerJnt = FINGER_JOINTS.map(n => id('mjOBJ_JOINT', `${side}_hand_${n}_joint`))
      return {
        side,
        ready: mirror(READY, side),
        ik: new ArmIK(mj, m, side, posture),
        act: Int32Array.from(ARM_JOINTS, n => id('mjOBJ_ACTUATOR', `${side}_${n}_joint`)),
        fingerAct: Int32Array.from(FINGER_JOINTS, n => id('mjOBJ_ACTUATOR', `${side}_hand_${n}_joint`)),
        fingerLo: fingerJnt.map(j => m.jnt_range[2 * j]),
        fingerHi: fingerJnt.map(j => m.jnt_range[2 * j + 1]),
        palmSite: id('mjOBJ_SITE', `${side}_palm`),
        gripSite: id('mjOBJ_SITE', `${side}_grip`),
        qCmd: new Float64Array(ARM_JOINTS.length),
        fingerCmd: new Float64Array(FINGER_JOINTS.length),
      }
    })

    // Which hand (0 left, 1 right, -1 none) each body belongs to. The palm geom lives on the wrist_yaw body.
    this.handOfBody = new Int8Array(m.nbody).fill(-1)
    for (let b = 0; b < m.nbody; b++) {
      const n = this.name('mjOBJ_BODY', b)
      SIDES.forEach((s, i) => {
        if (n.startsWith(`${s}_hand_`) || n === `${s}_wrist_yaw_link`) this.handOfBody[b] = i
      })
    }

    this.qposNames = []
    this.qvelNames = []
    for (let j = 0; j < m.njnt; j++) {
      const n = this.name('mjOBJ_JOINT', j)
      if (m.jnt_type[j] === 0) { // free joint
        this.qposNames.push(...['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'].map(s => `${n}:${s}`))
        this.qvelNames.push(...['vx', 'vy', 'vz', 'wx', 'wy', 'wz'].map(s => `${n}:${s}`))
      } else {
        this.qposNames.push(n)
        this.qvelNames.push(n)
      }
    }
    this.actuatorNames = Array.from({ length: m.nu }, (_, a) => this.name('mjOBJ_ACTUATOR', a))

    this.armDof = this.arms.flatMap(a => Array.from(a.ik.jnt, j => m.jnt_dofadr[j]))
    this.input = new Float32Array(INPUT_SIZE)
    this.raw = new Float32Array(RAW_SIZE)
    this.autoInput = new Float32Array(INPUT_SIZE)
    this.touching = new Uint8Array(2)
    this.recorder = new EpisodeRecorder([
      { name: 'time', size: 1 },
      { name: 'action', size: m.nu },
      { name: 'qpos', size: m.nq },
      { name: 'qvel', size: m.nv },
      { name: 'input', size: INPUT_SIZE },
      { name: 'raw', size: RAW_SIZE },
      { name: 'touching', size: 2 },
    ])

    this.episode = 0
    this.reset()
  }

  setInput(input, raw) {
    this.input.set(input)
    if (raw) this.raw.set(raw)
  }

  requestReset() { this.pendingReset = true }

  reset() {
    const { mj, m, d } = this
    if (this.status === 'running') this.endEpisode('aborted')
    this.episode++
    const rng = mulberry32(this.seed + this.episode * 9973)
    const jitter = () => (rng() * 2 - 1) * TASK.tubeJitter
    const uniform = ([lo, hi]) => lo + (hi - lo) * rng()

    mj.mj_resetData(m, d)
    this.setPhysics({ tube_mass: uniform(TASK.tubeMassRange), tube_friction: uniform(TASK.tubeFrictionRange) })
    for (const arm of this.arms) {
      for (let k = 0; k < arm.qCmd.length; k++) {
        arm.qCmd[k] = arm.ready[k]
        d.qpos[arm.ik.qadr[k]] = arm.ready[k]
        d.ctrl[arm.act[k]] = Math.fround(arm.ready[k])
      }
      arm.fingerCmd.fill(0)
    }
    const tube = [TASK.tubeHome[0] + jitter(), TASK.tubeHome[1] + jitter(), 0.861]
    d.qpos.set([...tube, 1, 0, 0, 0], this.tubeQ)
    mj.mj_forward(m, d)

    const b = 3 * this.boxBody
    this.layout = { tube, box: [m.body_pos[b], m.body_pos[b + 1], m.body_pos[b + 2]] }
    this.initialQpos = Array.from(d.qpos)
    this.peakArmVel = 0
    this.status = 'waiting' // until the operator's hands show up
    this.steps = 0
    this.startTime = 0
    this.endTime = 0
    this.successTimer = 0
    this.resetTimer = 0
    this.pendingReset = false
    this.recorder.clear()
    this.autoReady = null
  }

  /** Applies randomized object properties; also used to restore them for replay. */
  setPhysics({ tube_mass, tube_friction }) {
    const { mj, m, d } = this
    const scale = tube_mass / this.tubeMass0
    m.body_mass[this.tubeBody] = tube_mass
    for (let k = 0; k < 3; k++) m.body_inertia[3 * this.tubeBody + k] = this.tubeInertia0[k] * scale
    m.geom_friction[3 * this.tubeGeom] = tube_friction
    mj.mj_setConst(m, d)
    this.physics = { tube_mass, tube_friction }
  }

  /**
   * One physics step; runs the controller every `stepsPerControl` steps. Physics stays frozen until the
   * operator's hands appear, so every episode starts exactly from `initial_qpos` with zero velocity and
   * replays from its log alone. Returns false while frozen.
   */
  step() {
    if (this.steps % this.stepsPerControl === 0) this.control()
    if (this.status === 'waiting') return false
    this.mj.mj_step(this.m, this.d)
    this.steps++
    return true
  }

  control() {
    if (this.pendingReset) this.reset()
    const { mj, d } = this
    const input = this.autopilot ? this.autopilotInput() : this.input
    const anyTracked = input[0] > 0.5 || input[HAND_INPUT] > 0.5
    if (this.status === 'waiting' && anyTracked) {
      this.status = 'running'
      this.startTime = d.time
    }

    for (let s = 0; s < 2; s++) {
      const arm = this.arms[s]
      const o = s * HAND_INPUT
      if (input[o] > 0.5) {
        const q = this.ikData.qpos
        const { qadr } = arm.ik
        for (let k = 0; k < qadr.length; k++) q[qadr[k]] = arm.qCmd[k]
        arm.ik.solve(mj, this.ikData, input.subarray(o + 1, o + 4), input.subarray(o + 4, o + 8))
        for (let k = 0; k < qadr.length; k++) {
          const lim = ARM_SPEED[k] * this.controlDt
          arm.qCmd[k] += Math.max(-lim, Math.min(lim, q[qadr[k]] - arm.qCmd[k]))
          const actual = d.qpos[qadr[k]]
          arm.qCmd[k] = Math.max(actual - this.armLead, Math.min(actual + this.armLead, arm.qCmd[k]))
        }
        for (let k = 0; k < FINGER_JOINTS.length; k++) arm.fingerCmd[k] = this.fingerTarget(arm, k, input[o + 8 + k])
      }
      // float32-exact targets so a recorded episode replays bit-for-bit from its log
      for (let k = 0; k < arm.act.length; k++) d.ctrl[arm.act[k]] = Math.fround(arm.qCmd[k])
      for (let k = 0; k < arm.fingerAct.length; k++) d.ctrl[arm.fingerAct[k]] = Math.fround(arm.fingerCmd[k])
    }

    this.updateTask()
    if (this.status === 'running') {
      for (const dof of this.armDof) this.peakArmVel = Math.max(this.peakArmVel, Math.abs(d.qvel[dof]))
      this.recorder.push([d.time - this.startTime, d.ctrl, d.qpos, d.qvel, input, this.raw, this.touching])
    }
  }

  // Same mapping the kinematic app used: thumb_0 is a signed rotation, everything else curls toward the
  // joint limit with the larger magnitude.
  fingerTarget(arm, k, v) {
    const lo = arm.fingerLo[k], hi = arm.fingerHi[k]
    const target = k === 0
      ? v * Math.min(Math.abs(lo), Math.abs(hi))
      : v * (Math.abs(lo) > Math.abs(hi) ? lo : hi)
    return Math.max(lo, Math.min(hi, target))
  }

  updateTask() {
    const { m, d } = this
    const cdt = this.controlDt
    const q = this.tubeQ, v = this.tubeV
    const px = d.qpos[q], py = d.qpos[q + 1], pz = d.qpos[q + 2]
    const speed = Math.hypot(d.qvel[v], d.qvel[v + 1], d.qvel[v + 2])

    this.touching[0] = this.touching[1] = 0
    const contacts = d.contact
    for (let i = 0, n = contacts.size(); i < n; i++) {
      const c = contacts.get(i)
      const other = c.geom1 === this.tubeGeom ? c.geom2 : c.geom2 === this.tubeGeom ? c.geom1 : -1
      c.delete()
      if (other < 0) continue
      const hand = this.handOfBody[m.geom_bodyid[other]]
      if (hand >= 0) this.touching[hand] = 1
    }
    contacts.delete()

    const [bx, by] = this.layout.box
    this.tubeInBox = Math.abs(px - bx) < TASK.boxInner[0] && Math.abs(py - by) < TASK.boxInner[1] && pz < TASK.boxMaxZ
    const settled = this.tubeInBox && speed < TASK.restSpeed && !this.touching[0] && !this.touching[1]

    if (this.status === 'running') {
      this.successTimer = settled ? this.successTimer + cdt : 0
      if (this.successTimer >= TASK.successHold) this.endEpisode('success')
      else if (pz < TASK.dropZ) this.endEpisode('dropped')
      else if (d.time - this.startTime > TASK.timeout) this.endEpisode('timeout')
    } else if (this.status !== 'waiting' && d.time - this.endTime > TASK.endHold) {
      this.reset()
      return
    }

    let near = false
    for (const arm of this.arms) {
      const s = 3 * arm.palmSite
      const [rx, ry, rz] = TASK.resetButton
      if (Math.hypot(d.site_xpos[s] - rx, d.site_xpos[s + 1] - ry, d.site_xpos[s + 2] - rz) < TASK.resetRadius) near = true
    }
    this.resetTimer = near ? this.resetTimer + cdt : 0
    if (this.resetTimer >= TASK.resetHold) this.pendingReset = true
  }

  endEpisode(outcome) {
    const { m, d } = this
    this.status = outcome
    this.endTime = d.time
    const frames = this.recorder.snapshot()
    const duration = d.time - this.startTime
    if (!this.onEpisode || duration < 1) return
    this.onEpisode({
      header: {
        format: 'iamr-episode-v1',
        task: TASK.name,
        instruction: TASK.instruction,
        outcome,
        success: outcome === 'success',
        episode: this.episode,
        seed: this.seed,
        duration,
        recorded_at: new Date().toISOString(),
        timestep: this.dt,
        control_hz: 1 / this.controlDt,
        steps_per_control: this.stepsPerControl,
        layout: this.layout,
        physics: this.physics,
        peak_arm_velocity: this.peakArmVel,
        flags: this.peakArmVel > TASK.fastMotion ? ['fast_motion'] : [],
        initial_qpos: this.initialQpos,
        nq: m.nq, nv: m.nv, nu: m.nu,
        qpos_names: this.qposNames,
        qvel_names: this.qvelNames,
        actuator_names: this.actuatorNames,
        fields: this.recorder.fields,
        frame_size: this.recorder.frameSize,
        frames: this.recorder.frames,
        ...this.meta,
      },
      frames,
    })
  }

  /** Body poses for rendering: [x, y, z, qw, qx, qy, qz] per body. */
  writeBodies(out) {
    const { xpos, xquat } = this.d
    for (let b = 0, n = this.m.nbody; b < n; b++) {
      const o = 7 * b
      out[o] = xpos[3 * b]; out[o + 1] = xpos[3 * b + 1]; out[o + 2] = xpos[3 * b + 2]
      out[o + 3] = xquat[4 * b]; out[o + 4] = xquat[4 * b + 1]; out[o + 5] = xquat[4 * b + 2]; out[o + 6] = xquat[4 * b + 3]
    }
    return out
  }

  eyePosition() {
    const s = 3 * this.eyeSite
    return Array.from(this.d.site_xpos.slice(s, s + 3))
  }

  // Scripted left-hand pick and place: side grasp, carry over the box, release, pull back.
  autopilotInput() {
    const { d } = this
    const t = this.status === 'running' ? d.time - this.startTime : 0
    const [tx, ty] = this.layout.tube
    const [bx, by] = this.layout.box
    const arm = this.arms[0]
    if (!this.autoReady) {
      const s = 3 * arm.gripSite
      this.autoReady = Array.from(d.site_xpos.slice(s, s + 3))
    }
    const r = this.autoReady
    const grip = this.layout.tube[2] + 0.02 // grasp a little above the tube's center
    const lift = grip + 0.13
    // [time, grip x, y, z, palm yaw, finger closure]; yaw turns the fingers inward to reach across the body
    const keys = [
      [0, r[0], r[1], r[2], 0, 0],
      [1.0, tx, ty + 0.09, lift, 0, 0],
      [2.0, tx, ty + 0.09, grip, 0, 0],
      [3.0, tx, ty + 0.012, grip, 0, 0],
      [3.8, tx, ty + 0.012, grip, 0, 1],
      [5.0, tx, ty + 0.012, lift, 0, 1],
      [7.0, bx - 0.01, by + 0.06, lift, -0.5, 1],
      [8.0, bx - 0.01, by + 0.06, grip + 0.02, -0.5, 1],
      [8.6, bx - 0.01, by + 0.06, grip + 0.02, -0.5, 0],
      [10.0, r[0], r[1], r[2], 0, 0],
    ]
    let i = 0
    while (i < keys.length - 2 && t > keys[i + 1][0]) i++
    const [t0, ...a] = keys[i]
    const [t1, ...b] = keys[i + 1]
    const u = (t - t0) / (t1 - t0)
    const [gx, gy, gz, yaw, close] = a.map((v, k) => smooth(v, b[k], u))

    // palm = grip - R(yaw) (grip offset - palm offset); palm frame = world frame rotated by yaw about z
    const c = Math.cos(yaw), s = Math.sin(yaw)
    const ox = 0.0735, oy = -0.038
    const inp = this.autoInput
    inp.fill(0)
    inp[0] = 1
    inp[1] = gx - (c * ox - s * oy); inp[2] = gy - (s * ox + c * oy); inp[3] = gz
    inp[4] = Math.cos(yaw / 2); inp[7] = Math.sin(yaw / 2)
    for (let k = 1; k < 7; k++) inp[8 + k] = close
    return inp
  }
}
