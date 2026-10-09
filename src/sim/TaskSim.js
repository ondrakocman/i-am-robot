// Generic manipulation task on the fixed-base G1 + Dex3 model. Owns the MuJoCo state, turns operator input into
// actuator targets (IK for the arms, retargeted curls for the fingers), runs the task module's reset / goal
// logic, and records episodes. Pure JS: runs in the physics worker and in Node (scripts/sim-check.mjs).
//
// A task module (src/sim/tasks/*.js) provides:
//   name, instruction (language instruction logged with each episode), title (short HUD text), scene (xml under
//   public/), objects (free bodies, joint named `${body}_free`), timeout, reset(sim, rng) -> layout,
//   randomize?(rng) -> physics params, and either
//     goal(sim) -> boolean with dropZ   (static tasks: success once the goal holds with objects at rest and hands off)
//   or
//     update(sim) -> outcome | null     (dynamic tasks: called every control tick, runs its own spawning/scoring and
//                                        returns 'success' / 'partial' / ... to end the episode)
//   plus optional hud(sim) -> string for the panel, result(sim) -> per-episode scoring for the header,
//   autopilot?(sim, t) -> per-hand grip targets, solved?(sim) for the headless check.
//   Per-episode task state belongs in `sim.taskState` (set in reset), never on the module object.
//   Teleports done by a task while running must go through sim.teleportObject so replay can re-apply them.
//   An event's `tick` is the index of the last frame recorded before the teleport: replay applies it after
//   comparing that frame and before stepping on to the next (see replay.js).

import { ArmIK, ARM_JOINTS } from './ik.js'
import { EpisodeRecorder, EPISODE_FORMAT } from './episode.js'
import { writeHandInput } from './autopilot.js'

export const CONTROL_HZ = 50
export const SIDES = ['left', 'right']
export const FINGER_JOINTS = ['thumb_0', 'thumb_1', 'thumb_2', 'index_0', 'index_1', 'middle_0', 'middle_1']
/** Physics timesteps that divide the control period exactly (so control stays at CONTROL_HZ). */
export const TIMESTEPS = [0.001, 0.002, 0.0025, 0.004, 0.005]

// Per-hand operator input: tracked, palm position (3), palm quaternion w,x,y,z (4), finger commands (7):
// thumb rotation in [-1, 1], then curls in [0, 1] for thumb_1, thumb_2, index_0, index_1, middle_0, middle_1.
// Positions/orientations are in the MuJoCo world frame (x forward, y left, z up).
export const HAND_INPUT = 15
export const INPUT_SIZE = 2 * HAND_INPUT
// Raw operator data, recorded for re-retargeting later: viewer (head) pose (pos 3 + quat wxyz 4), then for
// each hand the 25 WebXR joints (pos 3 + quat wxyz 4), all in the MuJoCo world frame. An untracked joint is
// all zeros (its quaternion has zero norm).
export const RAW_SIZE = 7 + 2 * 25 * 7

// Shared across tasks
export const COMMON = {
  restSpeed: 0.05,              // m/s: objects must be slower than this for the goal to count
  successHold: 0.5,             // s the goal must hold with both hands off the objects
  endHold: 1.5,                 // s to show the outcome before the next episode
  minEpisode: 1,                // s: shorter episodes are discarded, not saved
  resetButton: [0.18, 0.36, 1.06],
  resetRadius: 0.06,
  resetHold: 0.6,
  // Episodes whose arm joints exceed this are flagged 'fast_motion' (normal teleop stays < 3 rad/s)
  fastMotion: 6,                // rad/s
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
const ROBOT_BODY = /_link$|^pelvis$/

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

export class TaskSim {
  /**
   * @param mj      loaded MuJoCo module
   * @param m       MjModel of the task's scene
   * @param task    task module
   * @param opts.onEpisode  called with { header, frames: Float32Array } whenever an episode ends
   * @param opts.autopilot  drive the hands with the task's scripted demonstration (testing / desktop demo)
   * @param opts.meta       extra header fields (session id, asset hashes, app version)
   */
  constructor(mj, m, task, { seed = (Math.random() * 2 ** 31) >>> 0, autopilot = false, onEpisode = null, meta = {}, armLead = ARM_LEAD } = {}) {
    this.mj = mj
    this.m = m
    this.task = task
    this.d = new mj.MjData(m)
    this.ikData = new mj.MjData(m)
    this.seed = seed
    this.autopilot = autopilot && !!task.autopilot
    this.onEpisode = onEpisode
    this.meta = meta
    this.armLead = armLead

    this.dt = m.opt.timestep
    if (!TIMESTEPS.some(t => Math.abs(t - this.dt) < 1e-9)) {
      throw new Error(`timestep ${this.dt} must be one of ${TIMESTEPS.join(', ')} so control stays at ${CONTROL_HZ} Hz`)
    }
    this.stepsPerControl = Math.round(1 / (CONTROL_HZ * this.dt))
    this.controlDt = this.stepsPerControl * this.dt
    this.steps = 0

    const id = (type, name) => {
      const i = mj.mj_name2id(m, mj.mjtObj[type].value, name)
      if (i < 0) throw new Error(`model is missing ${name}`)
      return i
    }
    this.name = (type, i) => mj.mj_id2name(m, mj.mjtObj[type].value, i) ?? ''
    this.bodyId = name => id('mjOBJ_BODY', name)
    this.eyeSite = id('mjOBJ_SITE', 'eye')

    const mirror = (pose, side) => ARM_JOINTS.map(n => (MIRRORED.has(n) && side === 'right' ? -1 : 1) * pose[n])
    this.arms = SIDES.map(side => {
      const fingerJnt = FINGER_JOINTS.map(n => id('mjOBJ_JOINT', `${side}_hand_${n}_joint`))
      return {
        side,
        ready: mirror(READY, side),
        ik: new ArmIK(mj, m, side, mirror(POSTURE, side)),
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
    this.armDof = this.arms.flatMap(a => Array.from(a.ik.jnt, j => m.jnt_dofadr[j]))

    // Robot actuators come first in the model (the robot file is included before the scene); scene actuators
    // such as belt motors follow and are recorded in `action` too, after the robot's.
    const actuatorBody = a => this.name('mjOBJ_BODY', m.jnt_bodyid[m.actuator_trnid[2 * a]])
    this.robotNu = 0
    for (let a = 0; a < m.nu; a++) {
      if (ROBOT_BODY.test(actuatorBody(a))) {
        if (a !== this.robotNu) throw new Error('robot actuators must precede scene actuators')
        this.robotNu++
      }
    }

    // Which hand (0 left, 1 right, -1 none) each body belongs to. The palm geom lives on the wrist_yaw body.
    this.handOfBody = new Int8Array(m.nbody).fill(-1)
    for (let b = 0; b < m.nbody; b++) {
      const n = this.name('mjOBJ_BODY', b)
      SIDES.forEach((s, i) => {
        if (n.startsWith(`${s}_hand_`) || n === `${s}_wrist_yaw_link`) this.handOfBody[b] = i
      })
    }

    // Task objects: free bodies the hands manipulate
    this.objectOfGeom = new Int16Array(m.ngeom).fill(-1)
    this.objects = task.objects.map((name, i) => {
      const body = id('mjOBJ_BODY', name)
      const jnt = id('mjOBJ_JOINT', `${name}_free`)
      const geoms = []
      for (let g = 0; g < m.ngeom; g++) {
        if (m.geom_bodyid[g] === body && (m.geom_contype[g] || m.geom_conaffinity[g])) { geoms.push(g); this.objectOfGeom[g] = i }
      }
      return {
        name, body, geoms,
        q: m.jnt_qposadr[jnt], v: m.jnt_dofadr[jnt],
        mass0: m.body_mass[body],
        inertia0: Array.from(m.body_inertia.slice(3 * body, 3 * body + 3)),
      }
    })
    this.touch = new Uint8Array(2 * this.objects.length) // [object][hand]
    this.touching = new Uint8Array(2)                     // any object, per hand

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

    this.input = new Float32Array(INPUT_SIZE)
    this.raw = new Float32Array(RAW_SIZE)
    this.autoInput = new Float32Array(INPUT_SIZE)
    this.recorder = new EpisodeRecorder([
      { name: 'time', size: 1 },
      { name: 'action', size: m.nu },
      { name: 'qpos', size: m.nq },
      { name: 'qvel', size: m.nv },
      { name: 'input', size: INPUT_SIZE },
      { name: 'raw', size: RAW_SIZE },
      { name: 'touching', size: 2 },
    ])
    this.rtf = { min: Infinity, sum: 0, n: 0 } // real-time factor samples supplied by the host while running

    this.episode = 0
    this.reset()
  }

  setInput(input, raw) {
    this.input.set(input)
    if (raw) this.raw.set(raw)
  }

  /** Marks both hands untracked (operator left, tracking lost): a running episode keeps going on held targets. */
  clearInput() {
    this.input.fill(0)
    this.raw.fill(0)
  }

  requestReset() { this.pendingReset = true }

  /** Ends a running episode as 'aborted' and resets; used when the XR session ends. */
  abort() {
    if (this.status === 'running') this.endEpisode('aborted')
    this.pendingReset = true
  }

  /** Host-side real-time factor sample (1 = keeping up), folded into the episode header. */
  reportRealtime(rtf) {
    if (this.status !== 'running') return
    this.rtf.min = Math.min(this.rtf.min, rtf)
    this.rtf.sum += rtf
    this.rtf.n++
  }

  reset() {
    const { mj, m, d, task } = this
    if (this.status === 'running') this.endEpisode('aborted')
    this.episode++
    const rng = mulberry32(this.seed + this.episode * 9973)

    mj.mj_resetData(m, d)
    // mj_setConst (inside setPhysics) rewrites qpos to the model default, so it must run before posing anything
    this.setPhysics(task.randomize ? task.randomize(rng) : {})
    for (const arm of this.arms) {
      for (let k = 0; k < arm.qCmd.length; k++) {
        arm.qCmd[k] = arm.ready[k]
        d.qpos[arm.ik.qadr[k]] = arm.ready[k]
        d.ctrl[arm.act[k]] = Math.fround(arm.ready[k])
      }
      arm.fingerCmd.fill(0)
    }
    this.taskState = null
    this.layout = task.reset(this, rng)
    mj.mj_forward(m, d)
    this.readyGrip = this.arms.map(arm => Array.from(d.site_xpos.slice(3 * arm.gripSite, 3 * arm.gripSite + 3)))

    this.initialQpos = Array.from(d.qpos)
    this.initialCtrl = Array.from(d.ctrl)
    this.events = []
    this.scheduled = []
    this.frameIndex = 0
    this.peakArmVel = 0
    this.rtf = { min: Infinity, sum: 0, n: 0 }
    this.status = 'waiting' // until the operator's hands show up
    this.steps = 0
    this.startTime = 0
    this.startedAt = null
    this.endTime = 0
    this.successTimer = 0
    this.resetTimer = 0
    this.pendingReset = false
    this.recorder.clear()
  }

  // ── Object helpers for task modules ──────────────────────────────────────

  placeObject(i, [x, y, z], yaw = 0) {
    this.d.qpos.set([x, y, z, Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)], this.objects[i].q)
  }

  objectPos(i) {
    const q = this.objects[i].q
    return [this.d.qpos[q], this.d.qpos[q + 1], this.d.qpos[q + 2]]
  }

  objectSpeed(i) {
    const v = this.objects[i].v
    return Math.hypot(this.d.qvel[v], this.d.qvel[v + 1], this.d.qvel[v + 2])
  }

  /** z component of the object's own +z axis in the world: 1 upright, 0 on its side, -1 upside down. */
  objectUp(i) {
    const q = this.objects[i].q
    const x = this.d.qpos[q + 4], y = this.d.qpos[q + 5]
    return 1 - 2 * (x * x + y * y)
  }

  /** Runs fn right after frame `tick` is recorded, so any teleport it does lands on a frame boundary. */
  scheduleAtFrame(tick, fn) { this.scheduled.push({ tick, fn }) }

  /** Teleports an object mid-episode (spawning / despawning) and logs it so replay can re-apply it. */
  teleportObject(i, pos, quat = [1, 0, 0, 0]) {
    const obj = this.objects[i]
    this.d.qpos.set([...pos, ...quat], obj.q)
    this.d.qvel.fill(0, obj.v, obj.v + 6)
    if (this.status === 'running') this.events.push({ tick: this.frameIndex - 1, qpos_adr: obj.q, qpos: [...pos, ...quat], qvel_adr: obj.v })
  }

  /**
   * Deepest penetration (m, positive) among contacts involving a task object. Used by the headless check to
   * assert that resets start from a physically valid state.
   */
  maxObjectPenetration() {
    const contacts = this.d.contact
    let worst = 0
    for (let i = 0, n = contacts.size(); i < n; i++) {
      const c = contacts.get(i)
      if ((this.objectOfGeom[c.geom1] >= 0 || this.objectOfGeom[c.geom2] >= 0) && c.dist < 0) worst = Math.max(worst, -c.dist)
      c.delete()
    }
    contacts.delete()
    return worst
  }

  /** Applies randomized object properties { [objectName]: { mass?, friction? } } (replay.js mirrors this). */
  setPhysics(params) {
    const { mj, m, d } = this
    for (const obj of this.objects) {
      const p = params[obj.name]
      if (!p) continue
      if (p.mass !== undefined) {
        const scale = p.mass / obj.mass0
        m.body_mass[obj.body] = p.mass
        for (let k = 0; k < 3; k++) m.body_inertia[3 * obj.body + k] = obj.inertia0[k] * scale
      }
      if (p.friction !== undefined) for (const g of obj.geoms) m.geom_friction[3 * g] = p.friction
    }
    mj.mj_setConst(m, d)
    this.physics = params
  }

  // ── Stepping ─────────────────────────────────────────────────────────────

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
      this.startedAt = new Date().toISOString()
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

    this.scanContacts()
    if (this.status === 'running') {
      for (const dof of this.armDof) this.peakArmVel = Math.max(this.peakArmVel, Math.abs(d.qvel[dof]))
      this.recorder.push([d.time - this.startTime, d.ctrl, d.qpos, d.qvel, input, this.raw, this.touching])
      this.frameIndex++
      for (const s of this.scheduled.splice(0)) {
        if (s.tick === this.frameIndex - 1) s.fn()
        else if (s.tick > this.frameIndex - 1) this.scheduled.push(s)
      }
    }
    this.updateTask() // teleports logged here come after the frame just recorded
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

  /** Which hand touches which task object, from the current contact list. */
  scanContacts() {
    const { m, d } = this
    this.touch.fill(0)
    this.touching.fill(0)
    const contacts = d.contact
    for (let i = 0, n = contacts.size(); i < n; i++) {
      const c = contacts.get(i)
      const g1 = c.geom1, g2 = c.geom2
      c.delete()
      const o1 = this.objectOfGeom[g1], o2 = this.objectOfGeom[g2]
      const obj = o1 >= 0 ? o1 : o2
      if (obj < 0) continue
      const hand = this.handOfBody[m.geom_bodyid[o1 >= 0 ? g2 : g1]]
      if (hand < 0) continue
      this.touch[2 * obj + hand] = 1
      this.touching[hand] = 1
    }
    contacts.delete()
  }

  updateTask() {
    const { d, task } = this
    const cdt = this.controlDt

    if (task.update) {
      if (this.status === 'running') {
        const outcome = task.update(this)
        if (outcome) this.endEpisode(outcome)
        else if (d.time - this.startTime > task.timeout) this.endEpisode('timeout')
      }
    } else {
      let settled = task.goal(this) && !this.touching[0] && !this.touching[1]
      let dropped = false
      for (let i = 0; i < this.objects.length; i++) {
        if (this.objectSpeed(i) > COMMON.restSpeed) settled = false
        if (this.objectPos(i)[2] < task.dropZ) dropped = true
      }
      if (this.status === 'running') {
        this.successTimer = settled ? this.successTimer + cdt : 0
        if (this.successTimer >= COMMON.successHold) this.endEpisode('success')
        else if (dropped) this.endEpisode('dropped')
        else if (d.time - this.startTime > task.timeout) this.endEpisode('timeout')
      }
    }
    if (this.status !== 'running' && this.status !== 'waiting' && d.time - this.endTime > COMMON.endHold) {
      this.reset()
      return
    }

    let near = false
    for (const arm of this.arms) {
      const s = 3 * arm.palmSite
      const [rx, ry, rz] = COMMON.resetButton
      if (Math.hypot(d.site_xpos[s] - rx, d.site_xpos[s + 1] - ry, d.site_xpos[s + 2] - rz) < COMMON.resetRadius) near = true
    }
    this.resetTimer = near ? this.resetTimer + cdt : 0
    if (this.resetTimer >= COMMON.resetHold) this.pendingReset = true
  }

  /** Ends the running episode. Returns true if it was handed to onEpisode (long enough to keep). */
  endEpisode(outcome) {
    const { m, d, task } = this
    this.status = outcome
    this.endTime = d.time
    const duration = d.time - this.startTime
    this.lastSaved = this.onEpisode && duration >= COMMON.minEpisode
    if (!this.lastSaved) return false
    const frames = this.recorder.snapshot()
    this.onEpisode({
      header: {
        format: EPISODE_FORMAT,
        task: task.name,
        instruction: task.instruction,
        scene: task.scene,
        outcome,
        success: outcome === 'success',
        episode: this.episode,
        seed: this.seed,
        duration,
        started_at: this.startedAt,
        ended_at: new Date().toISOString(),
        timestep: this.dt,
        control_hz: 1 / this.controlDt,
        steps_per_control: this.stepsPerControl,
        autopilot: this.autopilot,
        layout: this.layout,
        physics: this.physics,
        objects: task.objects,
        peak_arm_velocity: this.peakArmVel,
        realtime_factor: this.rtf.n ? { min: this.rtf.min, mean: this.rtf.sum / this.rtf.n } : null,
        flags: this.peakArmVel > COMMON.fastMotion ? ['fast_motion'] : [],
        initial_qpos: this.initialQpos,
        initial_ctrl: this.initialCtrl,
        final_qpos: Array.from(d.qpos),
        final_qvel: Array.from(d.qvel),
        events: this.events,
        result: task.result ? task.result(this) : undefined,
        nq: m.nq, nv: m.nv, nu: m.nu,
        robot_nu: this.robotNu,
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
    return true
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

  // The task's scripted demonstration: returns per-hand [gx, gy, gz, yaw, close] (null = hand at rest)
  autopilotInput() {
    const t = this.status === 'running' ? this.d.time - this.startTime : 0
    const hands = this.task.autopilot(this, t)
    const inp = this.autoInput
    inp.fill(0)
    for (let s = 0; s < 2; s++) {
      const g = hands[s] ?? [...this.readyGrip[s], 0, 0]
      writeHandInput(inp, s * HAND_INPUT, s, g)
    }
    return inp
  }
}
