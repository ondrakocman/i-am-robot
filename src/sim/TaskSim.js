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
//   autopilot?(sim, t) -> per-hand grip targets, solved?(sim) and reachTargets?(sim) for the headless check,
//   materials (MuJoCo material name -> three.js look) and geometry (geom name -> custom visual) for the renderer.
//   Per-episode task state belongs in `sim.taskState` (set in reset), never on the module object.
//   Teleports done by a task while running must go through sim.teleportObject so replay can re-apply them.
//   update() runs after the frame's action was recorded: a task must only write ctrl in reset() (belt motors),
//   never in update(), or replay from the log would silently differ.
//   An event's `tick` is the index of the last frame recorded before the teleport: replay applies it after
//   comparing that frame and before stepping on to the next (see replay.js).
//   soft?: { bodies: { [name]: { half, cells, mass?, ... } }, belts?: [geom prefixes] } declares XPBD soft
//   parcels (soft.js) the task places with placeSoft/teleportSoft and reads with softPos/softUp/softSpeed;
//   randomizeSoft?(rng) -> per-body physics; their particle positions are recorded per frame and replayed.

import { ArmIK, ARM_JOINTS } from './ik.js'
import { EpisodeRecorder, EPISODE_FORMAT } from './episode.js'
import { writeHandInput } from './autopilot.js'
import { applyPhysics, compiledPhysics } from './replay.js'
import { SIDES, HAND_INPUT, INPUT_SIZE, INPUT_NAMES, RAW_SIZE, RAW_LAYOUT, handOfBodyName } from './inputLayout.js'
import { SoftWorld } from './soft.js'

export { SIDES, HAND_INPUT, INPUT_SIZE, INPUT_NAMES, RAW_SIZE, RAW_LAYOUT }
export const CONTROL_HZ = 50
export const FINGER_JOINTS = ['thumb_0', 'thumb_1', 'thumb_2', 'index_0', 'index_1', 'middle_0', 'middle_1']
/** Physics timesteps that divide the control period exactly (so control stays at CONTROL_HZ). */
export const TIMESTEPS = [0.001, 0.002, 0.0025, 0.004, 0.005]

// The operator input and raw tracking layouts are in inputLayout.js (shared with the renderer).

// Shared across tasks
export const COMMON = {
  restSpeed: 0.05,              // m/s: objects must be slower than this for the goal to count
  restSpin: 0.5,                // rad/s: and turning slower than this
  untrackedTimeout: 5,          // s without any tracked hand before a running episode is abandoned
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
// IK rest posture, approached in the null space of the palm task (so it never moves the palm): elbow down
// and slightly out
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
// MuJoCo warning names in mjtWarning order, from the loaded build (the enum has changed between releases).
// bad_qpos/bad_qvel/bad_qacc mean MuJoCo found an invalid state and reset the data; the others (constraint/
// contact buffer overflow, bad inertia or ctrl) corrupt the step.
function warningNames(mj) {
  return Object.entries(mj.mjtWarning).filter(([k]) => k.startsWith('mjWARN_')).sort((a, b) => a[1].value - b[1].value)
    .map(([k]) => k.slice('mjWARN_'.length).toLowerCase().replace(/^bad(\w)/, 'bad_$1').replace('cnstrfull', 'constraint_full').replace('contactfull', 'contact_full'))
}

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
  constructor(mj, m, task, { seed = (Math.random() * 2 ** 31) >>> 0, autopilot = false, onEpisode = null, meta = {} } = {}) {
    this.mj = mj
    this.m = m
    this.task = task
    this.d = new mj.MjData(m)
    this.ikData = new mj.MjData(m)
    this.seed = seed
    this.autopilot = autopilot && !!task.autopilot
    this.onEpisode = onEpisode
    this.meta = meta

    this.dt = m.opt.timestep // m.opt is a reference view into the model: read it, never delete it
    this.compiled = compiledPhysics(m) // must be the fresh model: see replay.js
    this.warningNames = warningNames(mj)
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
    // The robot is the kinematic tree rooted at the pelvis (fixed base); everything else is scene
    const pelvis = id('mjOBJ_BODY', 'pelvis')
    this.isRobotBody = b => m.body_rootid[b] === pelvis

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
    // Grip site relative to the palm site (both on the wrist body): where a held object's centre sits
    this.gripOffset = this.arms.map(a => [0, 1, 2].map(k => m.site_pos[3 * a.gripSite + k] - m.site_pos[3 * a.palmSite + k]))

    // Robot actuators come first in the model (the robot file is included before the scene); scene actuators
    // such as belt motors follow and are recorded in `action` too, after the robot's.
    this.robotNu = 0
    for (let a = 0; a < m.nu; a++) {
      if (this.isRobotBody(m.jnt_bodyid[m.actuator_trnid[2 * a]])) {
        if (a !== this.robotNu) throw new Error('robot actuators must precede scene actuators')
        this.robotNu++
      }
    }

    // Which hand (0 left, 1 right, -1 none) each body belongs to. The palm geom lives on the wrist_yaw body.
    this.bodyNames = Array.from({ length: m.nbody }, (_, b) => this.name('mjOBJ_BODY', b))
    this.handOfBody = Int8Array.from(this.bodyNames, handOfBodyName)

    // Task objects: free bodies the hands manipulate
    this.objectOfGeom = new Int16Array(m.ngeom).fill(-1)
    this.objects = task.objects.map((name, i) => {
      const body = id('mjOBJ_BODY', name)
      const jnt = id('mjOBJ_JOINT', `${name}_free`)
      const geoms = []
      for (let g = 0; g < m.ngeom; g++) {
        if (m.geom_bodyid[g] === body && (m.geom_contype[g] || m.geom_conaffinity[g])) { geoms.push(g); this.objectOfGeom[g] = i }
      }
      return { name, body, geoms, q: m.jnt_qposadr[jnt], v: m.jnt_dofadr[jnt] }
    })
    this.touching = new Uint8Array(2) // per hand: touching any task object

    this.qposNames = []
    this.qvelNames = []
    for (let j = 0; j < m.njnt; j++) {
      const n = this.name('mjOBJ_JOINT', j)
      const type = m.jnt_type[j] // mjtJoint: 0 free, 1 ball, 2 slide, 3 hinge
      if (type === 0) {
        this.qposNames.push(...['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'].map(s => `${n}:${s}`))
        this.qvelNames.push(...['vx', 'vy', 'vz', 'wx', 'wy', 'wz'].map(s => `${n}:${s}`))
      } else if (type === 1) {
        this.qposNames.push(...['qw', 'qx', 'qy', 'qz'].map(s => `${n}:${s}`))
        this.qvelNames.push(...['wx', 'wy', 'wz'].map(s => `${n}:${s}`))
      } else {
        this.qposNames.push(n)
        this.qvelNames.push(n)
      }
    }
    if (this.qposNames.length !== m.nq || this.qvelNames.length !== m.nv) throw new Error('joint naming does not cover qpos/qvel')
    this.actuatorNames = Array.from({ length: m.nu }, (_, a) => this.name('mjOBJ_ACTUATOR', a))

    this.input = new Float32Array(INPUT_SIZE)
    this.raw = new Float32Array(RAW_SIZE)
    this.autoInput = new Float32Array(INPUT_SIZE)
    // Soft parcels (XPBD, soft.js), stepped in lockstep with MuJoCo and recorded per frame
    this.soft = task.soft ? new SoftWorld(mj, m, task.soft.bodies, { handOfBody: this.handOfBody, belts: task.soft.belts ?? [] }) : null
    this.softBuf = new Float64Array(this.soft ? 3 * this.soft.total : 0)
    this.recorder = new EpisodeRecorder([
      { name: 'time', size: 1 },
      { name: 'action', size: m.nu },
      { name: 'qpos', size: m.nq },
      { name: 'qvel', size: m.nv },
      { name: 'input', size: INPUT_SIZE },
      { name: 'raw', size: RAW_SIZE },
      { name: 'touching', size: 2 },
      ...(this.soft ? [{ name: 'soft', size: 3 * this.soft.total }] : []),
    ], Math.ceil((task.timeout + 1) * CONTROL_HZ)) // preallocated: no buffer growth inside the physics loop
    this.rtf = { min: Infinity, sum: 0, n: 0 } // real-time factor samples supplied by the host while running
    this.warnings = new Int32Array(this.warningNames.length)
    this.lastQpos = new Float64Array(m.nq)
    this.lastQvel = new Float64Array(m.nv)

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

  /** Ends a running episode (default outcome 'aborted') and resets; used when the XR session ends or recenters. */
  abort(outcome = 'aborted') {
    if (this.status === 'running') this.endEpisode(outcome)
    this.pendingReset = true
  }

  /** Host-side real-time factor sample (1 = keeping up), folded into the episode header. */
  reportRealtime(rtf) {
    if (this.status !== 'running') return
    this.rtf.min = Math.min(this.rtf.min, rtf)
    this.rtf.sum += rtf
    this.rtf.n++
  }

  /** Frees the WASM-side data; the model belongs to the caller. */
  dispose() {
    this.d.delete()
    this.ikData.delete()
  }

  reset() {
    const { mj, m, d, task } = this
    if (this.status === 'running') this.endEpisode('aborted')
    this.episode++
    const rng = mulberry32(this.seed + this.episode * 9973)

    mj.mj_resetData(m, d)
    // mj_setConst (inside setPhysics) rewrites qpos to the model default, so it must run before posing anything
    this.setPhysics(task.randomize ? task.randomize(rng) : {})
    this.softPhysics = this.soft && task.randomizeSoft ? task.randomizeSoft(rng) : {}
    if (this.soft) this.soft.setPhysics(this.softPhysics)
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
    this.initialSoft = this.soft ? Array.from(this.soft.gather(this.softBuf)) : undefined
    this.initialSoftActive = this.soft ? this.soft.bodies.map(b => b.active) : undefined
    this.events = []
    this.scheduled = []
    this.frameIndex = 0
    this.peakArmVel = 0
    this.rtf = { min: Infinity, sum: 0, n: 0 }
    this.warnings.fill(0)
    this.untrackedFor = 0
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

  objectSpin(i) {
    const v = this.objects[i].v + 3
    return Math.hypot(this.d.qvel[v], this.d.qvel[v + 1], this.d.qvel[v + 2])
  }

  /**
   * MuJoCo warning counters since the episode started. Any BAD_* count means MuJoCo hit an invalid state and
   * reset the data mid-episode; the episode is then ended as 'unstable'. Read once per control tick.
   */
  readWarnings() {
    const w = this.d.warning // reference view: elements may be deleted, the vector must not be
    let changed = false
    for (let i = 0; i < this.warnings.length; i++) {
      const x = w.get(i)
      if (x.number !== this.warnings[i]) { this.warnings[i] = x.number; changed = true }
      x.delete()
    }
    return changed
  }

  /** z component of the object's own +z axis in the world: 1 upright, 0 on its side, -1 upside down. */
  objectUp(i) {
    const q = this.objects[i].q
    const x = this.d.qpos[q + 4], y = this.d.qpos[q + 5]
    return 1 - 2 * (x * x + y * y)
  }

  /** Runs fn right after frame `tick` is recorded, so any teleport it does lands on a frame boundary. */
  scheduleAtFrame(tick, fn) { this.scheduled.push({ tick, fn }) }

  // ── Soft parcels ──────────────────────────────────────────────────────────

  /** Places a soft body undeformed (reset-time; inactive = parked and not simulated). */
  placeSoft(name, pos, quat = [1, 0, 0, 0], active = false) {
    this.soft.byName[name].place(pos, quat, active)
  }

  /** Teleports a soft body mid-episode and logs it so replay can re-apply it. */
  teleportSoft(name, pos, quat = [1, 0, 0, 0], active = true) {
    this.soft.byName[name].place(pos, quat, active)
    if (this.status === 'running') this.events.push({ tick: this.frameIndex - 1, soft: name, pos: [...pos], quat: [...quat], active })
  }

  softPos(name) { return this.soft.byName[name].centroid() }
  softUp(name) { return this.soft.byName[name].up() }
  softSpeed(name) { return this.soft.byName[name].speed() }

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
    let worst = this.soft ? this.soft.measurePenetration(this.d) : 0
    for (let i = 0, n = contacts.size(); i < n; i++) {
      const c = contacts.get(i)
      if ((this.objectOfGeom[c.geom1] >= 0 || this.objectOfGeom[c.geom2] >= 0) && c.dist < 0) worst = Math.max(worst, -c.dist)
      c.delete()
    }
    contacts.delete()
    return worst
  }

  /** Applies randomized object properties { [objectName]: { mass?, friction? } }; replay uses the same function. */
  setPhysics(params) {
    applyPhysics(this.mj, this.m, params, this.compiled, this.d)
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
    if (this.soft) this.soft.step(this.d) // leaves the parcels' reaction forces in xfrc_applied for this step
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
          arm.qCmd[k] = Math.max(actual - ARM_LEAD, Math.min(actual + ARM_LEAD, arm.qCmd[k]))
        }
        for (let k = 0; k < FINGER_JOINTS.length; k++) arm.fingerCmd[k] = this.fingerTarget(arm, k, input[o + 8 + k])
      }
      // float32-exact targets so a recorded episode replays bit-for-bit from its log
      for (let k = 0; k < arm.act.length; k++) d.ctrl[arm.act[k]] = Math.fround(arm.qCmd[k])
      for (let k = 0; k < arm.fingerAct.length; k++) d.ctrl[arm.fingerAct[k]] = Math.fround(arm.fingerCmd[k])
    }

    this.scanContacts()
    if (this.soft) { this.touching[0] |= this.soft.touching[0]; this.touching[1] |= this.soft.touching[1] }
    if (this.status === 'running') {
      // A MuJoCo warning during the last steps means the state is reset/corrupt: end before recording it
      if (this.readWarnings() || (this.soft && !this.soft.finite())) { this.endEpisode('unstable'); return }
      for (const dof of this.armDof) this.peakArmVel = Math.max(this.peakArmVel, Math.abs(d.qvel[dof]))
      const frame = [this.frameIndex * this.controlDt, d.ctrl, d.qpos, d.qvel, input, this.raw, this.touching]
      if (this.soft) frame.push(this.soft.gather(this.softBuf))
      this.recorder.push(frame)
      this.lastQpos.set(d.qpos)
      this.lastQvel.set(d.qvel)
      this.frameIndex++
      this.untrackedFor = anyTracked ? 0 : this.untrackedFor + this.controlDt
      if (this.untrackedFor > COMMON.untrackedTimeout) { this.endEpisode('lost_tracking'); return }
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

  /** Which hand touches any task object, from the current contact list. */
  scanContacts() {
    const { m, d } = this
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
      if (hand >= 0) this.touching[hand] = 1
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
        if (this.objectSpeed(i) > COMMON.restSpeed || this.objectSpin(i) > COMMON.restSpin) settled = false
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
    const { m, task } = this
    this.status = outcome
    this.endTime = this.d.time
    // from recorded frames, not d.time: MuJoCo's auto-reset on an unstable state sends d.time back to 0
    const duration = Math.max(0, this.recorder.frames - 1) * this.controlDt
    if (!this.onEpisode || this.recorder.frames === 0) return false
    if (duration < COMMON.minEpisode && outcome !== 'unstable') return false
    const frames = this.recorder.view() // a view: onEpisode must copy (encode) it before the next episode
    const flags = []
    if (this.peakArmVel > COMMON.fastMotion) flags.push('fast_motion')
    if (outcome === 'unstable') flags.push('unstable')
    if (this.rtf.n && this.rtf.min < 0.9) flags.push('slow_physics')
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
        mujoco_warnings: Object.fromEntries(this.warningNames.map((n, i) => [n, this.warnings[i]])),
        flags,
        initial_qpos: this.initialQpos,
        initial_ctrl: this.initialCtrl,
        soft_bodies: this.soft ? this.soft.header() : undefined,
        soft_belts: this.soft ? (task.soft.belts ?? []) : undefined,
        soft_physics: this.soft ? this.softPhysics : undefined,
        initial_soft: this.initialSoft,
        initial_soft_active: this.initialSoftActive,
        // state at the last recorded control tick (teleports done by the task in that tick are not included)
        final_qpos: Array.from(this.lastQpos),
        final_qvel: Array.from(this.lastQvel),
        events: this.events,
        result: task.result ? task.result(this) : undefined,
        nq: m.nq, nv: m.nv, nu: m.nu,
        robot_nu: this.robotNu,
        body_names: this.bodyNames,
        qpos_names: this.qposNames,
        qvel_names: this.qvelNames,
        actuator_names: this.actuatorNames,
        input_names: INPUT_NAMES,
        raw_layout: RAW_LAYOUT,
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
      writeHandInput(inp, s * HAND_INPUT, this.gripOffset[s], g)
    }
    return inp
  }
}
