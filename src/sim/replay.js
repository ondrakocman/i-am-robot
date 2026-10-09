// Reference replay of a recorded episode: initial state + initial ctrl + logged actions + logged teleport events
// must reproduce the logged qpos/qvel bit-for-bit with the same MuJoCo build. Used by TaskSim (physics
// randomization), scripts/sim-check.mjs and scripts/replay.mjs; also the specification of how a downstream
// renderer should step an episode.
import { SoftWorld } from './soft.js'

/**
 * Snapshot of the compiled mass/inertia/friction, taken on a freshly loaded model. Randomized physics scales
 * the compiled inertia by mass/compiled_mass; the recorder and replay must do that arithmetic from the same
 * values to be bit-identical, so both scale from this snapshot rather than from whatever a previous episode
 * left in the model.
 */
export function compiledPhysics(m) {
  return { mass: Float64Array.from(m.body_mass), inertia: Float64Array.from(m.body_inertia), friction: Float64Array.from(m.geom_friction) }
}

/**
 * Applies randomized object physics { [bodyName]: { mass?, friction? } } to the model, restoring every body
 * and geom to its compiled values first so nothing leaks between episodes. `d` is the data to recompute
 * constants into (a scratch MjData is used if omitted).
 */
export function applyPhysics(mj, m, params, base, d = null) {
  m.body_mass.set(base.mass)
  m.body_inertia.set(base.inertia)
  m.geom_friction.set(base.friction)
  for (const [name, p] of Object.entries(params ?? {})) {
    const body = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY.value, name)
    if (body < 0) throw new Error(`physics: model has no body ${name}`)
    if (p.mass !== undefined) {
      const scale = p.mass / base.mass[body]
      m.body_mass[body] = p.mass
      for (let k = 0; k < 3; k++) m.body_inertia[3 * body + k] = base.inertia[3 * body + k] * scale
    }
    if (p.friction !== undefined) {
      for (let g = 0; g < m.ngeom; g++) {
        if (m.geom_bodyid[g] === body && (m.geom_contype[g] || m.geom_conaffinity[g])) m.geom_friction[3 * g] = p.friction
      }
    }
  }
  const scratch = d ?? new mj.MjData(m)
  mj.mj_setConst(m, scratch)
  if (!d) scratch.delete()
}

/**
 * Steps through an episode, yielding { i, d } for every frame whose logged qpos/qvel the simulation
 * reproduced exactly (forward kinematics already run on d), and finally returning the index of the first
 * diverging frame (-1 if none). The MjData is deleted when the generator finishes or is closed.
 */
export function* replayFrames(mj, m, header, frames) {
  const field = Object.fromEntries(header.fields.map(f => [f.name, f]))
  const d = new mj.MjData(m)
  d.qpos.set(header.initial_qpos)
  d.ctrl.set(header.initial_ctrl ?? [])
  // soft parcels: rebuilt from the header and stepped in lockstep, their logged positions compared too
  const soft = header.soft_bodies?.length ? new SoftWorld(mj, m, Object.fromEntries(header.soft_bodies.map(b => [b.name, { half: b.half, cells: b.cells }])), { belts: header.soft_belts ?? [] }) : null
  if (soft) {
    soft.setPhysics(header.soft_physics ?? {})
    soft.scatter(Float64Array.from(header.initial_soft))
    soft.bodies.forEach((b, k) => { b.active = !!header.initial_soft_active?.[k] })
  }
  const softBuf = soft ? new Float64Array(3 * soft.total) : null
  const q32 = new Float32Array(header.nq)
  const v32 = new Float32Array(header.nv)
  let ev = 0
  try {
    for (let i = 0; i < header.frames; i++) {
      const o = i * header.frame_size
      // Each frame logs the state at its control tick, before that tick's physics steps
      q32.set(d.qpos)
      v32.set(d.qvel)
      for (let k = 0; k < header.nq; k++) if (q32[k] !== frames[o + field.qpos.offset + k]) return i
      for (let k = 0; k < header.nv; k++) if (v32[k] !== frames[o + field.qvel.offset + k]) return i
      if (soft) {
        soft.gather(softBuf)
        for (let k = 0; k < softBuf.length; k++) if (Math.fround(softBuf[k]) !== frames[o + field.soft.offset + k]) return i
      }
      mj.mj_kinematics(m, d) // d.xpos/xquat still described the previous substep
      yield { i, d, soft }
      // Logged teleports (spawns) come after the frame they are tagged with, before the steps to the next one
      for (; ev < header.events.length && header.events[ev].tick === i; ev++) {
        const e = header.events[ev]
        if (e.soft) { soft.byName[e.soft].place(e.pos, e.quat, e.active); continue }
        d.qpos.set(e.qpos, e.qpos_adr)
        d.qvel.fill(0, e.qvel_adr, e.qvel_adr + 6)
      }
      for (let a = 0; a < header.nu; a++) d.ctrl[a] = frames[o + field.action.offset + a]
      for (let s = 0; s < header.steps_per_control; s++) {
        if (soft) soft.step(d)
        mj.mj_step(m, d)
      }
    }
    return -1
  } finally {
    d.delete()
  }
}

/** Replays a whole episode; returns { frames, mismatch } with mismatch = first diverging frame or -1. */
export function replayEpisode(mj, m, header, frames) {
  const gen = replayFrames(mj, m, header, frames)
  let r = gen.next()
  while (!r.done) r = gen.next()
  return { frames: header.frames, mismatch: r.value }
}
