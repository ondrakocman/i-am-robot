// Reference replay of a recorded episode: initial state + initial ctrl + logged actions + logged teleport events
// must reproduce the logged qpos/qvel bit-for-bit with the same MuJoCo build. Used by scripts/sim-check.mjs and
// scripts/replay.mjs; also the specification of how a downstream renderer should step an episode.

/**
 * Snapshot of the compiled mass/inertia, taken on a freshly loaded model. The recorder scales the compiled
 * inertia by mass/compiled_mass; replay must do the same arithmetic from the same values to be bit-identical,
 * so it scales from this snapshot rather than from whatever a previous episode left in the model.
 */
export function compiledPhysics(m) {
  return { mass: Float64Array.from(m.body_mass), inertia: Float64Array.from(m.body_inertia) }
}

/** Applies the header's randomized physics (mass with scaled inertia, sliding friction) to a model. */
export function applyPhysics(mj, m, header, base) {
  for (const [name, p] of Object.entries(header.physics ?? {})) {
    const body = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY.value, name)
    if (body < 0) throw new Error(`replay: model has no body ${name}`)
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
  const d = new mj.MjData(m)
  mj.mj_setConst(m, d)
  d.delete()
}

/**
 * Steps through an episode, comparing each logged frame against the simulated state.
 * Returns { frames, mismatch } where mismatch is the first diverging frame index (-1 if none), plus the
 * largest absolute qpos/qvel difference seen before any mismatch.
 */
export function replayEpisode(mj, m, header, frames, { onFrame = null } = {}) {
  const field = Object.fromEntries(header.fields.map(f => [f.name, f]))
  const d = new mj.MjData(m)
  d.qpos.set(header.initial_qpos)
  d.ctrl.set(header.initial_ctrl ?? [])
  const q32 = new Float32Array(header.nq)
  const v32 = new Float32Array(header.nv)
  let mismatch = -1
  let ev = 0
  try {
    for (let i = 0; i < header.frames; i++) {
      const o = i * header.frame_size
      // Each frame logs the state at its control tick, before that tick's physics steps
      q32.set(d.qpos)
      v32.set(d.qvel)
      for (let k = 0; k < header.nq; k++) if (q32[k] !== frames[o + field.qpos.offset + k]) { mismatch = i; break }
      if (mismatch < 0) for (let k = 0; k < header.nv; k++) if (v32[k] !== frames[o + field.qvel.offset + k]) { mismatch = i; break }
      if (mismatch >= 0) break
      // d.xpos/xquat still describe the previous substep: run forward kinematics on this frame's qpos first
      if (onFrame) { mj.mj_kinematics(m, d); onFrame(i, d) }
      // Logged teleports (spawns) come after the frame they are tagged with, before the steps to the next one
      for (; ev < header.events.length && header.events[ev].tick === i; ev++) {
        const e = header.events[ev]
        d.qpos.set(e.qpos, e.qpos_adr)
        d.qvel.fill(0, e.qvel_adr, e.qvel_adr + 6)
      }
      for (let a = 0; a < header.nu; a++) d.ctrl[a] = frames[o + field.action.offset + a]
      for (let s = 0; s < header.steps_per_control; s++) mj.mj_step(m, d)
    }
  } finally {
    d.delete()
  }
  return { frames: header.frames, mismatch }
}
