// Headless check of a task: runs its scripted autopilot through TaskSim in Node and reports whether the
// episodes succeed, plus step cost. Also verifies that a recorded episode replays bit-for-bit from its log.
//   node scripts/sim-check.mjs [task=tube_box] [dt=0.002] [n=3]
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { loadScene } from '../src/sim/loadScene.js'
import { TaskSim } from '../src/sim/TaskSim.js'
import { getTask } from '../src/sim/tasks/index.js'

const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
const args = Object.fromEntries(process.argv.slice(2).map(a => a.includes('=') ? a.split('=') : ['task', a]))
const task = getTask(args.task)
const timestep = Number(args.dt ?? 0.002)
const runs = Number(args.n ?? 3)

const mj = await loadMujoco()
const readFile = async p => (p.endsWith('.xml') ? fs.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.readFile(path.join(PUBLIC, p))))
const m = await loadScene(mj, readFile, { scene: task.scene, timestep })

const episodes = []
const sim = new TaskSim(mj, m, task, { seed: 7, autopilot: true, onEpisode: e => episodes.push(e) })
console.log(`task ${task.name}  timestep ${sim.dt}  control ${1 / sim.controlDt} Hz  nq=${m.nq} nv=${m.nv} nu=${m.nu} nbody=${m.nbody}`)
if (!task.autopilot) console.log('no scripted demonstration for this task: checking goal detection from the solved configuration')

const t0 = performance.now()
let steps = 0
let solvedAt = -1
while (episodes.length < runs && steps < runs * (task.timeout + 5) / sim.dt) {
  if (!task.autopilot) {
    // start the episode with a "tracked" idle hand, then after 2 s teleport the objects into the goal
    if (sim.status === 'waiting') { sim.input.fill(0); sim.input[0] = 1; sim.input.set([...sim.readyGrip[0], 1, 0, 0, 0], 1); solvedAt = -1 }
    if (sim.status === 'running' && solvedAt < 0) solvedAt = sim.steps
    if (sim.status === 'running' && sim.steps === solvedAt + Math.round(2 / sim.dt)) { task.solved(sim); mj.mj_forward(m, sim.d) }
  }
  sim.step()
  steps++
}
const ms = (performance.now() - t0) / steps
console.log(`${steps} steps, ${ms.toFixed(3)} ms/step (${(ms / (sim.dt * 1000) * 100).toFixed(0)}% of real time on one core)`)
for (const e of episodes) {
  const h = e.header
  const phys = Object.entries(h.physics).map(([k, v]) => `${k} ${v.mass.toFixed(2)}kg/mu${v.friction.toFixed(2)}`).join(' ')
  console.log(`episode ${h.episode}: ${h.outcome} after ${h.duration.toFixed(2)} s, ${h.frames} frames, peak arm ${h.peak_arm_velocity.toFixed(1)} rad/s${h.flags.length ? ' FLAGS ' + h.flags : ''}\n    ${phys}`)
}

// Replay check: initial qpos + logged float32 actions must reproduce the logged qpos exactly
const e = episodes[0]
if (e) {
  const { header, frames } = e
  const f = Object.fromEntries(header.fields.map(x => [x.name, x]))
  const d = new mj.MjData(m)
  sim.setPhysics(header.physics)
  d.qpos.set(header.initial_qpos)
  const q32 = new Float32Array(header.nq)
  let mismatch = -1
  // In solved mode the objects are teleported at 2 s, which no action can reproduce: replay only up to that
  const replayFrames = task.autopilot ? header.frames : Math.min(header.frames, Math.round(2 * header.control_hz))
  for (let i = 0; i < replayFrames && mismatch < 0; i++) {
    const o = i * header.frame_size
    q32.set(d.qpos) // each frame logs the state at its control tick, before that tick's steps
    for (let k = 0; k < header.nq; k++) if (q32[k] !== frames[o + f.qpos.offset + k]) { mismatch = i; break }
    for (let a = 0; a < header.nu; a++) d.ctrl[a] = frames[o + f.action.offset + a]
    for (let s = 0; s < header.steps_per_control; s++) mj.mj_step(m, d)
  }
  console.log(mismatch < 0 ? `replay: all ${replayFrames} frames match bit-for-bit` : `replay: diverged at frame ${mismatch}`)
  d.delete()
}
