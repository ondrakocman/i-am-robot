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
let solvedAt = 0
while (episodes.length < runs && steps < runs * (task.timeout + 5) / sim.dt) {
  if (!task.autopilot && sim.status === 'waiting') {
    // start the episode with a "tracked" idle hand; 2 s in, teleport the objects into the goal (on a frame
    // boundary, so the logged event replays exactly)
    sim.input.fill(0); sim.input[0] = 1; sim.input.set([...sim.readyGrip[0], 1, 0, 0, 0], 1)
    if (solvedAt !== sim.episode) { sim.scheduleAtFrame(Math.round(2 / sim.controlDt), () => task.solved(sim)); solvedAt = sim.episode }
  }
  sim.step()
  steps++
}
const ms = (performance.now() - t0) / steps
console.log(`${steps} steps, ${ms.toFixed(3)} ms/step (${(ms / (sim.dt * 1000) * 100).toFixed(0)}% of real time on one core)`)
for (const e of episodes) {
  const h = e.header
  const phys = Object.entries(h.physics).map(([k, v]) => `${k} ${v.mass.toFixed(2)}kg/mu${v.friction.toFixed(2)}`).join(' ')
  console.log(`episode ${h.episode}: ${h.outcome} after ${h.duration.toFixed(2)} s, ${h.frames} frames, peak arm ${h.peak_arm_velocity.toFixed(1)} rad/s${h.flags.length ? ' FLAGS ' + h.flags : ''}${h.events.length ? `, ${h.events.length} events` : ''}${h.result ? ' ' + JSON.stringify(h.result) : ''}\n    ${phys}`)
}

// Replay check: initial qpos + logged float32 actions must reproduce the logged qpos exactly
const e = episodes[0]
if (e) {
  const { header, frames } = e
  const f = Object.fromEntries(header.fields.map(x => [x.name, x]))
  const d = new mj.MjData(m)
  sim.setPhysics(header.physics)
  d.qpos.set(header.initial_qpos)
  d.ctrl.set(header.initial_ctrl)
  const q32 = new Float32Array(header.nq)
  let mismatch = -1
  const replayFrames = header.frames
  let ev = 0
  for (let i = 0; i < replayFrames && mismatch < 0; i++) {
    const o = i * header.frame_size
    q32.set(d.qpos) // each frame logs the state at its control tick, before that tick's steps
    for (let k = 0; k < header.nq; k++) if (q32[k] !== frames[o + f.qpos.offset + k]) { mismatch = i; break }
    // logged teleports (spawns) come after the frame they are tagged with, before the steps to the next one
    for (; ev < header.events.length && header.events[ev].tick === i; ev++) {
      const e = header.events[ev]
      d.qpos.set(e.qpos, e.qpos_adr)
      d.qvel.fill(0, e.qvel_adr, e.qvel_adr + 6)
    }
    for (let a = 0; a < header.nu; a++) d.ctrl[a] = frames[o + f.action.offset + a]
    for (let s = 0; s < header.steps_per_control; s++) mj.mj_step(m, d)
  }
  console.log(mismatch < 0 ? `replay: all ${replayFrames} frames match bit-for-bit` : `replay: diverged at frame ${mismatch}`)
  d.delete()
}
