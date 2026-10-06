// Headless check of the physics task: runs the scripted autopilot through TubeBoxSim in Node and reports
// whether the tube ends up in the box, plus step cost. Also verifies that a recorded episode replays
// bit-for-bit from its log.
//   node scripts/sim-check.mjs [timestep=0.002] [episodes=3]
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { loadScene } from '../src/sim/loadScene.js'
import { TubeBoxSim } from '../src/sim/TubeBoxSim.js'

const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
const timestep = Number(process.argv[2] ?? 0.002)
const runs = Number(process.argv[3] ?? 3)

const mj = await loadMujoco()
const readFile = async p => (p.endsWith('.xml') ? fs.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.readFile(path.join(PUBLIC, p))))
const m = await loadScene(mj, readFile, { timestep })

const episodes = []
const sim = new TubeBoxSim(mj, m, { seed: 7, autopilot: true, onEpisode: e => episodes.push(e) })
console.log(`timestep ${sim.dt}  control ${1 / sim.controlDt} Hz  nq=${m.nq} nv=${m.nv} nu=${m.nu} nbody=${m.nbody}`)

const t0 = performance.now()
let steps = 0
while (episodes.length < runs && steps < runs * 20 / sim.dt) {
  sim.step()
  steps++
}
const ms = (performance.now() - t0) / steps
console.log(`${steps} steps, ${ms.toFixed(3)} ms/step (${(ms / (sim.dt * 1000) * 100).toFixed(0)}% of real time on one core)`)
for (const e of episodes) {
  const h = e.header
  console.log(`episode ${h.episode}: ${h.outcome} after ${h.duration.toFixed(2)} s, ${h.frames} frames, tube start ${h.layout.tube.map(v => v.toFixed(3))}`)
}

// Replay check: initial qpos + logged float32 actions must reproduce the logged qpos exactly
const e = episodes[0]
if (e) {
  const { header, frames } = e
  const f = Object.fromEntries(header.fields.map(x => [x.name, x]))
  const d = new mj.MjData(m)
  d.qpos.set(header.initial_qpos)
  const q32 = new Float32Array(header.nq)
  let mismatch = -1
  for (let i = 0; i < header.frames && mismatch < 0; i++) {
    const o = i * header.frame_size
    q32.set(d.qpos) // each frame logs the state at its control tick, before that tick's steps
    for (let k = 0; k < header.nq; k++) if (q32[k] !== frames[o + f.qpos.offset + k]) { mismatch = i; break }
    for (let a = 0; a < header.nu; a++) d.ctrl[a] = frames[o + f.action.offset + a]
    for (let s = 0; s < header.steps_per_control; s++) mj.mj_step(m, d)
  }
  console.log(mismatch < 0 ? `replay: all ${header.frames} frames match bit-for-bit` : `replay: diverged at frame ${mismatch}`)
  d.delete()
}
