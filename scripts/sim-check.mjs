// Headless check of a task, used as the CI gate (non-zero exit on any failure):
//   1. resets over several seeds start from physically valid states (no object penetration)
//   2. episodes succeed: tasks with a scripted demonstration run it; others teleport the objects into the
//      solved configuration (on a frame boundary, so it is a logged event) and goal detection must fire
//   3. every episode replays bit-for-bit (qpos and qvel) from its header + actions + events
//   4. MuJoCo raised no warnings (instability, constraint overflow)
//   node scripts/sim-check.mjs [task=tube_box] [dt=0.002] [n=3] [seeds=25] [out=episodes.iamr]
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { loadScene } from '../src/sim/loadScene.js'
import { TaskSim } from '../src/sim/TaskSim.js'
import { getTask } from '../src/sim/tasks/index.js'
import { applyPhysics, compiledPhysics, replayEpisode } from '../src/sim/replay.js'
import { encodeEpisode } from '../src/sim/episode.js'

const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
const MAX_PENETRATION = 0.002 // m: contact softness allows ~1 mm at rest; anything deeper is a bad reset
const SOLVE_AT_S = 2

const args = Object.fromEntries(process.argv.slice(2).map(a => a.includes('=') ? a.split('=') : ['task', a]))
const task = getTask(args.task)
const timestep = Number(args.dt ?? 0.002)
const runs = Number(args.n ?? 3)
const seeds = Number(args.seeds ?? 25)
let failures = 0
const fail = msg => { failures++; console.error('FAIL:', msg) }

const mj = await loadMujoco()
const readFile = async p => (p.endsWith('.xml') ? fs.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.readFile(path.join(PUBLIC, p))))
const { model: m, assets } = await loadScene(mj, readFile, { scene: task.scene, timestep })
const MUJOCO_VERSION = JSON.parse(await fs.readFile(new URL('../node_modules/@mujoco/mujoco/package.json', import.meta.url))).version
const base = compiledPhysics(m) // before anything randomizes the model, like the headset's fresh load
// d.warning is a reference view into MjData: its elements may be deleted, the vector itself must not be
const warningCount = d => { const w = d.warning; let n = 0; for (let i = 0; i < w.size(); i++) { const x = w.get(i); n += x.number; x.delete() } return n }
console.log(`task ${task.name}  timestep ${timestep}  nq=${m.nq} nv=${m.nv} nu=${m.nu} nbody=${m.nbody}`)

// 1. episodes (the recording sim is built on the fresh model, as on the headset)
const episodes = []
const sim = new TaskSim(mj, m, task, {
  seed: 7, autopilot: true, onEpisode: e => episodes.push(e),
  meta: { session: 'sim-check', app_version: 'sim-check', mujoco: MUJOCO_VERSION, assets }, // same header fields as the headset
})
if (!task.autopilot) console.log('no scripted demonstration for this task: checking goal detection from the solved configuration')
const t0 = performance.now()
let steps = 0
let solvedFor = 0
while (episodes.length < runs && steps < runs * (task.timeout + 5) / sim.dt) {
  if (!task.autopilot && sim.status === 'waiting') {
    // a tracked left hand (held at its ready grip point) starts the episode; SOLVE_AT_S in, the objects are
    // teleported into the goal
    sim.input.fill(0); sim.input[0] = 1; sim.input.set([...sim.readyGrip[0], 1, 0, 0, 0], 1)
    if (solvedFor !== sim.episode) { sim.scheduleAtFrame(Math.round(SOLVE_AT_S / sim.controlDt), () => task.solved(sim)); solvedFor = sim.episode }
  }
  sim.step()
  steps++
}
const ms = (performance.now() - t0) / steps
console.log(`${steps} steps, ${ms.toFixed(3)} ms/step (${(ms / (sim.dt * 1000) * 100).toFixed(0)}% of real time on one core)`)
if (warningCount(sim.d)) fail('MuJoCo raised warnings after the last episode')
if (episodes.length < runs) fail(`only ${episodes.length}/${runs} episodes finished`)
for (const e of episodes) {
  const h = e.header
  const warned = Object.entries(h.mujoco_warnings).filter(([, n]) => n).map(([k, n]) => `${k}=${n}`)
  if (warned.length) fail(`episode ${h.episode} raised MuJoCo warnings ${warned.join(' ')}`)
  const phys = Object.entries(h.physics).map(([k, v]) => `${k} ${v.mass.toFixed(2)}kg/mu${v.friction.toFixed(2)}`).join(' ')
  console.log(`episode ${h.episode}: ${h.outcome} after ${h.duration.toFixed(2)} s, ${h.frames} frames, peak arm ${h.peak_arm_velocity.toFixed(1)} rad/s${h.flags.length ? ' FLAGS ' + h.flags : ''}${h.events.length ? `, ${h.events.length} events` : ''}${h.result ? ' ' + JSON.stringify(h.result) : ''}\n    ${phys}`)
  if (h.outcome !== 'success') fail(`episode ${h.episode} ended with ${h.outcome}`)
}

// 2. replay (fresh data, physics restored from the header, like a downstream consumer would)
for (const { header, frames } of episodes) {
  applyPhysics(mj, m, header, base)
  const r = replayEpisode(mj, m, header, frames)
  if (r.mismatch >= 0) fail(`episode ${header.episode} replay diverged at frame ${r.mismatch}`)
  else console.log(`episode ${header.episode} replay: all ${r.frames} frames match bit-for-bit`)
}

// 3. an unstable state must end the episode as 'unstable', flagged, keeping the frames recorded before it
{
  const got = []
  const probe = new TaskSim(mj, m, task, { seed: 11, autopilot: true, onEpisode: e => got.push(e) })
  probe.input.fill(0); probe.input[0] = 1; probe.input.set([...probe.readyGrip[0], 1, 0, 0, 0], 1)
  probe.scheduleAtFrame(50, () => { probe.d.qvel[probe.objects[0].v] = NaN })
  while (!got.length && probe.steps < 10 / probe.dt) probe.step()
  const h = got[0]?.header
  if (!h) fail('NaN injection did not produce an episode')
  else if (h.outcome !== 'unstable' || !h.flags.includes('unstable') || h.frames !== 51 || !h.mujoco_warnings.bad_qvel) {
    fail(`NaN injection gave outcome ${h?.outcome}, flags ${h?.flags}, ${h?.frames} frames, warnings ${JSON.stringify(h?.mujoco_warnings)}`)
  } else console.log(`instability check: 'unstable' episode with ${h.frames} clean frames`)
  probe.dispose()
}

// 4. every goal region must be reachable by the hand that serves it (IK on the kinematic model)
if (task.reachTargets) {
  const probe = new TaskSim(mj, m, task, { seed: 1 })
  for (const { side, point, tolerance = 0.02 } of task.reachTargets(probe)) {
    const arm = probe.arms[side]
    const ik = probe.ikData
    mj.mj_resetData(m, ik)
    arm.ik.qadr.forEach((a, k) => { ik.qpos[a] = arm.ready[k] })
    const palm = [point[0] - probe.gripOffset[side][0], point[1] - probe.gripOffset[side][1], point[2] - probe.gripOffset[side][2]]
    for (let i = 0; i < 80; i++) arm.ik.solve(mj, ik, palm, [1, 0, 0, 0], { iterations: 1 })
    mj.mj_kinematics(m, ik)
    const s = 3 * arm.gripSite
    const err = Math.hypot(ik.site_xpos[s] - point[0], ik.site_xpos[s + 1] - point[1], ik.site_xpos[s + 2] - point[2])
    if (err > tolerance) fail(`${arm.side} hand cannot reach goal point ${point.map(v => v.toFixed(2))} (${(err * 100).toFixed(1)} cm off)`)
  }
  if (!failures) console.log(`reach check: ${task.reachTargets(probe).length} goal points within tolerance`)
  probe.dispose()
}

// 5. reset validity over many seeds (each TaskSim re-randomizes the shared model; fine after the replays)
let worst = 0
for (let seed = 1; seed <= seeds; seed++) {
  const probe = new TaskSim(mj, m, task, { seed })
  worst = Math.max(worst, probe.maxObjectPenetration())
  probe.dispose()
}
console.log(`reset check: deepest object penetration over ${seeds} seeds ${(worst * 1000).toFixed(2)} mm`)
if (worst > MAX_PENETRATION) fail(`resets start interpenetrating (${(worst * 1000).toFixed(1)} mm > ${MAX_PENETRATION * 1000} mm)`)

if (args.out) {
  await fs.writeFile(args.out, Buffer.concat(episodes.map(e => Buffer.from(encodeEpisode(e.header, e.frames)))))
  console.log(`wrote ${episodes.length} episodes to ${args.out}`)
}
sim.dispose(); m.delete()
if (failures) { console.error(`${failures} check(s) failed`); process.exit(1) }
console.log('all checks passed')
