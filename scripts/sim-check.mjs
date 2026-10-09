// Headless check of the tasks, used as the CI gate (non-zero exit on any failure). For every task:
//   1. episodes succeed: tasks with a scripted demonstration run it; others teleport the objects into the
//      solved configuration (on a frame boundary, so it is a logged event) and goal detection must fire
//   2. every episode replays bit-for-bit (qpos and qvel) from its header + actions + events, and MuJoCo
//      raised no warnings
//   3. an injected instability ends the episode as 'unstable', keeping the frames recorded before it
//   4. every goal region the task declares is reachable by the hand that serves it (2 cm: the IK's deliberate
//      posture regularizer settles up to ~2 cm short of a target, see ik.js)
//   5. resets over several seeds start from physically valid states (no object penetration), and so does every
//      teleport a task performs during the checked episodes and every pose its spawnPoses() can draw
//   6. every geom the renderer draws has a material (collision-only geoms belong in group 3; an unmaterialed
//      box drawn in default grey over a visual mesh is the classic mistake)
//   7. a task's failureCase() ends the episode with the outcome it promises (the scoring's failure paths run)
//   node scripts/sim-check.mjs [task=<name>|all] [dt=0.002] [n=3] [seeds=25] [out=episodes.iamr]
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs/promises'
import { loadScene } from '../src/sim/loadScene.js'
import { TaskSim, HAND_INPUT } from '../src/sim/TaskSim.js'
import { TASKS, getTask } from '../src/sim/tasks/index.js'
import { applyPhysics, compiledPhysics, replayEpisode } from '../src/sim/replay.js'
import { encodeEpisode } from '../src/sim/episode.js'
import { mat2quat } from '../src/sim/ik.js'
import { MUJOCO_VERSION, readPublic as readFile } from './lib.mjs'

const MAX_PENETRATION = 0.002 // m: contact softness allows ~1 mm at rest; anything deeper is a bad reset
const SOLVE_AT_S = 2

const args = Object.fromEntries(process.argv.slice(2).map(a => a.includes('=') ? a.split('=') : ['task', a]))
const tasks = !args.task || args.task === 'all' ? Object.values(TASKS) : [getTask(args.task)]
const timestep = Number(args.dt ?? 0.002)
const runs = Number(args.n ?? 3)
const seeds = Number(args.seeds ?? 25)
let failures = 0
const fail = msg => { failures++; console.error('FAIL:', msg) }

const mj = await loadMujoco()
// d.warning is a reference view into MjData: its elements may be deleted, the vector itself must not be
const warningCount = d => { const w = d.warning; let n = 0; for (let i = 0; i < w.size(); i++) { const x = w.get(i); n += x.number; x.delete() } return n }

// A "tracked" hand that asks for exactly the palm pose the ready posture already has, so the arm stays put
function holdReadyPose(sim, side = 0) {
  const arm = sim.arms[side]
  const q = mat2quat(sim.d.site_xmat, 9 * arm.palmSite, new Float64Array(4))
  sim.input.fill(0)
  sim.input[side * HAND_INPUT] = 1
  sim.input.set(sim.d.site_xpos.slice(3 * arm.palmSite, 3 * arm.palmSite + 3), side * HAND_INPUT + 1)
  sim.input.set(q, side * HAND_INPUT + 4)
}

const recorded = []
for (const task of tasks) {
  const { model: m, assets } = await loadScene(mj, readFile, { scene: task.scene, timestep })
  const base = compiledPhysics(m) // before anything randomizes the model, like the headset's fresh load
  console.log(`\ntask ${task.name}  timestep ${timestep}  nq=${m.nq} nv=${m.nv} nu=${m.nu} nbody=${m.nbody}`)

  // 1. episodes (the recording sim is built on the fresh model, as on the headset)
  const episodes = []
  const sim = new TaskSim(mj, m, task, {
    seed: 7, autopilot: true, onEpisode: ({ header, frames }) => episodes.push({ header, frames: frames.slice() }),
    meta: { session: 'sim-check', app_version: 'sim-check', mujoco: MUJOCO_VERSION, assets }, // same header fields as the headset
  })
  if (!task.autopilot) console.log('no scripted demonstration for this task: checking goal detection from the solved configuration')
  // every teleport a task performs mid-episode (spawns, solved placement) must land in a valid state
  let worstTeleport = 0
  const teleport = sim.teleportObject.bind(sim)
  sim.teleportObject = (...args) => { teleport(...args); mj.mj_forward(m, sim.d); worstTeleport = Math.max(worstTeleport, sim.maxObjectPenetration()) }
  const t0 = performance.now()
  let steps = 0
  let solvedFor = 0
  while (episodes.length < runs && steps < runs * (task.timeout + 5) / sim.dt) {
    if (!task.autopilot && sim.status === 'waiting') {
      // a tracked hand holding its ready pose starts the episode; SOLVE_AT_S in, the objects are teleported into
      // the goal
      holdReadyPose(sim)
      if (solvedFor !== sim.episode) { sim.scheduleAtFrame(Math.round(SOLVE_AT_S / sim.controlDt), () => task.solved(sim)); solvedFor = sim.episode }
    }
    sim.step()
    steps++
  }
  const ms = (performance.now() - t0) / steps
  console.log(`${steps} steps, ${ms.toFixed(3)} ms/step (${(ms / (sim.dt * 1000) * 100).toFixed(0)}% of real time on one core)`)
  if (warningCount(sim.d)) fail('MuJoCo raised warnings after the last episode')
  if (episodes.length < runs) fail(`only ${episodes.length}/${runs} episodes finished`)
  if (worstTeleport > MAX_PENETRATION) fail(`a teleport during the episodes landed ${(worstTeleport * 1000).toFixed(1)} mm inside something`)
  else console.log(`teleports during the episodes: deepest penetration ${(worstTeleport * 1000).toFixed(2)} mm`)
  for (const e of episodes) {
    const h = e.header
    const warned = Object.entries(h.mujoco_warnings).filter(([, n]) => n).map(([k, n]) => `${k}=${n}`)
    if (warned.length) fail(`episode ${h.episode} raised MuJoCo warnings ${warned.join(' ')}`)
    const phys = Object.entries(h.physics).map(([k, v]) => `${k} ${v.mass.toFixed(2)}kg/mu${v.friction.toFixed(2)}`).join(' ')
    console.log(`episode ${h.episode}: ${h.outcome} after ${h.duration.toFixed(2)} s, ${h.frames} frames, peak arm ${h.peak_arm_velocity.toFixed(1)} rad/s${h.flags.length ? ' FLAGS ' + h.flags : ''}${h.events.length ? `, ${h.events.length} events` : ''}${h.result ? ' ' + JSON.stringify(h.result) : ''}\n    ${phys}`)
    if (h.outcome !== 'success') fail(`episode ${h.episode} ended with ${h.outcome}`)
  }
  sim.dispose()

  // 2. replay (fresh data, physics restored from the header, like a downstream consumer would)
  for (const { header, frames } of episodes) {
    applyPhysics(mj, m, header.physics, base)
    const r = replayEpisode(mj, m, header, frames)
    if (r.mismatch >= 0) fail(`episode ${header.episode} replay diverged at frame ${r.mismatch}`)
    else console.log(`episode ${header.episode} replay: all ${r.frames} frames match bit-for-bit`)
  }
  recorded.push(...episodes)

  // 3. an unstable state must end the episode as 'unstable', flagged, keeping the frames recorded before it
  {
    const got = []
    const probe = new TaskSim(mj, m, task, { seed: 11, autopilot: true, onEpisode: e => got.push(e) })
    holdReadyPose(probe)
    probe.scheduleAtFrame(50, () => { probe.d.qvel[0] = NaN }) // a robot DOF: a task spawn on the same frame would overwrite an object's
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
    let unreachable = 0
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
      if (err > tolerance) { unreachable++; fail(`${arm.side} hand cannot reach goal point ${point.map(v => v.toFixed(2))} (${(err * 100).toFixed(1)} cm off)`) }
    }
    if (!unreachable) console.log(`reach check: ${task.reachTargets(probe).length} goal points within tolerance`)
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

  // 5b. every pose the task's spawner can draw must be valid too
  if (task.spawnPoses) {
    const probe = new TaskSim(mj, m, task, { seed: 3 })
    let worstSpawn = 0, bad = null
    for (const { body, pos, quat } of task.spawnPoses(probe)) {
      probe.teleportObject(body, pos, quat)
      mj.mj_forward(m, probe.d)
      const pen = probe.maxObjectPenetration()
      if (pen > worstSpawn) { worstSpawn = pen; bad = { body, pos, quat } }
      probe.placeObject(body, [-3 - 0.3 * body, 0, 0.05])
    }
    if (worstSpawn > MAX_PENETRATION) fail(`spawn pose ${JSON.stringify(bad)} starts ${(worstSpawn * 1000).toFixed(1)} mm inside something`)
    else console.log(`spawn check: ${task.spawnPoses(probe).length} poses, deepest penetration ${(worstSpawn * 1000).toFixed(2)} mm`)
    probe.dispose()
  }

  // 7. the scoring's failure path: the task sets up a state that must end as the outcome it promises
  if (task.failureCase) {
    const got = []
    const probe = new TaskSim(mj, m, task, { seed: 5, onEpisode: e => got.push(e) })
    let expect = null
    holdReadyPose(probe)
    probe.scheduleAtFrame(Math.round(SOLVE_AT_S / probe.controlDt), () => { expect = task.failureCase(probe) })
    while (!got.length && probe.steps < (task.timeout + 5) / probe.dt) probe.step()
    const h = got[0]?.header
    if (!h) fail('failure case produced no episode')
    else if (h.outcome !== expect.outcome || (expect.result && !expect.result(h.result))) fail(`failure case ended as ${h.outcome} ${JSON.stringify(h.result)}, expected ${expect.outcome}`)
    else console.log(`failure case: episode ended as '${h.outcome}' as promised`)
    probe.dispose()
  }

  // 6. rendered geoms (group <= 2, as sim.worker.js describes the scene) must carry a material
  const bare = []
  for (let g = 0; g < m.ngeom; g++) {
    if (m.geom_group[g] <= 2 && m.geom_matid[g] < 0) bare.push(mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM.value, g) || `geom ${g} of ${mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY.value, m.geom_bodyid[g])}`)
  }
  if (bare.length) fail(`rendered geoms without a material (collision-only geoms need group="3"): ${bare.join(', ')}`)
  else console.log('material check: every rendered geom has a material')
  m.delete()
}

if (args.out) {
  await fs.writeFile(args.out, Buffer.concat(recorded.map(e => Buffer.from(encodeEpisode(e.header, e.frames)))))
  console.log(`\nwrote ${recorded.length} episodes to ${args.out}`)
}
if (failures) { console.error(`\n${failures} check(s) failed`); process.exit(1) }
console.log('\nall checks passed')
