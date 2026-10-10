// Verifies and (optionally) dumps recorded episodes with the same MuJoCo build that recorded them.
//   node scripts/replay.mjs episodes.iamr                 verify every episode replays bit-for-bit
//   node scripts/replay.mjs episodes.iamr poses.ndjson    also stream per-frame body poses for a renderer
// The dump is newline-delimited JSON: one {"episode": header} line (header.body_names gives the body order),
// then one {"t", "xpos", "xquat"} line per frame (world frame, quaternions w,x,y,z), written with back-pressure
// so memory stays flat.
// Soft parcels add a "soft" array of particle positions to each frame line (header.soft_bodies gives the
// lattices; softLattice in src/sim/soft.js gives their surface triangles). A renderer needs only the logged
// qpos (and soft); this script is the reference for how the actions + events reproduce them, and a check that
// the headset's build matches this one.
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs'
import { once } from 'node:events'
import { loadScene } from '../src/sim/loadScene.js'
import { TIMESTEPS } from '../src/sim/TaskSim.js'
import { TASKS, hasTask } from '../src/sim/tasks/index.js'
import { applyPhysics, compiledPhysics, replayFrames } from '../src/sim/replay.js'
import { MUJOCO_VERSION as INSTALLED_MUJOCO, readPublic as readFile, readEpisodeFile } from './lib.mjs'

const [file, out] = process.argv.slice(2)
if (!file) { console.error('usage: node scripts/replay.mjs <episodes.iamr> [poses.ndjson]'); process.exit(2) }

const mj = await loadMujoco()
const models = new Map() // `${scene}|${timestep}` -> compiled model, asset hashes, compiled physics
const sink = out ? fs.createWriteStream(out) : null
const write = async line => { if (!sink.write(line + '\n')) await once(sink, 'drain') }
let failures = 0

// one episode in memory at a time, whatever the file size
for await (const { header, frames } of readEpisodeFile(file)) {
  const tag = `episode ${header.episode} (${header.task}, ${header.outcome}, ${header.frames} frames)`
  // headers come from a downloaded file: only load scenes this build knows, at timesteps it supports
  if (!hasTask(header.task) || TASKS[header.task].scene !== header.scene || !TIMESTEPS.includes(header.timestep)) {
    failures++; console.error(`${tag}: unknown scene/timestep ${header.scene} @ ${header.timestep}`); continue
  }
  if (header.mujoco !== INSTALLED_MUJOCO) console.warn(`${tag}: recorded with MuJoCo ${header.mujoco}, installed ${INSTALLED_MUJOCO}; replay may differ`)
  const key = `${header.scene}|${header.timestep}`
  if (!models.has(key)) {
    const loaded = await loadScene(mj, readFile, { scene: header.scene, timestep: header.timestep })
    models.set(key, { ...loaded, base: compiledPhysics(loaded.model) })
  }
  const { model: m, assets, base } = models.get(key)
  const stale = Object.entries(header.assets ?? {}).filter(([p, h]) => assets[p] !== h).map(([p]) => p)
  if (stale.length) console.warn(`${tag}: assets changed since recording: ${stale.join(', ')}`)
  applyPhysics(mj, m, header.physics, base)
  if (sink) await write(JSON.stringify({ episode: header }))
  const gen = replayFrames(mj, m, header, frames)
  let r = gen.next()
  while (!r.done) {
    if (sink) {
      const line = { t: r.value.i / header.control_hz, xpos: Array.from(r.value.d.xpos), xquat: Array.from(r.value.d.xquat) }
      if (r.value.soft) line.soft = Array.from(r.value.soft.gather(new Float64Array(3 * r.value.soft.total)))
      await write(JSON.stringify(line))
    }
    r = gen.next()
  }
  if (r.value >= 0) { failures++; console.error(`${tag}: diverged at frame ${r.value}`) }
  else console.log(`${tag}: replays bit-for-bit`)
}
if (sink) await new Promise(resolve => sink.end(resolve))
for (const { model } of models.values()) model.delete()
if (failures) process.exit(1)
