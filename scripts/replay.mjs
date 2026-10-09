// Verifies and (optionally) dumps recorded episodes with the same MuJoCo build that recorded them.
//   node scripts/replay.mjs episodes.iamr                 verify every episode replays bit-for-bit
//   node scripts/replay.mjs episodes.iamr poses.ndjson    also stream per-frame body poses for a renderer
// The dump is newline-delimited JSON: one {"episode": header} line, then one {"t", "xpos", "xquat"} line per
// frame (MuJoCo body order, world frame, quaternions w,x,y,z). A renderer needs only the logged qpos; this
// script is the reference for how the actions + events reproduce them, and a check that the headset's build
// matches this one.
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { decodeEpisodes } from '../src/sim/episode.js'
import { loadScene } from '../src/sim/loadScene.js'
import { TIMESTEPS } from '../src/sim/TaskSim.js'
import { TASKS } from '../src/sim/tasks/index.js'
import { applyPhysics, compiledPhysics, replayEpisode } from '../src/sim/replay.js'

const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
// the package does not export its package.json, so read it from node_modules directly
const INSTALLED_MUJOCO = JSON.parse(fs.readFileSync(new URL('../node_modules/@mujoco/mujoco/package.json', import.meta.url))).version
const [file, out] = process.argv.slice(2)
if (!file) { console.error('usage: node scripts/replay.mjs <episodes.iamr> [poses.ndjson]'); process.exit(2) }

const mj = await loadMujoco()
const readFile = async p => (p.endsWith('.xml') ? fs.promises.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.promises.readFile(path.join(PUBLIC, p))))
const bytes = await fs.promises.readFile(file)
const episodes = decodeEpisodes(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)) // Buffer may be a pooled slice
const models = new Map() // `${scene}|${timestep}` -> compiled model, asset hashes, compiled physics
const sink = out ? fs.createWriteStream(out) : null
let failures = 0

for (const { header, frames } of episodes) {
  const tag = `episode ${header.episode} (${header.task}, ${header.outcome}, ${header.frames} frames)`
  // headers come from a downloaded file: only load scenes this build knows, at timesteps it supports
  if (TASKS[header.task]?.scene !== header.scene || !TIMESTEPS.includes(header.timestep)) {
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
  applyPhysics(mj, m, header, base)
  if (sink) sink.write(JSON.stringify({ episode: header }) + '\n')
  const r = replayEpisode(mj, m, header, frames, {
    onFrame: sink ? (i, d) => sink.write(JSON.stringify({ t: i / header.control_hz, xpos: Array.from(d.xpos), xquat: Array.from(d.xquat) }) + '\n') : null,
  })
  if (r.mismatch >= 0) { failures++; console.error(`${tag}: diverged at frame ${r.mismatch}`) }
  else console.log(`${tag}: replays bit-for-bit`)
}
if (sink) await new Promise(resolve => sink.end(resolve))
for (const { model } of models.values()) model.delete()
if (failures) process.exit(1)
