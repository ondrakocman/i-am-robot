// Verifies and (optionally) dumps recorded episodes with the same MuJoCo build that recorded them.
//   node scripts/replay.mjs episodes.iamr            verify every episode replays bit-for-bit
//   node scripts/replay.mjs episodes.iamr out.json   also write per-frame body poses (for a renderer)
// A downstream renderer only needs the logged qpos (no simulation); this script is the reference for how
// the actions + events reproduce them, and a check that the headset's build matches this one.
import loadMujoco from '@mujoco/mujoco'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { decodeEpisodes } from '../src/sim/episode.js'
import { loadScene } from '../src/sim/loadScene.js'
import { applyPhysics, compiledPhysics, replayEpisode } from '../src/sim/replay.js'

const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
const [file, out] = process.argv.slice(2)
if (!file) { console.error('usage: node scripts/replay.mjs <episodes.iamr> [poses.json]'); process.exit(2) }

const mj = await loadMujoco()
const readFile = async p => (p.endsWith('.xml') ? fs.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.readFile(path.join(PUBLIC, p))))
const episodes = decodeEpisodes((await fs.readFile(file)).buffer)
const models = new Map() // scene -> compiled model (+ asset hashes to compare with the header)
let failures = 0
const dump = []

for (const { header, frames } of episodes) {
  if (!models.has(header.scene)) {
    const loaded = await loadScene(mj, readFile, { scene: header.scene, timestep: header.timestep })
    models.set(header.scene, { ...loaded, base: compiledPhysics(loaded.model) })
  }
  const { model: m, assets, base } = models.get(header.scene)
  const stale = Object.entries(header.assets ?? {}).filter(([p, h]) => assets[p] !== h).map(([p]) => p)
  if (stale.length) console.warn(`episode ${header.episode} (${header.task}): assets changed since recording: ${stale.join(', ')}`)
  applyPhysics(mj, m, header, base)
  const bodies = []
  const r = replayEpisode(mj, m, header, frames, {
    onFrame: out ? (i, d) => { bodies.push(Array.from(d.xpos), Array.from(d.xquat)) } : null,
  })
  const tag = `episode ${header.episode} (${header.task}, ${header.outcome}, ${header.frames} frames)`
  if (r.mismatch >= 0) { failures++; console.error(`${tag}: diverged at frame ${r.mismatch}`) }
  else console.log(`${tag}: replays bit-for-bit`)
  if (out) dump.push({ header, xpos_xquat: bodies })
}
if (out) await fs.writeFile(out, JSON.stringify(dump))
if (failures) process.exit(1)
