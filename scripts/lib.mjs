// Shared bits of the Node tooling: the installed MuJoCo version, a reader for public/ and a streaming reader
// for .iamr files.
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { EPISODE_MAGIC, checkEpisode } from '../src/sim/episode.js'

export const PUBLIC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'public')
// the package does not export its package.json, so read it from node_modules directly
export const MUJOCO_VERSION = JSON.parse(fs.readFileSync(new URL('../node_modules/@mujoco/mujoco/package.json', import.meta.url))).version

/** loadScene's file reader for the checked-out public/ directory. */
export const readPublic = async p => (p.endsWith('.xml') ? fs.promises.readFile(path.join(PUBLIC, p), 'utf8') : new Uint8Array(await fs.promises.readFile(path.join(PUBLIC, p))))

/**
 * Iterates the episodes of a .iamr file, reading one at a time, so files larger than memory (or than the
 * 2 GiB readFile limit) work. Yields { header, frames: Float32Array }.
 */
export async function* readEpisodeFile(file) {
  const fh = await fs.promises.open(file)
  try {
    const { size } = await fh.stat()
    let o = 0
    const read = async (n, into = new Uint8Array(n)) => {
      const { bytesRead } = await fh.read(into, 0, n, o)
      if (bytesRead !== n) throw new Error(`truncated file: episode at byte ${o} is incomplete`)
      o += n
      return into
    }
    while (o < size) {
      const start = o
      const head = new DataView((await read(8)).buffer)
      if (head.getUint32(0, true) !== EPISODE_MAGIC) throw new Error(`bad magic at byte ${start}`)
      const headerBytes = head.getUint32(4, true)
      const header = JSON.parse(new TextDecoder().decode(await read(headerBytes)).replace(/\0+$/, ''))
      const dataBytes = new DataView((await read(4)).buffer).getUint32(0, true)
      checkEpisode(header, dataBytes)
      const frames = new Float32Array(dataBytes / 4)
      await read(dataBytes, new Uint8Array(frames.buffer))
      yield { header, frames }
    }
  } finally {
    await fh.close()
  }
}
