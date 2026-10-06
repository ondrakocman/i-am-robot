// Episode container: a file is any number of concatenated chunks
//   u32 magic 'IAMR' | u32 header bytes | header JSON (utf-8, zero-padded to 4) | u32 data bytes | float32 frames
// Reader: scripts/load_episodes.py

export const EPISODE_MAGIC = 0x524d4149 // "IAMR" read as little-endian u32

export function encodeEpisode(header, frames) {
  const json = new TextEncoder().encode(JSON.stringify(header))
  const headerBytes = Math.ceil(json.length / 4) * 4
  const buf = new ArrayBuffer(8 + headerBytes + 4 + frames.byteLength)
  const view = new DataView(buf)
  view.setUint32(0, EPISODE_MAGIC, true)
  view.setUint32(4, headerBytes, true)
  new Uint8Array(buf, 8, json.length).set(json)
  view.setUint32(8 + headerBytes, frames.byteLength, true)
  new Uint8Array(buf, 12 + headerBytes).set(new Uint8Array(frames.buffer, frames.byteOffset, frames.byteLength))
  return buf
}

/** Fixed-layout frame log, grown on demand. `fields` is [{ name, size }] in frame order. */
export class EpisodeRecorder {
  constructor(fields) {
    let offset = 0
    this.fields = fields.map(f => { const r = { ...f, offset }; offset += f.size; return r })
    this.frameSize = offset
    this.buf = new Float32Array(this.frameSize * 50 * 60)
    this.frames = 0
  }

  clear() { this.frames = 0 }

  /** Appends one frame; `values` are array-likes in field order. */
  push(values) {
    const need = (this.frames + 1) * this.frameSize
    if (need > this.buf.length) {
      const grown = new Float32Array(this.buf.length * 2)
      grown.set(this.buf)
      this.buf = grown
    }
    let o = this.frames * this.frameSize
    for (let i = 0; i < values.length; i++) {
      const v = values[i], size = this.fields[i].size
      if (typeof v === 'number') this.buf[o] = v
      else for (let k = 0; k < size; k++) this.buf[o + k] = v[k]
      o += size
    }
    this.frames++
  }

  snapshot() { return this.buf.slice(0, this.frames * this.frameSize) }
}
