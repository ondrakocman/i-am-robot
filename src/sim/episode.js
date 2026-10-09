// Episode container: a file is any number of concatenated chunks
//   u32 magic 'IAMR' | u32 header bytes | header JSON (utf-8, zero-padded to 4) | u32 data bytes | float32 frames
// Readers: decodeEpisodes below (JS) and scripts/load_episodes.py (Python).

export const EPISODE_MAGIC = 0x524d4149 // "IAMR" read as little-endian u32
export const EPISODE_FORMAT = 'iamr-episode-v1'

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

/** Parses a .iamr file (one or more chunks) into [{ header, frames: Float32Array }]. */
export function decodeEpisodes(buffer) {
  return [...iterateEpisodes(buffer)]
}

/** Same as decodeEpisodes, one episode at a time (only one episode's frames are copied at once). */
export function* iterateEpisodes(buffer) {
  const view = new DataView(buffer)
  let o = 0
  while (o < buffer.byteLength) {
    if (view.getUint32(o, true) !== EPISODE_MAGIC) throw new Error(`bad magic at byte ${o}`)
    const headerBytes = view.getUint32(o + 4, true)
    const json = new TextDecoder().decode(new Uint8Array(buffer, o + 8, headerBytes)).replace(/\0+$/, '')
    const header = JSON.parse(json)
    if (header.format !== EPISODE_FORMAT) throw new Error(`unsupported episode format ${header.format}`)
    o += 8 + headerBytes
    const dataBytes = view.getUint32(o, true)
    o += 4
    if (o + dataBytes > buffer.byteLength) throw new Error(`truncated file: episode at byte ${o - 12 - headerBytes} is incomplete`)
    if (dataBytes !== header.frames * header.frame_size * 4) throw new Error(`episode ${header.episode}: data size does not match header (${dataBytes} bytes for ${header.frames} frames of ${header.frame_size})`)
    // copy so the frames are 4-byte aligned regardless of the chunk offset
    const frames = new Float32Array(buffer.slice(o, o + dataBytes))
    o += dataBytes
    yield { header, frames }
  }
}

/** Fixed-layout frame log. `fields` is [{ name, size }] in frame order; `capacity` frames are preallocated. */
export class EpisodeRecorder {
  constructor(fields, capacity = 50 * 60) {
    let offset = 0
    this.fields = fields.map(f => { const r = { ...f, offset }; offset += f.size; return r })
    this.frameSize = offset
    this.buf = new Float32Array(this.frameSize * capacity)
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

  /** The recorded frames as a view into the buffer (valid until clear()); copy before keeping it. */
  view() { return this.buf.subarray(0, this.frames * this.frameSize) }

  snapshot() { return this.buf.slice(0, this.frames * this.frameSize) }
}
