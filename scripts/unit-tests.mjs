// Unit tests for the parts the physics gate cannot reach: XR <-> robot frame conventions, hand retargeting,
// smoothing, and the episode file round trip. Run with `npm test` (node:test, no extra dependencies).
import { test } from 'node:test'
import assert from 'node:assert/strict'
import * as THREE from 'three'
import { ROBOT_BASE_QUAT, XR_TO_URDF_L, XR_TO_URDF_R } from '../src/constants/kinematics.js'
import { retargetHand, RetargetingFilter } from '../src/systems/HandRetargeting.js'
import { smoothingAlpha, QuaternionSmoother } from '../src/systems/Smoothing.js'
import { OneEuroVector3 } from '../src/systems/OneEuroFilter.js'
import { encodeEpisode, decodeEpisodes, EpisodeRecorder, EPISODE_FORMAT } from '../src/sim/episode.js'

const close = (a, b, eps = 1e-6) => assert.ok(a.distanceTo(b) < eps, `${a.toArray()} != ${b.toArray()}`)
const v = (x, y, z) => new THREE.Vector3(x, y, z)

test('robot frame (x forward, z up) maps to three.js (y up, robot facing -z)', () => {
  close(v(1, 0, 0).applyQuaternion(ROBOT_BASE_QUAT), v(0, 0, -1)) // forward
  close(v(0, 0, 1).applyQuaternion(ROBOT_BASE_QUAT), v(0, 1, 0))  // up
  close(v(0, 1, 0).applyQuaternion(ROBOT_BASE_QUAT), v(-1, 0, 0)) // robot's left is three.js -x
})

test('XR wrist -> URDF palm: fingers point along palm +x, palms face each other', () => {
  // WebXR wrist frame: -z along the fingers, +y out of the back of the hand. A hand held flat, palm down,
  // fingers forward (three.js -z) has the identity wrist orientation.
  const fingersXR = v(0, 0, -1)
  const backOfHandXR = v(0, 1, 0)
  for (const [correction, side] of [[XR_TO_URDF_L, 'left'], [XR_TO_URDF_R, 'right']]) {
    // the corrected quaternion q_palm = q_wrist * correction: a palm-frame vector p appears in the world at
    // q_wrist * correction * p, so correction maps palm axes into the wrist frame
    const palmX = v(1, 0, 0).applyQuaternion(correction)
    close(palmX, fingersXR, 1e-6)
    // URDF: the left palm faces -y, the right palm faces +y, i.e. the back of the hand is +y (left) / -y (right)
    const palmY = v(0, 1, 0).applyQuaternion(correction)
    close(palmY, side === 'left' ? backOfHandXR : backOfHandXR.clone().negate(), 1e-6)
  }
})

test('retargeting: an open hand gives zero curl, a fist gives full curl', () => {
  const chain = (names, step) => Object.fromEntries(names.map((n, i) => [n, { position: v(0, 0, -0.03 * i).add(step.clone().multiplyScalar(i)) }]))
  const open = {
    ...chain(['index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip'], v(0, 0, 0)),
    ...chain(['middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip'], v(0, 0, 0)),
  }
  const r = retargetHand(open)
  assert.equal(r.index.curl[0], 0)
  assert.equal(r.middle.curl[0], 0)
  // a fist: the tip folds back next to the metacarpal
  const fist = {
    'index-finger-metacarpal': { position: v(0, 0, 0) }, 'index-finger-phalanx-proximal': { position: v(0, 0, -0.04) },
    'index-finger-phalanx-intermediate': { position: v(0, -0.02, -0.05) }, 'index-finger-phalanx-distal': { position: v(0, -0.04, -0.03) },
    'index-finger-tip': { position: v(0, -0.04, -0.01) },
  }
  assert.ok(retargetHand(fist).index.curl[0] > 0.9)
  assert.equal(retargetHand({}).thumb.abduction, 0) // nothing tracked: neutral
})

test('filters: time-constant smoothing is independent of the frame rate', () => {
  // the same wall-clock time at 72 Hz and 120 Hz must reach the same fraction of the target
  const run = (hz, tau) => { let x = 0; for (let i = 0; i < hz; i++) x += (1 - x) * smoothingAlpha(tau, 1 / hz); return x }
  assert.ok(Math.abs(run(72, 0.05) - run(120, 0.05)) < 0.01)
  const q = new QuaternionSmoother(0.02)
  const target = new THREE.Quaternion().setFromAxisAngle(v(0, 1, 0), 1)
  q.update(new THREE.Quaternion(), 1 / 90)
  for (let i = 0; i < 90; i++) q.update(target, 1 / 90)
  assert.ok(q.value.angleTo(target) < 0.01)
  const f = new RetargetingFilter(0.03)
  const raw = { thumb: { abduction: 1, curl: [1, 1] }, index: { curl: [1, 1] }, middle: { curl: [1, 1] } }
  f.update({ thumb: { abduction: 0, curl: [0, 0] }, index: { curl: [0, 0] }, middle: { curl: [0, 0] } }, 1 / 90)
  for (let i = 0; i < 90; i++) f.update(raw, 1 / 90)
  assert.ok(f.last.index.curl[0] > 0.99)
  const e = new OneEuroVector3()
  e.update(v(0, 0, 0), 1 / 90)
  for (let i = 0; i < 90; i++) e.update(v(1, 0, 0), 1 / 90)
  assert.ok(e.value.x > 0.95)
})

test('episode file: encode -> decode round trip and corruption detection', () => {
  const rec = new EpisodeRecorder([{ name: 'time', size: 1 }, { name: 'qpos', size: 3 }], 4)
  for (let i = 0; i < 3; i++) rec.push([i * 0.02, [i, i + 1, i + 2]])
  const header = { format: EPISODE_FORMAT, episode: 1, frames: rec.frames, frame_size: rec.frameSize, fields: rec.fields }
  const one = encodeEpisode(header, rec.snapshot())
  const two = new Uint8Array(one.byteLength * 2)
  two.set(new Uint8Array(one), 0)
  two.set(new Uint8Array(one), one.byteLength)
  const episodes = decodeEpisodes(two.buffer)
  assert.equal(episodes.length, 2)
  assert.deepEqual(Array.from(episodes[1].frames.subarray(4, 8)), [0.02 * 1, 1, 2, 3].map(Math.fround))
  assert.throws(() => decodeEpisodes(one.slice(0, one.byteLength - 4)), /truncated|does not match/)
  const bad = { ...header, frames: 2 }
  assert.throws(() => decodeEpisodes(encodeEpisode(bad, rec.snapshot())), /does not match/)
})
