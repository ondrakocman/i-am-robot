import * as THREE from 'three'

// ─── Hand Retargeting: Quest 3 → Dex 3.1 ─────────────────────────────────────
//
// Outputs normalized curl/spread factors (0-1), NOT joint angles.
// The caller maps these to actual URDF joint limits (which differ
// between left and right hands due to mirrored joint conventions).
//
// Output:
//   thumb:  { abduction: -1..1, curl: [j1_curl, j2_curl] }  (0=open, 1=closed)
//   index:  { curl: [j0_curl, j1_curl] }
//   middle: { curl: [j0_curl, j1_curl] }

const _v0 = new THREE.Vector3()
const _v1 = new THREE.Vector3()
const _v2 = new THREE.Vector3()
const _palmNormal = new THREE.Vector3()

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

// Measure how curled a finger is based on tip-to-base distance ratio.
// Returns 0 (straight) to 1 (fully curled/fist).
function measureCurl(metacarpal, proximal, tip) {
  const fingerLength = metacarpal.distanceTo(proximal) + proximal.distanceTo(tip)
  if (fingerLength < 0.001) return 0
  const directDist = metacarpal.distanceTo(tip)
  const ratio = directDist / fingerLength
  // ratio ~0.95 when straight, ~0.25 when fully curled
  return clamp((0.92 - ratio) / 0.60, 0, 1)
}

// Measure individual joint bend. Returns 0 (straight) to 1 (max bend).
function measureJointBend(A, B, C) {
  _v0.copy(A).sub(B).normalize()
  _v1.copy(C).sub(B).normalize()
  const angle = Math.acos(clamp(_v0.dot(_v1), -1, 1))
  // angle = PI when straight, ~PI/3 when fully bent
  // Map: PI → 0, PI/3 → 1
  return clamp((Math.PI - angle) / (Math.PI * 0.6), 0, 1)
}

// Thumb abduction: how far the thumb is spread from the palm.
// Returns -1 (adducted/tucked in) to +1 (fully spread out).
function measureThumbAbduction(wrist, indexMeta, thumbProx) {
  // Palm forward direction: wrist → index metacarpal
  _v0.copy(indexMeta).sub(wrist).normalize()
  // Thumb direction: wrist → thumb proximal
  _v1.copy(thumbProx).sub(wrist).normalize()

  // Cross product gives palm normal
  _palmNormal.crossVectors(_v0, _v1).normalize()

  // Angle between palm forward and thumb direction
  const dot = _v0.dot(_v1)
  const angle = Math.acos(clamp(dot, -1, 1))

  // Determine sign: use cross product to check which side of palm the thumb is on
  const cross = _v0.clone().cross(_v1)
  const sign = cross.dot(_palmNormal) > 0 ? 1 : -1

  // Remap: ~30° when adducted to ~90° when spread
  // Neutral (thumb alongside palm) ≈ 45°
  return clamp((angle - 0.75) / 0.7 * sign, -1, 1)
}

export function retargetHand(joints) {
  const result = {
    thumb:  { abduction: 0, curl: [0, 0] },
    index:  { curl: [0, 0] },
    middle: { curl: [0, 0] },
  }

  if (!joints) return result

  const get = (name) => joints[name]?.position

  // ── Thumb ──────────────────────────────────────────────────────────────────
  const wrist     = get('wrist')
  const thumbMeta = get('thumb-metacarpal')
  const thumbProx = get('thumb-phalanx-proximal')
  const thumbDist = get('thumb-phalanx-distal')
  const thumbTip  = get('thumb-tip')
  const indexMeta = get('index-finger-metacarpal')

  if (wrist && thumbMeta && thumbProx && thumbDist && thumbTip && indexMeta) {
    // Abduction: side-to-side spread
    result.thumb.abduction = measureThumbAbduction(wrist, indexMeta, thumbProx)

    // Thumb curl: j1 (proximal flexion) and j2 (distal flexion)
    result.thumb.curl[0] = measureJointBend(thumbMeta, thumbProx, thumbDist)
    result.thumb.curl[1] = measureJointBend(thumbProx, thumbDist, thumbTip)
  }

  // ── Index ──────────────────────────────────────────────────────────────────
  const indexProx = get('index-finger-phalanx-proximal')
  const indexMid  = get('index-finger-phalanx-intermediate')
  const indexDist = get('index-finger-phalanx-distal')
  const indexTip  = get('index-finger-tip')

  if (indexMeta && indexProx && indexMid && indexDist && indexTip) {
    // Overall curl for j0 (metacarpal-proximal flexion)
    result.index.curl[0] = measureCurl(indexMeta, indexProx, indexTip)
    // Distal curl for j1
    result.index.curl[1] = measureJointBend(indexMid, indexDist, indexTip)
  }

  // ── Middle ─────────────────────────────────────────────────────────────────
  const middleMeta = get('middle-finger-metacarpal')
  const middleProx = get('middle-finger-phalanx-proximal')
  const middleMid  = get('middle-finger-phalanx-intermediate')
  const middleDist = get('middle-finger-phalanx-distal')
  const middleTip  = get('middle-finger-tip')

  if (middleMeta && middleProx && middleMid && middleDist && middleTip) {
    result.middle.curl[0] = measureCurl(middleMeta, middleProx, middleTip)
    result.middle.curl[1] = measureJointBend(middleMid, middleDist, middleTip)
  }

  return result
}

// ─── Smooth Retargeting State ─────────────────────────────────────────────────
export class RetargetingFilter {
  constructor(alpha = 0.3) {
    this.alpha = alpha
    this.last = null
  }

  update(raw) {
    if (!this.last) {
      this.last = {
        thumb:  { abduction: raw.thumb.abduction, curl: [...raw.thumb.curl] },
        index:  { curl: [...raw.index.curl] },
        middle: { curl: [...raw.middle.curl] },
      }
      return this.last
    }

    const lerp = (a, b) => a + (b - a) * this.alpha

    this.last.thumb.abduction = lerp(this.last.thumb.abduction, raw.thumb.abduction)
    this.last.thumb.curl  = raw.thumb.curl.map((v, i)  => lerp(this.last.thumb.curl[i],  v))
    this.last.index.curl  = raw.index.curl.map((v, i)  => lerp(this.last.index.curl[i],  v))
    this.last.middle.curl = raw.middle.curl.map((v, i) => lerp(this.last.middle.curl[i], v))

    return this.last
  }

  reset() { this.last = null }
}
