import * as THREE from 'three'
import { DEX31_JOINTS } from '../constants/kinematics.js'

// ─── Hand Retargeting: Quest 3 (25 joints) → Dex 3.1 (7 DoF) ────────────────
//
// Strategy: measure flexion angles from consecutive joint positions
// in the human hand, then map linearly to Dex 3.1 joint ranges.
// Ring and pinky data are discarded entirely.

const _v0 = new THREE.Vector3()
const _v1 = new THREE.Vector3()
const _v2 = new THREE.Vector3()

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

// Compute the bend angle (radians) at joint B given positions A–B–C
function bendAngle(A, B, C) {
  _v0.copy(A).sub(B).normalize()
  _v1.copy(C).sub(B).normalize()
  return Math.acos(clamp(_v0.dot(_v1), -1, 1))
}

// Compute abduction angle of thumb relative to palm normal
// Uses wrist, index metacarpal, thumb metacarpal to define palm plane
function thumbAbductionAngle(wrist, indexMeta, thumbMeta) {
  const palmRight  = _v0.copy(indexMeta).sub(wrist).normalize()
  const thumbDir   = _v1.copy(thumbMeta).sub(wrist).normalize()
  const angle = Math.acos(clamp(palmRight.dot(thumbDir), -1, 1))
  // Remap: straight out = 0, fully adducted = negative, abducted = positive
  return angle - Math.PI / 2
}

// ─── Main retargeting function ────────────────────────────────────────────────
// joints: object { [jointName]: { position: THREE.Vector3 } }
// Returns: { thumb: [j0,j1,j2], index: [j0,j1], middle: [j0,j1] } in radians

export function retargetHand(joints) {
  const result = {
    thumb:  [0, 0, 0],
    index:  [0, 0],
    middle: [0, 0],
  }

  if (!joints) return result

  const get = (name) => joints[name]?.position

  // ── Thumb ──────────────────────────────────────────────────────────────────
  const thumbMeta = get('thumb-metacarpal')
  const thumbProx = get('thumb-phalanx-proximal')
  const thumbDist = get('thumb-phalanx-distal')
  const thumbTip  = get('thumb-tip')
  const wrist     = get('wrist')
  const indexMeta = get('index-finger-metacarpal')

  if (thumbMeta && thumbProx && thumbDist && thumbTip && wrist && indexMeta) {
    // j0: abduction
    const abduct = thumbAbductionAngle(wrist, indexMeta, thumbMeta)
    result.thumb[0] = clamp(abduct, DEX31_JOINTS.thumb[0].min, DEX31_JOINTS.thumb[0].max)

    // j1: proximal flexion — angle at thumb metacarpal–proximal–distal
    const proxFlex = bendAngle(thumbMeta, thumbProx, thumbDist)
    // bend angle is 180° when straight, less when bent — invert mapping
    const proxNorm = 1 - (proxFlex / Math.PI)
    result.thumb[1] = clamp(
      proxNorm * (DEX31_JOINTS.thumb[1].max - DEX31_JOINTS.thumb[1].min) + DEX31_JOINTS.thumb[1].min,
      DEX31_JOINTS.thumb[1].min,
      DEX31_JOINTS.thumb[1].max
    )

    // j2: distal flexion — angle at proximal–distal–tip
    const distFlex = bendAngle(thumbProx, thumbDist, thumbTip)
    const distNorm = 1 - (distFlex / Math.PI)
    result.thumb[2] = clamp(
      distNorm * DEX31_JOINTS.thumb[2].max,
      DEX31_JOINTS.thumb[2].min,
      DEX31_JOINTS.thumb[2].max
    )
  }

  // ── Index Finger ───────────────────────────────────────────────────────────
  const indexProx = get('index-finger-phalanx-proximal')
  const indexMid  = get('index-finger-phalanx-intermediate')
  const indexDist = get('index-finger-phalanx-distal')
  const indexTip  = get('index-finger-tip')

  if (indexMeta && indexProx && indexMid && indexDist && indexTip) {
    // j0: proximal — angle at metacarpal–proximal–intermediate
    const flex0 = bendAngle(indexMeta, indexProx, indexMid)
    const norm0 = 1 - (flex0 / Math.PI)
    result.index[0] = clamp(norm0 * DEX31_JOINTS.index[0].max, 0, DEX31_JOINTS.index[0].max)

    // j1: distal — average of intermediate and distal joints for stability
    const flex1a = bendAngle(indexProx, indexMid, indexDist)
    const flex1b = bendAngle(indexMid, indexDist, indexTip)
    const norm1 = 1 - ((flex1a + flex1b) / (2 * Math.PI))
    result.index[1] = clamp(norm1 * DEX31_JOINTS.index[1].max, 0, DEX31_JOINTS.index[1].max)
  }

  // ── Middle Finger ──────────────────────────────────────────────────────────
  const middleMeta = get('middle-finger-metacarpal')
  const middleProx = get('middle-finger-phalanx-proximal')
  const middleMid  = get('middle-finger-phalanx-intermediate')
  const middleDist = get('middle-finger-phalanx-distal')
  const middleTip  = get('middle-finger-tip')

  if (middleMeta && middleProx && middleMid && middleDist && middleTip) {
    const flex0 = bendAngle(middleMeta, middleProx, middleMid)
    const norm0 = 1 - (flex0 / Math.PI)
    result.middle[0] = clamp(norm0 * DEX31_JOINTS.middle[0].max, 0, DEX31_JOINTS.middle[0].max)

    const flex1a = bendAngle(middleProx, middleMid, middleDist)
    const flex1b = bendAngle(middleMid, middleDist, middleTip)
    const norm1 = 1 - ((flex1a + flex1b) / (2 * Math.PI))
    result.middle[1] = clamp(norm1 * DEX31_JOINTS.middle[1].max, 0, DEX31_JOINTS.middle[1].max)
  }

  return result
}

// ─── Smooth Retargeting State ─────────────────────────────────────────────────
// Applies per-joint exponential smoothing to prevent jitter in finger angles
export class RetargetingFilter {
  constructor(alpha = 0.15) {
    this.alpha = alpha
    this.last = null
  }

  update(raw) {
    if (!this.last) {
      this.last = {
        thumb:  [...raw.thumb],
        index:  [...raw.index],
        middle: [...raw.middle],
      }
      return this.last
    }

    const lerp = (a, b) => a + (b - a) * this.alpha

    this.last.thumb  = raw.thumb.map((v, i)  => lerp(this.last.thumb[i],  v))
    this.last.index  = raw.index.map((v, i)  => lerp(this.last.index[i],  v))
    this.last.middle = raw.middle.map((v, i) => lerp(this.last.middle[i], v))

    return this.last
  }

  reset() {
    this.last = null
  }
}
