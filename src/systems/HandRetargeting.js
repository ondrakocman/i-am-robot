import * as THREE from 'three'

const _v0 = new THREE.Vector3()
const _v1 = new THREE.Vector3()

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

function measureCurl(metacarpal, proximal, tip) {
  const len = metacarpal.distanceTo(proximal) + proximal.distanceTo(tip)
  if (len < 0.001) return 0
  return clamp((0.92 - metacarpal.distanceTo(tip) / len) / 0.35, 0, 1)
}

function measureJointBend(A, B, C) {
  _v0.copy(A).sub(B).normalize()
  _v1.copy(C).sub(B).normalize()
  return clamp((Math.PI - Math.acos(clamp(_v0.dot(_v1), -1, 1))) / (Math.PI * 0.35), 0, 1)
}

/**
 * Maps Quest 3 hand joints to normalized Dex 3.1 curl/abduction factors.
 * Output is 0-1 normalized; the caller maps to actual URDF joint limits.
 */
export function retargetHand(joints) {
  const result = {
    thumb:  { abduction: 0, curl: [0, 0] },
    index:  { curl: [0, 0] },
    middle: { curl: [0, 0] },
  }
  if (!joints) return result

  const get = (name) => joints[name]?.position

  const wrist     = get('wrist')
  const thumbMeta = get('thumb-metacarpal')
  const thumbProx = get('thumb-phalanx-proximal')
  const thumbDist = get('thumb-phalanx-distal')
  const thumbTip  = get('thumb-tip')
  const indexMeta = get('index-finger-metacarpal')
  const indexProx = get('index-finger-phalanx-proximal')
  const indexTip  = get('index-finger-tip')
  const middleMeta = get('middle-finger-metacarpal')

  // Thumb abduction: angle between thumb bone and index bone directions
  if (wrist && thumbMeta && thumbProx && thumbDist && thumbTip && indexMeta && indexProx) {
    _v0.copy(thumbProx).sub(thumbMeta).normalize()
    _v1.copy(indexProx).sub(indexMeta).normalize()
    const angle = Math.acos(clamp(_v0.dot(_v1), -1, 1))
    result.thumb.abduction = clamp((angle - 1.15) / 0.5, -1, 1)

    // Thumb curl: direct joint bend measurement (×1.5 compensates for thumb's smaller flex range)
    result.thumb.curl[0] = clamp(measureJointBend(thumbMeta, thumbProx, thumbDist) * 1.5, 0, 1)
    result.thumb.curl[1] = clamp(measureJointBend(thumbProx, thumbDist, thumbTip) * 1.5, 0, 1)
  }

  // Index finger
  const indexMid  = get('index-finger-phalanx-intermediate')
  const indexDist = get('index-finger-phalanx-distal')
  if (indexMeta && indexProx && indexMid && indexDist && indexTip) {
    result.index.curl[0] = measureCurl(indexMeta, indexProx, indexTip)
    result.index.curl[1] = measureJointBend(indexMid, indexDist, indexTip)
  }

  // Middle finger
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

    const a = this.alpha
    const lerp = (prev, next) => prev + (next - prev) * a

    this.last.thumb.abduction = lerp(this.last.thumb.abduction, raw.thumb.abduction)
    this.last.thumb.curl  = raw.thumb.curl.map((v, i)  => lerp(this.last.thumb.curl[i],  v))
    this.last.index.curl  = raw.index.curl.map((v, i)  => lerp(this.last.index.curl[i],  v))
    this.last.middle.curl = raw.middle.curl.map((v, i) => lerp(this.last.middle.curl[i], v))

    return this.last
  }

  reset() { this.last = null }
}
