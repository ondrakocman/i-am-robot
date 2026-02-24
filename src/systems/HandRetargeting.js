import * as THREE from 'three'

// ─── Hand Retargeting: Quest 3 → Dex 3.1 ─────────────────────────────────────
//
// Outputs normalized curl/spread factors (0-1), NOT joint angles.
// The caller maps these to actual URDF joint limits (which differ
// between left and right hands due to mirrored joint conventions).

const _v0 = new THREE.Vector3()
const _v1 = new THREE.Vector3()

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

// Finger curl via tip-to-base distance ratio.
// 0 = straight, 1 = fully curled.
function measureCurl(metacarpal, proximal, tip) {
  const fingerLength = metacarpal.distanceTo(proximal) + proximal.distanceTo(tip)
  if (fingerLength < 0.001) return 0
  const directDist = metacarpal.distanceTo(tip)
  const ratio = directDist / fingerLength
  return clamp((0.92 - ratio) / 0.60, 0, 1)
}

// Individual joint bend: 0 = straight (PI), 1 = max bend (~PI/3).
function measureJointBend(A, B, C) {
  _v0.copy(A).sub(B).normalize()
  _v1.copy(C).sub(B).normalize()
  const angle = Math.acos(clamp(_v0.dot(_v1), -1, 1))
  return clamp((Math.PI - angle) / (Math.PI * 0.6), 0, 1)
}

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

  // ── Thumb ──────────────────────────────────────────────────────────────────
  if (wrist && thumbMeta && thumbProx && thumbDist && thumbTip && indexMeta) {
    // Abduction: distance between thumb proximal and index proximal,
    // normalized by hand span. Captures side-to-side spread reliably.
    if (indexProx && middleMeta) {
      const handSpan = wrist.distanceTo(middleMeta)
      if (handSpan > 0.01) {
        const thumbToIndex = thumbProx.distanceTo(indexProx)
        const ratio = thumbToIndex / handSpan
        // ratio ~0.75 neutral, ~1.1 spread, ~0.35 tucked
        result.thumb.abduction = clamp((ratio - 0.75) / 0.35, -1, 1)
      }
    }

    // Curl: distance-based for much better pinch/curl sensitivity.
    // Bone-angle measurement is too insensitive for the thumb's complex motion.
    const thumbLen = thumbMeta.distanceTo(thumbProx) +
                     thumbProx.distanceTo(thumbDist) +
                     thumbDist.distanceTo(thumbTip)

    if (thumbLen > 0.01) {
      // General curl: how close is thumb tip to wrist?
      const tipToWrist = thumbTip.distanceTo(wrist)
      const generalCurl = clamp(1 - tipToWrist / (thumbLen * 1.3), 0, 1)

      // Pinch boost: extra curl when thumb tip approaches index finger tip
      let pinchBoost = 0
      if (indexTip) {
        const pinchDist = thumbTip.distanceTo(indexTip)
        pinchBoost = clamp(1 - pinchDist / 0.06, 0, 1) * 0.5
      }

      const totalCurl = clamp(generalCurl + pinchBoost, 0, 1)
      result.thumb.curl[0] = totalCurl
      result.thumb.curl[1] = clamp(totalCurl * 1.2, 0, 1)
    }
  }

  // ── Index ──────────────────────────────────────────────────────────────────
  const indexMid  = get('index-finger-phalanx-intermediate')
  const indexDist = get('index-finger-phalanx-distal')

  if (indexMeta && indexProx && indexMid && indexDist && indexTip) {
    result.index.curl[0] = measureCurl(indexMeta, indexProx, indexTip)
    result.index.curl[1] = measureJointBend(indexMid, indexDist, indexTip)
  }

  // ── Middle ─────────────────────────────────────────────────────────────────
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
