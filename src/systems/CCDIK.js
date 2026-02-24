import * as THREE from 'three'

// ─── CCD IK with Position + Orientation Goals ────────────────────────────────
// Works in world space throughout. Each joint's rotation axis is projected
// to world space via the joint's world quaternion, avoiding frame mismatch.

const _endPos      = new THREE.Vector3()
const _jointPos    = new THREE.Vector3()
const _toEnd       = new THREE.Vector3()
const _toTarget    = new THREE.Vector3()
const _cross       = new THREE.Vector3()
const _worldAxis   = new THREE.Vector3()
const _jointWorldQ = new THREE.Quaternion()
const _endQuat     = new THREE.Quaternion()
const _deltaQuat   = new THREE.Quaternion()
const _orientAxis  = new THREE.Vector3()

export function solveCCDIK(joints, endEffector, targetPos, targetQuat, iterations = 25) {
  const n = joints.length

  for (let iter = 0; iter < iterations; iter++) {
    endEffector.getWorldPosition(_endPos)
    endEffector.getWorldQuaternion(_endQuat)

    const posErr = _endPos.distanceTo(targetPos)
    const oriErr = _endQuat.angleTo(targetQuat)
    if (posErr < 0.003 && oriErr < 0.03) return

    for (let i = n - 1; i >= 0; i--) {
      const joint = joints[i]
      if (!joint.setJointValue) continue

      const t = n > 1 ? i / (n - 1) : 0
      const posW = 1.0 - t * 0.8
      const oriW = t * 0.8

      // Joint rotation axis in world space.
      // Rotation around this axis doesn't change its own direction,
      // so the world quaternion (which includes the joint angle) still
      // gives the correct world-space axis.
      joint.getWorldQuaternion(_jointWorldQ)
      _worldAxis.copy(joint.axis).applyQuaternion(_jointWorldQ).normalize()

      let totalAngle = 0

      // ── Position contribution ───────────────────────────────────────────
      endEffector.getWorldPosition(_endPos)
      joint.getWorldPosition(_jointPos)

      _toEnd.subVectors(_endPos, _jointPos)
      _toTarget.subVectors(targetPos, _jointPos)
      const lenE = _toEnd.length()
      const lenT = _toTarget.length()

      if (lenE > 1e-6 && lenT > 1e-6) {
        _toEnd.divideScalar(lenE)
        _toTarget.divideScalar(lenT)
        _cross.crossVectors(_toEnd, _toTarget)
        const sinA = _cross.length()
        const cosA = _toEnd.dot(_toTarget)

        if (sinA > 1e-6) {
          _cross.divideScalar(sinA)
          const proj = _cross.dot(_worldAxis)
          if (Math.abs(proj) > 0.01) {
            totalAngle += Math.atan2(sinA, cosA) * Math.sign(proj) * posW
          }
        }
      }

      // ── Orientation contribution ────────────────────────────────────────
      if (oriW > 0.01 && targetQuat) {
        endEffector.getWorldQuaternion(_endQuat)
        _deltaQuat.copy(_endQuat).invert().premultiply(targetQuat).normalize()

        if (_deltaQuat.w < 0) {
          _deltaQuat.x = -_deltaQuat.x
          _deltaQuat.y = -_deltaQuat.y
          _deltaQuat.z = -_deltaQuat.z
          _deltaQuat.w = -_deltaQuat.w
        }

        const halfAngle = Math.acos(Math.min(1, _deltaQuat.w))
        if (halfAngle > 0.005) {
          const sa = Math.sin(halfAngle)
          _orientAxis.set(
            _deltaQuat.x / sa,
            _deltaQuat.y / sa,
            _deltaQuat.z / sa
          ).normalize()

          const proj = _orientAxis.dot(_worldAxis)
          if (Math.abs(proj) > 0.01) {
            totalAngle += halfAngle * 2 * Math.sign(proj) * oriW
          }
        }
      }

      totalAngle *= 0.5
      if (Math.abs(totalAngle) < 0.0001) continue

      const newAngle = (joint.angle || 0) + totalAngle
      const lo = joint.limit?.lower ?? -Math.PI
      const hi = joint.limit?.upper ?? Math.PI
      joint.setJointValue(Math.max(lo, Math.min(hi, newAngle)))
    }
  }
}
