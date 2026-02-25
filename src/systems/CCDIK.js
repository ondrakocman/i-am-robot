import * as THREE from 'three'

const _endPos = new THREE.Vector3()
const _jointPos = new THREE.Vector3()
const _toEnd = new THREE.Vector3()
const _toTarget = new THREE.Vector3()
const _cross = new THREE.Vector3()
const _worldAxis = new THREE.Vector3()
const _jointWorldQ = new THREE.Quaternion()
const _endQuat = new THREE.Quaternion()
const _deltaQuat = new THREE.Quaternion()
const _orientAxis = new THREE.Vector3()

const MAX_ANGLE_STEP = 0.15
const REGULARIZATION = 0.005
const SMOOTH_COST = 0.08
const DAMPING = 0.4

/**
 * CCD IK with proximal/distal split weighting:
 *   - Proximal joints (shoulder + elbow): position only
 *   - Distal joints (wrist roll/pitch/yaw): orientation dominant, small position
 *
 * wristStartIdx: index of the first wrist joint in the chain (default 4 for 7-DoF arm)
 */
export function solveCCDIK(joints, endEffector, targetPos, targetQuat, iterations = 20, wristStartIdx = 4) {
  const n = joints.length

  for (let iter = 0; iter < iterations; iter++) {
    endEffector.getWorldPosition(_endPos)
    endEffector.getWorldQuaternion(_endQuat)

    if (_endPos.distanceTo(targetPos) < 0.002 && _endQuat.angleTo(targetQuat) < 0.03) return

    for (let i = n - 1; i >= 0; i--) {
      const joint = joints[i]
      if (!joint.setJointValue) continue

      const isWrist = i >= wristStartIdx
      const posW = isWrist ? 0.05 : 1.0
      const oriW = isWrist ? 1.0 : 0.0

      joint.getWorldQuaternion(_jointWorldQ)
      _worldAxis.copy(joint.axis).applyQuaternion(_jointWorldQ).normalize()

      let angle = 0

      if (posW > 0.001) {
        endEffector.getWorldPosition(_endPos)
        joint.getWorldPosition(_jointPos)
        _toEnd.subVectors(_endPos, _jointPos)
        _toTarget.subVectors(targetPos, _jointPos)
        const le = _toEnd.length()
        const lt = _toTarget.length()

        if (le > 1e-6 && lt > 1e-6) {
          _toEnd.divideScalar(le)
          _toTarget.divideScalar(lt)
          _cross.crossVectors(_toEnd, _toTarget)
          const sinA = _cross.length()
          const cosA = _toEnd.dot(_toTarget)
          if (sinA > 1e-6) {
            _cross.divideScalar(sinA)
            const proj = _cross.dot(_worldAxis)
            if (Math.abs(proj) > 0.01) {
              angle += Math.atan2(sinA, cosA) * Math.sign(proj) * posW
            }
          }
        }
      }

      if (oriW > 0.001 && targetQuat) {
        endEffector.getWorldQuaternion(_endQuat)
        _deltaQuat.copy(_endQuat).invert().premultiply(targetQuat).normalize()
        if (_deltaQuat.w < 0) { _deltaQuat.x *= -1; _deltaQuat.y *= -1; _deltaQuat.z *= -1; _deltaQuat.w *= -1 }

        const half = Math.acos(Math.min(1, _deltaQuat.w))
        if (half > 0.005) {
          const sa = Math.sin(half)
          _orientAxis.set(_deltaQuat.x / sa, _deltaQuat.y / sa, _deltaQuat.z / sa).normalize()
          const proj = _orientAxis.dot(_worldAxis)
          if (Math.abs(proj) > 0.01) {
            angle += half * 2 * Math.sign(proj) * oriW
          }
        }
      }

      const cur = joint.angle || 0
      angle -= cur * REGULARIZATION
      angle *= (1.0 - SMOOTH_COST)
      angle *= DAMPING

      if (Math.abs(angle) < 0.0001) continue
      angle = Math.max(-MAX_ANGLE_STEP, Math.min(MAX_ANGLE_STEP, angle))

      const lo = joint.limit?.lower ?? -Math.PI
      const hi = joint.limit?.upper ?? Math.PI
      joint.setJointValue(Math.max(lo, Math.min(hi, cur + angle)))
    }
  }
}
