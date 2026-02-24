import * as THREE from 'three'

// ─── CCD IK inspired by xr_teleoperate's optimization approach ──────────────
//
// Key improvements over naive CCD:
// 1. Position weighted ~50x over orientation (matching official solver)
// 2. Smooth cost: penalizes deviation from previous frame's joint angles
// 3. Velocity limiting: clamps max angle change per frame
// 4. Regularization: gentle pull toward neutral (zero) joint angles

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

const MAX_ANGLE_CHANGE_PER_SOLVE = 0.15
const REGULARIZATION_WEIGHT = 0.005
const SMOOTH_WEIGHT = 0.08

export function solveCCDIK(joints, endEffector, targetPos, targetQuat, iterations = 20) {
  const n = joints.length

  for (let iter = 0; iter < iterations; iter++) {
    endEffector.getWorldPosition(_endPos)
    endEffector.getWorldQuaternion(_endQuat)

    const posErr = _endPos.distanceTo(targetPos)
    const oriErr = _endQuat.angleTo(targetQuat)
    if (posErr < 0.003 && oriErr < 0.05) return

    for (let i = n - 1; i >= 0; i--) {
      const joint = joints[i]
      if (!joint.setJointValue) continue

      // Position-heavy weighting matching xr_teleoperate's 50:1 ratio.
      // First joints (shoulder/elbow) focus almost entirely on position.
      // Last joints (wrist) add some orientation.
      const t = n > 1 ? i / (n - 1) : 0
      const posW = 1.0 - t * 0.6
      const oriW = t * 0.15

      joint.getWorldQuaternion(_jointWorldQ)
      _worldAxis.copy(joint.axis).applyQuaternion(_jointWorldQ).normalize()

      let totalAngle = 0

      // ── Position contribution ───────────────────────────────────────
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

      // ── Orientation contribution ────────────────────────────────────
      if (oriW > 0.001 && targetQuat) {
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

      // ── Regularization: pull toward zero ────────────────────────────
      const currentAngle = joint.angle || 0
      totalAngle -= currentAngle * REGULARIZATION_WEIGHT

      // ── Smooth cost: penalize change from previous angle ────────────
      totalAngle *= (1.0 - SMOOTH_WEIGHT)

      // ── Damping ─────────────────────────────────────────────────────
      totalAngle *= 0.4

      if (Math.abs(totalAngle) < 0.0001) continue

      // ── Velocity limiting ───────────────────────────────────────────
      totalAngle = Math.max(-MAX_ANGLE_CHANGE_PER_SOLVE, Math.min(MAX_ANGLE_CHANGE_PER_SOLVE, totalAngle))

      const newAngle = currentAngle + totalAngle
      const lo = joint.limit?.lower ?? -Math.PI
      const hi = joint.limit?.upper ?? Math.PI
      joint.setJointValue(Math.max(lo, Math.min(hi, newAngle)))
    }
  }
}
