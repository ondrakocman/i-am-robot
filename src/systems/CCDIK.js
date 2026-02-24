import * as THREE from 'three'

// ─── CCD (Cyclic Coordinate Descent) IK ──────────────────────────────────────
// Operates entirely in world space.
// targetPos MUST be a world-space THREE.Vector3.
// joints: array of URDF joint objects (from shoulder toward wrist).
// endEffector: the URDF link whose world position should reach targetPos.

const _endPos    = new THREE.Vector3()
const _jointPos  = new THREE.Vector3()
const _toEnd     = new THREE.Vector3()
const _toTarget  = new THREE.Vector3()
const _cross     = new THREE.Vector3()
const _localAxis = new THREE.Vector3()
const _parentQ   = new THREE.Quaternion()

export function solveCCDIK(joints, endEffector, targetPos, iterations = 20) {
  for (let iter = 0; iter < iterations; iter++) {
    // Check convergence
    endEffector.getWorldPosition(_endPos)
    if (_endPos.distanceToSquared(targetPos) < 0.0001) return // within 1cm

    for (let i = joints.length - 1; i >= 0; i--) {
      const joint = joints[i]
      if (!joint.setJointValue) continue

      // Refresh positions each step (they change as we adjust joints)
      endEffector.getWorldPosition(_endPos)
      joint.getWorldPosition(_jointPos)

      _toEnd.subVectors(_endPos, _jointPos)
      _toTarget.subVectors(targetPos, _jointPos)

      const lenEnd = _toEnd.length()
      const lenTarget = _toTarget.length()
      if (lenEnd < 1e-6 || lenTarget < 1e-6) continue

      _toEnd.divideScalar(lenEnd)
      _toTarget.divideScalar(lenTarget)

      // Desired rotation axis in world space
      _cross.crossVectors(_toEnd, _toTarget)
      const sinAngle = _cross.length()
      const cosAngle = _toEnd.dot(_toTarget)

      if (sinAngle < 1e-6) continue

      _cross.divideScalar(sinAngle)

      // Convert world rotation axis to the joint's parent frame
      // (because joint.axis is defined in its parent's local frame)
      joint.parent.getWorldQuaternion(_parentQ)
      _parentQ.invert()
      _localAxis.copy(_cross).applyQuaternion(_parentQ)

      // Project onto the joint's single rotation axis (revolute joint constraint)
      const projection = _localAxis.dot(joint.axis)
      if (Math.abs(projection) < 0.01) continue

      // Compute the angle change along this joint axis
      let angle = Math.atan2(sinAngle, cosAngle) * Math.sign(projection)

      // Damping: reduce step size to prevent oscillation
      angle *= 0.6

      // Apply
      const newAngle = (joint.angle || 0) + angle
      const clamped = clamp(newAngle, joint.limit?.lower ?? -Math.PI, joint.limit?.upper ?? Math.PI)
      joint.setJointValue(clamped)
    }
  }
}

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}
