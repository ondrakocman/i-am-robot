import * as THREE from 'three'

// ─── CCD (Cyclic Coordinate Descent) IK Solver ───────────────────────────────
// Works on any chain of URDF revolute joints.
// Iteratively rotates each joint to reduce end-effector error.
//
// Unlike analytical IK, this handles arbitrary axis configurations and
// joint limits natively — perfect for the G1's complex shoulder offset angles.

const _endWorld  = new THREE.Vector3()
const _jointWorld = new THREE.Vector3()
const _toEnd     = new THREE.Vector3()
const _toTarget  = new THREE.Vector3()
const _axis      = new THREE.Vector3()
const _localAxis = new THREE.Vector3()
const _cross     = new THREE.Vector3()
const _q         = new THREE.Quaternion()
const _invParent = new THREE.Quaternion()

export function solveCCDIK(joints, endEffector, targetPos, iterations = 12) {
  for (let iter = 0; iter < iterations; iter++) {
    for (let i = joints.length - 1; i >= 0; i--) {
      const joint = joints[i]

      endEffector.getWorldPosition(_endWorld)
      joint.getWorldPosition(_jointWorld)

      _toEnd.copy(_endWorld).sub(_jointWorld)
      _toTarget.copy(targetPos).sub(_jointWorld)

      if (_toEnd.lengthSq() < 1e-8 || _toTarget.lengthSq() < 1e-8) continue

      _toEnd.normalize()
      _toTarget.normalize()

      // Angle between current end-effector direction and target
      let dot = THREE.MathUtils.clamp(_toEnd.dot(_toTarget), -1, 1)
      let angle = Math.acos(dot)

      if (angle < 0.0001) continue

      // Cross product gives rotation axis in world space
      _cross.crossVectors(_toEnd, _toTarget)
      if (_cross.lengthSq() < 1e-10) continue
      _cross.normalize()

      // Convert world axis to joint's local space
      if (joint.parent) {
        _invParent.copy(joint.parent.getWorldQuaternion(new THREE.Quaternion())).invert()
        _localAxis.copy(_cross).applyQuaternion(_invParent)
      } else {
        _localAxis.copy(_cross)
      }

      // For URDF revolute joints, project onto the joint's rotation axis
      if (joint.axis) {
        const axisDot = _localAxis.dot(joint.axis)
        // Use the sign to determine rotation direction
        angle = angle * Math.sign(axisDot)
      }

      // Clamp step size to prevent overshooting
      angle = THREE.MathUtils.clamp(angle, -0.4, 0.4)

      // Apply rotation
      const currentAngle = (joint.angle || 0) + angle
      const clamped = clampJointAngle(joint, currentAngle)
      if (joint.setJointValue) {
        joint.setJointValue(clamped)
      }
    }
  }
}

function clampJointAngle(joint, angle) {
  if (joint.limit) {
    return THREE.MathUtils.clamp(angle, joint.limit.lower, joint.limit.upper)
  }
  return angle
}

// ─── Wrist orientation matching ──────────────────────────────────────────────
// After solving position IK for the arm, solve wrist joints to match
// the desired wrist orientation from hand tracking.

const _currentQ = new THREE.Quaternion()
const _desiredLocal = new THREE.Quaternion()

export function solveWristOrientation(wristJoints, endLink, targetQuat, iterations = 6) {
  for (let iter = 0; iter < iterations; iter++) {
    endLink.getWorldQuaternion(_currentQ)
    _desiredLocal.copy(_currentQ).invert().premultiply(targetQuat)

    // Decompose the remaining rotation into Euler angles matching wrist joint axes
    const euler = new THREE.Euler().setFromQuaternion(_desiredLocal)
    const angles = [euler.x, euler.y, euler.z]

    for (let i = 0; i < wristJoints.length && i < 3; i++) {
      const joint = wristJoints[i]
      if (!joint || !joint.setJointValue) continue
      const current = joint.angle || 0
      const delta = THREE.MathUtils.clamp(angles[i] * 0.3, -0.2, 0.2)
      const next = clampJointAngle(joint, current + delta)
      joint.setJointValue(next)
    }
  }
}
