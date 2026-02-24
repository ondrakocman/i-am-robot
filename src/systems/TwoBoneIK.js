import * as THREE from 'three'
import { G1 } from '../constants/kinematics.js'

// ─── Two-Bone Analytical IK ───────────────────────────────────────────────────
// Solves the classic two-bone chain: shoulder → elbow → wrist
//
// Given:
//   root:    shoulder world position (THREE.Vector3)
//   target:  desired wrist world position (THREE.Vector3)
//   pole:    pole vector (hint for elbow direction, world space)
//   L1:      upper arm length
//   L2:      forearm length
//
// Returns:
//   { elbowPos: THREE.Vector3, reachable: boolean }
//
// The pole vector pushes the elbow toward a natural position.
// For left arm:  pole points left and slightly back
// For right arm: pole points right and slightly back

const _rootToTarget = new THREE.Vector3()
const _rootToPole   = new THREE.Vector3()
const _perpAxis     = new THREE.Vector3()
const _elbowDir     = new THREE.Vector3()

export function solveTwoBoneIK(root, target, pole, L1, L2) {
  _rootToTarget.copy(target).sub(root)
  const dist = _rootToTarget.length()

  // Clamp target to max reach (avoid solver blow-up)
  const maxDist = (L1 + L2) * 0.999
  const minDist = Math.abs(L1 - L2) * 1.001
  const clampedDist = Math.max(minDist, Math.min(maxDist, dist))

  const reachable = dist <= maxDist

  // Law of cosines: angle at root joint (shoulder)
  // cos(A) = (L1² + d² - L2²) / (2·L1·d)
  const cosA = (L1 * L1 + clampedDist * clampedDist - L2 * L2) / (2 * L1 * clampedDist)
  const angleA = Math.acos(Math.max(-1, Math.min(1, cosA)))

  // Direction from root to target (unit vector)
  const dirToTarget = _rootToTarget.clone().normalize()
  if (dist < 0.001) {
    // Degenerate: target at root, use pole direction
    dirToTarget.copy(pole).sub(root).normalize()
  } else if (!reachable) {
    // Stretch toward target
    _rootToTarget.setLength(maxDist)
  }

  // Pole projection — elbow bends toward the pole vector
  _rootToPole.copy(pole).sub(root)
  // Gram-Schmidt: pole component perpendicular to root→target
  _perpAxis.copy(_rootToPole)
  _perpAxis.addScaledVector(dirToTarget, -_perpAxis.dot(dirToTarget))

  if (_perpAxis.lengthSq() < 0.0001) {
    // Pole is collinear with arm — build any perpendicular
    _perpAxis.set(0, 1, 0)
    if (Math.abs(dirToTarget.dot(_perpAxis)) > 0.9) {
      _perpAxis.set(1, 0, 0)
    }
    _perpAxis.cross(dirToTarget).normalize()
  } else {
    _perpAxis.normalize()
  }

  // Elbow direction = rotate dirToTarget by angleA toward perpAxis
  _elbowDir.copy(dirToTarget).multiplyScalar(Math.cos(angleA))
  _elbowDir.addScaledVector(_perpAxis, Math.sin(angleA))
  _elbowDir.normalize()

  const elbowPos = new THREE.Vector3().copy(root).addScaledVector(_elbowDir, L1)

  return { elbowPos, reachable, clampedTarget: root.clone().addScaledVector(dirToTarget, clampedDist) }
}

// ─── Pole vector helpers ──────────────────────────────────────────────────────
// Returns a world-space pole vector that puts elbows in a natural position.
// Elbows point outward and slightly back from the shoulder.

export function getLeftElbowPole(shoulderWorld) {
  // Left elbow naturally points left, slightly back, and slightly down
  return new THREE.Vector3(
    shoulderWorld.x + 0.6,
    shoulderWorld.y - 0.3,
    shoulderWorld.z - 0.3
  )
}

export function getRightElbowPole(shoulderWorld) {
  return new THREE.Vector3(
    shoulderWorld.x - 0.6,
    shoulderWorld.y - 0.3,
    shoulderWorld.z - 0.3
  )
}

// ─── Wrist offset from IK result ─────────────────────────────────────────────
// Because Quest 3 tracks the wrist joint (not the palm center),
// we offset the IK target slightly forward along wrist orientation.
const _offset = new THREE.Vector3()
export function applyWristOffset(wristPos, wristQuat, offsetDistance = 0.05) {
  _offset.set(0, 0, -offsetDistance).applyQuaternion(wristQuat)
  return wristPos.clone().add(_offset)
}

// ─── User arm scale calibration ──────────────────────────────────────────────
// Maps human arm length to G1 arm length to prevent singularities.
// Called once during calibration (arms outstretched to sides).
//
// humanReach: measured distance from shoulder to wrist when arm fully extended
// Returns a scale factor to apply to all wrist IK targets

export function computeArmScale(humanReach) {
  const robotReach = G1.upperArmLength + G1.forearmLength  // ~0.468m
  const scale = robotReach / Math.max(humanReach, 0.3)
  // Clamp scale to prevent extreme mappings
  return Math.max(0.5, Math.min(1.2, scale))
}
