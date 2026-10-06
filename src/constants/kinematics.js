import * as THREE from 'three'

export const XR_JOINT_NAMES = [
  'wrist',
  'thumb-metacarpal',
  'thumb-phalanx-proximal',
  'thumb-phalanx-distal',
  'thumb-tip',
  'index-finger-metacarpal',
  'index-finger-phalanx-proximal',
  'index-finger-phalanx-intermediate',
  'index-finger-phalanx-distal',
  'index-finger-tip',
  'middle-finger-metacarpal',
  'middle-finger-phalanx-proximal',
  'middle-finger-phalanx-intermediate',
  'middle-finger-phalanx-distal',
  'middle-finger-tip',
  'ring-finger-metacarpal',
  'ring-finger-phalanx-proximal',
  'ring-finger-phalanx-intermediate',
  'ring-finger-phalanx-distal',
  'ring-finger-tip',
  'pinky-finger-metacarpal',
  'pinky-finger-phalanx-proximal',
  'pinky-finger-phalanx-intermediate',
  'pinky-finger-phalanx-distal',
  'pinky-finger-tip',
]

// Robot frame (URDF/MJCF: x forward, z up) -> three.js world (y up); the robot faces -z
export const ROBOT_BASE_QUAT = new THREE.Quaternion().multiplyQuaternions(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2),
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2),
)

// Frame correction: WebXR wrist has -Z=fingers, +Y=back-of-hand.
// URDF palm has +X=fingers. Left palm faces -Y, right palm faces +Y
// (confirmed by mirrored finger curl limits in the URDF).
// Left:  Ry(π/2) aligns fingers (-Z→+X) and keeps Y axis.
// Right: Rz(π)·Ry(π/2) also flips the palm normal axis.
export const XR_TO_URDF_L = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(0, 1, 0), Math.PI / 2
)
export const XR_TO_URDF_R = new THREE.Quaternion().multiplyQuaternions(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI),
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2),
)
