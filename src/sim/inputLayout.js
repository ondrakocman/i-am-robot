// Layout of the per-frame operator input and raw tracking data, and the hand/body naming conventions the
// renderer and the simulation share. Dependency-free, so the main thread, the physics worker and Node tooling
// can all import it without pulling each other's modules in.

export const SIDES = ['left', 'right']

// Per-hand operator input: tracked, palm position (3), palm quaternion w,x,y,z (4), finger commands (7):
// thumb rotation in [-1, 1], then curls in [0, 1] for thumb_1, thumb_2, index_0, index_1, middle_0, middle_1.
// Positions/orientations are in the MuJoCo world frame (x forward, y left, z up).
const HAND_INPUT_NAMES = ['tracked', 'palm_x', 'palm_y', 'palm_z', 'palm_qw', 'palm_qx', 'palm_qy', 'palm_qz',
  'thumb_rotation', 'thumb_1', 'thumb_2', 'index_0', 'index_1', 'middle_0', 'middle_1']
export const HAND_INPUT = HAND_INPUT_NAMES.length
export const INPUT_SIZE = 2 * HAND_INPUT
export const INPUT_NAMES = SIDES.flatMap(s => HAND_INPUT_NAMES.map(n => `${s}_${n}`))

// The 25 WebXR hand joints, in the order they are recorded
export const XR_JOINT_NAMES = [
  'wrist',
  'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
  'index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip',
  'middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip',
  'ring-finger-metacarpal', 'ring-finger-phalanx-proximal', 'ring-finger-phalanx-intermediate', 'ring-finger-phalanx-distal', 'ring-finger-tip',
  'pinky-finger-metacarpal', 'pinky-finger-phalanx-proximal', 'pinky-finger-phalanx-intermediate', 'pinky-finger-phalanx-distal', 'pinky-finger-tip',
]

// Raw operator data, recorded for re-retargeting later: viewer (head) pose (pos 3 + quat wxyz 4), then for
// each hand the 25 WebXR joints (pos 3 + quat wxyz 4), all in the MuJoCo world frame. An untracked joint is
// all zeros (its quaternion has zero norm).
export const RAW_HAND = 7 * XR_JOINT_NAMES.length
export const RAW_SIZE = 7 + 2 * RAW_HAND
export const RAW_LAYOUT = {
  pose: ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'],
  order: ['head', ...SIDES.map(s => `${s}_hand`)],
  hand_joints: XR_JOINT_NAMES,
}

/** Which hand a MuJoCo body belongs to, from its name: 0 left, 1 right, -1 neither. */
export function handOfBodyName(name) {
  for (let i = 0; i < SIDES.length; i++) {
    if (name.startsWith(`${SIDES[i]}_hand_`) || name === `${SIDES[i]}_wrist_yaw_link`) return i
  }
  return -1
}
