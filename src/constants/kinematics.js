// Unitree G1 + Dex 3.1 kinematic constants
// Based on hardware specifications from official documentation

export const DEG = Math.PI / 180

// ─── G1 Arm Joint Limits ──────────────────────────────────────────────────────
// Each joint: { axis (local), min (rad), max (rad), restAngle (rad) }
// Joint order follows DH convention from shoulder outward
export const G1_ARM_JOINTS = {
  left: [
    { name: 'shoulder_pitch', axis: [0, 0, 1], min: -154 * DEG, max: 154 * DEG,  rest: 0 },
    { name: 'shoulder_roll',  axis: [1, 0, 0], min:  -91 * DEG, max: 129 * DEG,  rest: 20 * DEG },
    { name: 'shoulder_yaw',   axis: [0, 1, 0], min: -150 * DEG, max: 150 * DEG,  rest: 0 },
    { name: 'elbow',          axis: [0, 0, 1], min:    0 * DEG, max: 165 * DEG,  rest: 60 * DEG },
    { name: 'wrist_roll',     axis: [0, 1, 0], min: -180 * DEG, max: 180 * DEG,  rest: 0 },
    { name: 'wrist_pitch',    axis: [0, 0, 1], min: -92.5 * DEG, max: 92.5 * DEG, rest: 0 },
    { name: 'wrist_yaw',      axis: [1, 0, 0], min: -92.5 * DEG, max: 92.5 * DEG, rest: 0 },
  ],
  right: [
    { name: 'shoulder_pitch', axis: [0, 0, 1], min: -154 * DEG, max: 154 * DEG,  rest: 0 },
    { name: 'shoulder_roll',  axis: [1, 0, 0], min: -129 * DEG, max:  91 * DEG,  rest: -20 * DEG },
    { name: 'shoulder_yaw',   axis: [0, 1, 0], min: -150 * DEG, max: 150 * DEG,  rest: 0 },
    { name: 'elbow',          axis: [0, 0, 1], min:    0 * DEG, max: 165 * DEG,  rest: 60 * DEG },
    { name: 'wrist_roll',     axis: [0, 1, 0], min: -180 * DEG, max: 180 * DEG,  rest: 0 },
    { name: 'wrist_pitch',    axis: [0, 0, 1], min: -92.5 * DEG, max: 92.5 * DEG, rest: 0 },
    { name: 'wrist_yaw',      axis: [1, 0, 0], min: -92.5 * DEG, max: 92.5 * DEG, rest: 0 },
  ],
}

// ─── Dex 3.1 Hand Joint Limits ───────────────────────────────────────────────
// 7 DoF total: thumb(3) + index(2) + middle(2)
export const DEX31_JOINTS = {
  thumb: [
    { name: 'thumb_j0', min: -60 * DEG,  max:  60 * DEG, rest: 0 },   // abduction/adduction
    { name: 'thumb_j1', min: -35 * DEG,  max:  60 * DEG, rest: 0 },   // proximal flexion
    { name: 'thumb_j2', min:   0 * DEG,  max: 100 * DEG, rest: 0 },   // distal flexion (gear)
  ],
  index: [
    { name: 'index_j0', min: 0 * DEG, max:  90 * DEG, rest: 0 },      // proximal flexion
    { name: 'index_j1', min: 0 * DEG, max: 100 * DEG, rest: 0 },      // distal flexion
  ],
  middle: [
    { name: 'middle_j0', min: 0 * DEG, max:  90 * DEG, rest: 0 },
    { name: 'middle_j1', min: 0 * DEG, max: 100 * DEG, rest: 0 },
  ],
}

// ─── G1 Body Dimensions (meters) ────────────────────────────────────────────
export const G1 = {
  totalHeight:       1.32,
  eyeHeight:         1.24,   // camera/eye level when standing
  headHeight:        0.14,   // head mesh half-height
  neckLength:        0.08,
  torsoLength:       0.38,
  torsoWidth:        0.28,
  torsoDepth:        0.17,
  pelvisLength:      0.14,
  pelvisWidth:       0.20,

  // Shoulder attachment offset from neck base (torso top center)
  leftShoulderOffset:  [ 0.172,  0.0, -0.01],
  rightShoulderOffset: [-0.172,  0.0, -0.01],

  // Link lengths for arm IK chain
  upperArmLength:    0.250,
  forearmLength:     0.218,
  wristLength:       0.065,

  // Max reach from shoulder = upperArm + forearm + wrist
  maxReach:          0.533,

  // Leg dimensions
  thighLength:       0.32,
  shinLength:        0.30,
  hipWidth:          0.10,   // half-width, hip offset from center

  // Payload limits
  maxArmPayload:     2.0,    // kg
  maxFingerPayload:  0.5,    // kg (Dex 3.1 max grip)
}

// ─── Dex 3.1 Mesh Dimensions (meters) ───────────────────────────────────────
export const DEX31 = {
  palmWidth:   0.075,
  palmHeight:  0.095,
  palmDepth:   0.028,

  // Segment lengths [proximal, distal] in meters
  thumbSegments:  [0.042, 0.035, 0.024],
  fingerSegments: [0.040, 0.028],

  // Finger base offsets from palm center (local palm frame)
  thumbBase:  [ 0.030, -0.010,  0.014],  // side of palm
  indexBase:  [ 0.022,  0.050,  0.005],
  middleBase: [-0.008,  0.052,  0.005],

  // Fingertip radius for collision
  fingertipRadius: 0.009,
}

// ─── XR Hand Joint Names (WebXR spec order) ──────────────────────────────────
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

// Mapping: which Quest 3 joint names feed which Dex 3.1 joints
export const RETARGETING_MAP = {
  wrist:          'wrist',
  thumbProximal:  'thumb-phalanx-proximal',
  thumbDistal:    'thumb-phalanx-distal',
  thumbTip:       'thumb-tip',
  indexProximal:  'index-finger-phalanx-proximal',
  indexMid:       'index-finger-phalanx-intermediate',
  indexTip:       'index-finger-tip',
  middleProximal: 'middle-finger-phalanx-proximal',
  middleMid:      'middle-finger-phalanx-intermediate',
  middleTip:      'middle-finger-tip',
}
