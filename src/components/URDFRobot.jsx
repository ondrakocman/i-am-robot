import { useRef, useEffect, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { solveCCDIK } from '../systems/CCDIK.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { ExponentialSmoother, QuaternionSmoother } from '../systems/ImpedanceControl.js'
import { XR_JOINT_NAMES } from '../constants/kinematics.js'

const MAT_BODY = new THREE.MeshStandardMaterial({ color: 0x4a4a6e, roughness: 0.4, metalness: 0.25 })
const MAT_ACCENT = new THREE.MeshStandardMaterial({ color: 0x6a6a9e, roughness: 0.35, metalness: 0.3 })

// Full 7-DoF IK chain: shoulder(3) + elbow(1) + wrist(3)
const ARM_CHAIN = {
  left: [
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
  ],
  right: [
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
  ],
}
const HAND_LINK = { left: 'left_hand_palm_link', right: 'right_hand_palm_link' }

const FINGER_JOINTS = {
  left: {
    thumb:  ['left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint'],
    index:  ['left_hand_index_0_joint', 'left_hand_index_1_joint'],
    middle: ['left_hand_middle_0_joint', 'left_hand_middle_1_joint'],
  },
  right: {
    thumb:  ['right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint'],
    index:  ['right_hand_index_0_joint', 'right_hand_index_1_joint'],
    middle: ['right_hand_middle_0_joint', 'right_hand_middle_1_joint'],
  },
}

// Self-collision prevention: tighten shoulder roll limits so arms can't cross body
const COLLISION_OVERRIDES = {
  left_shoulder_roll_joint:  { lower: -0.3 },    // prevent left arm swinging right into torso
  right_shoulder_roll_joint: { upper:  0.3 },     // prevent right arm swinging left into torso
  left_shoulder_yaw_joint:   { lower: -1.8, upper: 1.8 }, // limit arm reach behind body
  right_shoulder_yaw_joint:  { lower: -1.8, upper: 1.8 },
  left_elbow_joint:          { lower: 0.05 },     // elbow can't hyperextend
  right_elbow_joint:         { lower: 0.05 },
}

const _headWorldPos = new THREE.Vector3()
const _wristPos = new THREE.Vector3()
const _wristQuat = new THREE.Quaternion()

export function URDFRobot() {
  const { gl, camera } = useThree()
  const groupRef = useRef()
  const [robot, setRobot] = useState(null)
  const calibrated = useRef(false)

  const smoothL = useRef({ pos: new ExponentialSmoother(0.2), quat: new QuaternionSmoother(0.2) })
  const smoothR = useRef({ pos: new ExponentialSmoother(0.2), quat: new QuaternionSmoother(0.2) })
  const retargetL = useRef(new RetargetingFilter(0.25))
  const retargetR = useRef(new RetargetingFilter(0.25))

  // Load URDF
  useEffect(() => {
    const loader = new URDFLoader()
    const stlLoader = new STLLoader()
    const basePath = import.meta.env.BASE_URL + 'models/'

    loader.loadMeshCb = (path, _manager, onLoad) => {
      stlLoader.load(basePath + path, (geometry) => {
        geometry.computeVertexNormals()
        const isAccent = path.includes('contour') || path.includes('shoulder_roll') ||
                         path.includes('shoulder_pitch') || path.includes('waist') ||
                         path.includes('logo')
        const mesh = new THREE.Mesh(geometry, (isAccent ? MAT_ACCENT : MAT_BODY).clone())
        const group = new THREE.Group()
        group.add(mesh)
        onLoad(group)
      }, undefined, () => onLoad(new THREE.Group()))
    }

    fetch(basePath + 'g1.urdf')
      .then(r => r.text())
      .then(urdfText => setRobot(loader.parse(urdfText)))
      .catch(err => console.error('URDF load failed:', err))
  }, [])

  // Add robot, apply collision limit overrides
  useEffect(() => {
    if (!robot || !groupRef.current) return

    robot.rotation.x = -Math.PI / 2
    groupRef.current.add(robot)

    // Default standing position (will be overridden on VR entry calibration)
    groupRef.current.position.set(0, 0.75, 0)

    if (robot.links?.head_link) robot.links.head_link.visible = false

    // Override joint limits for self-collision prevention
    for (const [jointName, overrides] of Object.entries(COLLISION_OVERRIDES)) {
      const joint = robot.joints?.[jointName]
      if (joint?.limit) {
        if (overrides.lower !== undefined) joint.limit.lower = Math.max(joint.limit.lower, overrides.lower)
        if (overrides.upper !== undefined) joint.limit.upper = Math.min(joint.limit.upper, overrides.upper)
      }
    }

    calibrated.current = false

    return () => { groupRef.current?.remove(robot) }
  }, [robot])

  useFrame((state, delta, xrFrame) => {
    if (!robot || !groupRef.current) return

    // ─── Camera calibration on VR entry ───────────────────────────────────
    // On the first XR frame, position the robot so its head_link matches
    // the user's head position, facing forward (-Z in Three.js).
    if (xrFrame && !calibrated.current) {
      const headLink = robot.links?.head_link
      if (headLink) {
        // Get where the head currently is in world space
        headLink.getWorldPosition(_headWorldPos)

        // Offset the group so head_link ends up at the camera position
        const offset = new THREE.Vector3().copy(camera.position).sub(_headWorldPos)
        groupRef.current.position.add(offset)

        // Face the robot forward: align robot's forward (-Z in URDF → +X in URDF)
        // with the user's forward direction (camera -Z)
        const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
        groupRef.current.rotation.y = euler.y

        // Re-calibrate offset after rotation
        headLink.getWorldPosition(_headWorldPos)
        const offset2 = new THREE.Vector3().copy(camera.position).sub(_headWorldPos)
        groupRef.current.position.add(offset2)

        calibrated.current = true
      }
    }

    if (!xrFrame) return
    const session = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return

    // ─── Process each hand ────────────────────────────────────────────────
    for (const source of session.inputSources) {
      if (!source.hand) continue
      const side = source.handedness
      if (side !== 'left' && side !== 'right') continue

      const xrJoints = {}
      for (const name of XR_JOINT_NAMES) {
        const space = source.hand.get(name)
        if (!space) continue
        const pose = xrFrame.getJointPose(space, refSpace)
        if (!pose) continue
        const p = pose.transform.position
        const o = pose.transform.orientation
        xrJoints[name] = {
          position: new THREE.Vector3(p.x, p.y, p.z),
          quaternion: new THREE.Quaternion(o.x, o.y, o.z, o.w),
        }
      }

      const wristData = xrJoints['wrist']
      if (!wristData) continue

      // Smooth tracking data
      const sm = side === 'left' ? smoothL.current : smoothR.current
      const smoothPos = sm.pos.update(wristData.position)
      const smoothQuat = sm.quat.update(wristData.quaternion)

      _wristPos.copy(smoothPos)
      _wristQuat.copy(smoothQuat)

      // ── Full 7-DoF IK with position + orientation ──────────────────────
      const chainJoints = ARM_CHAIN[side].map(n => robot.joints?.[n]).filter(Boolean)
      const endLink = robot.links?.[HAND_LINK[side]]

      if (chainJoints.length > 0 && endLink) {
        solveCCDIK(chainJoints, endLink, _wristPos, _wristQuat, 25)
      }

      // ── Fingers ─────────────────────────────────────────────────────────
      const rawAngles = retargetHand(xrJoints)
      const retarget = side === 'left' ? retargetL.current : retargetR.current
      const angles = retarget.update(rawAngles)
      setFingerAngles(robot, FINGER_JOINTS[side], angles)
    }
  })

  return <group ref={groupRef} />
}

function setFingerAngles(robot, map, angles) {
  if (!robot.joints || !angles) return
  for (const [finger, jointNames] of Object.entries(map)) {
    const vals = angles[finger]
    if (!vals) continue
    jointNames.forEach((name, i) => {
      if (i < vals.length) robot.joints[name]?.setJointValue?.(vals[i])
    })
  }
}

export function TrackingHUD() {
  const ref = useRef()
  const { gl } = useThree()
  useFrame((_, __, xrFrame) => {
    if (!xrFrame || !ref.current) return
    const session = gl.xr.getSession()
    if (!session) return
    let l = false, r = false
    for (const src of session.inputSources) {
      if (src.hand) {
        if (src.handedness === 'left') l = true
        if (src.handedness === 'right') r = true
      }
    }
    ref.current.material?.color.set((l && r) ? '#00ff88' : (l || r) ? '#ffaa00' : '#ff4444')
  })
  return (
    <mesh ref={ref} position={[0.28, -0.25, -0.5]}>
      <sphereGeometry args={[0.005, 6, 6]} />
      <meshBasicMaterial color="#ff4444" />
    </mesh>
  )
}
