import { useRef, useEffect, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { solveCCDIK } from '../systems/CCDIK.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { ExponentialSmoother, QuaternionSmoother } from '../systems/ImpedanceControl.js'
import { XR_JOINT_NAMES } from '../constants/kinematics.js'

// ─── Robot placement ──────────────────────────────────────────────────────────
// Static robot: pelvis at world origin, standing on the floor.
// Pelvis Y = distance from floor to pelvis center. G1 legs are ~0.75m.
const PELVIS_HEIGHT = 0.75
const ROBOT_POSITION = new THREE.Vector3(0, PELVIS_HEIGHT, 0)

// Bright materials so the robot is clearly visible
const MAT_BODY = new THREE.MeshStandardMaterial({
  color: 0x4a4a6e,
  roughness: 0.4,
  metalness: 0.25,
})
const MAT_ACCENT = new THREE.MeshStandardMaterial({
  color: 0x6a6a9e,
  roughness: 0.35,
  metalness: 0.3,
})

// ─── Joint names ──────────────────────────────────────────────────────────────
const ARM_JOINTS = {
  left: [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
  ],
  right: [
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
  ],
}
const WRIST_JOINTS = {
  left:  ['left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint'],
  right: ['right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'],
}
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
const WRIST_LINK = { left: 'left_wrist_yaw_link', right: 'right_wrist_yaw_link' }

export function URDFRobot() {
  const { gl } = useThree()
  const groupRef = useRef()
  const [robot, setRobot] = useState(null)

  // Per-hand smoothing (simple exponential, no spring-damper — less jitter)
  const smoothL = useRef({ pos: new ExponentialSmoother(0.15), quat: new QuaternionSmoother(0.15) })
  const smoothR = useRef({ pos: new ExponentialSmoother(0.15), quat: new QuaternionSmoother(0.15) })
  const retargetL = useRef(new RetargetingFilter(0.2))
  const retargetR = useRef(new RetargetingFilter(0.2))

  // Load URDF
  useEffect(() => {
    const loader = new URDFLoader()
    const stlLoader = new STLLoader()
    const basePath = import.meta.env.BASE_URL + 'models/'

    loader.loadMeshCb = (path, _manager, onLoad) => {
      const meshPath = basePath + path
      stlLoader.load(meshPath, (geometry) => {
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

  // Add robot to scene — STATIC position, no camera following
  useEffect(() => {
    if (!robot || !groupRef.current) return

    // Z-up → Y-up
    robot.rotation.x = -Math.PI / 2

    groupRef.current.add(robot)
    groupRef.current.position.copy(ROBOT_POSITION)

    // Hide head (egocentric — camera replaces head)
    if (robot.links?.head_link) robot.links.head_link.visible = false

    return () => { groupRef.current?.remove(robot) }
  }, [robot])

  // ─── Frame loop ─────────────────────────────────────────────────────────────
  useFrame((state, delta, xrFrame) => {
    if (!robot || !xrFrame) return

    const session  = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return

    for (const source of session.inputSources) {
      if (!source.hand) continue

      const side = source.handedness // 'left' or 'right'
      if (side !== 'left' && side !== 'right') continue

      // Gather XR joint positions
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

      // Smooth the wrist position (reduces Quest 3 tracking noise)
      const sm = side === 'left' ? smoothL.current : smoothR.current
      const smoothPos = sm.pos.update(wristData.position)
      const smoothQuat = sm.quat.update(wristData.quaternion)

      // ── ARM IK ──────────────────────────────────────────────────────────
      // CCD IK needs the target in WORLD SPACE because it uses getWorldPosition() internally
      const armJoints = ARM_JOINTS[side].map(n => robot.joints?.[n]).filter(Boolean)
      const wristLink = robot.links?.[WRIST_LINK[side]]

      if (armJoints.length > 0 && wristLink) {
        solveCCDIK(armJoints, wristLink, smoothPos, 20)
      }

      // ── WRIST ORIENTATION ───────────────────────────────────────────────
      const wristJoints = WRIST_JOINTS[side].map(n => robot.joints?.[n]).filter(Boolean)
      for (const wj of wristJoints) {
        wj.setJointValue?.(0)
      }

      // ── FINGERS ─────────────────────────────────────────────────────────
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
