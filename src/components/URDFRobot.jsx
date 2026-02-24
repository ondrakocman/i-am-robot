import { useRef, useEffect, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { solveCCDIK } from '../systems/CCDIK.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { ExponentialSmoother, QuaternionSmoother } from '../systems/ImpedanceControl.js'
import { XR_JOINT_NAMES } from '../constants/kinematics.js'

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)) }

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

  const smoothL = useRef({ pos: new ExponentialSmoother(0.25), quat: new QuaternionSmoother(0.25) })
  const smoothR = useRef({ pos: new ExponentialSmoother(0.25), quat: new QuaternionSmoother(0.25) })
  const retargetL = useRef(new RetargetingFilter(0.4))
  const retargetR = useRef(new RetargetingFilter(0.4))

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
      const rawData = retargetHand(xrJoints)
      const retarget = side === 'left' ? retargetL.current : retargetR.current
      const smoothed = retarget.update(rawData)
      applyFingerAngles(robot, side, smoothed)
    }
  })

  return <group ref={groupRef} />
}

// Map curl factor (0-1) to a URDF joint angle.
// Open = near 0, curled = toward whichever limit has larger magnitude.
function curlToAngle(joint, curl) {
  if (!joint?.limit) return 0
  const { lower, upper } = joint.limit
  if (Math.abs(lower) > Math.abs(upper)) return lower * curl
  return upper * curl
}

function applyFingerAngles(robot, side, data) {
  if (!robot.joints || !data) return
  const prefix = side + '_hand_'

  // ── Thumb j0: abduction (side-to-side) ──
  const thumbJ0 = robot.joints[prefix + 'thumb_0_joint']
  if (thumbJ0?.limit) {
    const abd = data.thumb.abduction
    // Positive abduction (spread) → positive angle for left, negative for right
    // because the URDF Y-axis thumb rotation is mirrored by geometry
    const sign = side === 'left' ? 1 : -1
    const range = Math.min(Math.abs(thumbJ0.limit.lower), Math.abs(thumbJ0.limit.upper))
    thumbJ0.setJointValue(clamp(sign * abd * range, thumbJ0.limit.lower, thumbJ0.limit.upper))
  }

  // ── Thumb j1, j2: curl ──
  const thumbJ1 = robot.joints[prefix + 'thumb_1_joint']
  const thumbJ2 = robot.joints[prefix + 'thumb_2_joint']
  if (thumbJ1) thumbJ1.setJointValue(curlToAngle(thumbJ1, data.thumb.curl[0]))
  if (thumbJ2) thumbJ2.setJointValue(curlToAngle(thumbJ2, data.thumb.curl[1]))

  // ── Index j0, j1: curl ──
  const indexJ0 = robot.joints[prefix + 'index_0_joint']
  const indexJ1 = robot.joints[prefix + 'index_1_joint']
  if (indexJ0) indexJ0.setJointValue(curlToAngle(indexJ0, data.index.curl[0]))
  if (indexJ1) indexJ1.setJointValue(curlToAngle(indexJ1, data.index.curl[1]))

  // ── Middle j0, j1: curl ──
  const middleJ0 = robot.joints[prefix + 'middle_0_joint']
  const middleJ1 = robot.joints[prefix + 'middle_1_joint']
  if (middleJ0) middleJ0.setJointValue(curlToAngle(middleJ0, data.middle.curl[0]))
  if (middleJ1) middleJ1.setJointValue(curlToAngle(middleJ1, data.middle.curl[1]))
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
