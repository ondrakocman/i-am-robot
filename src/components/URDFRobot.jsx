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

// URDF Z-up → Three.js Y-up, facing -Z (Three.js forward)
const ROBOT_BASE_QUAT = new THREE.Quaternion()
;(() => {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2)
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2)
  ROBOT_BASE_QUAT.multiplyQuaternions(qy, qx)
})()

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

const COLLISION_OVERRIDES = {
  left_shoulder_roll_joint:  { lower: -0.3 },
  right_shoulder_roll_joint: { upper:  0.3 },
  left_shoulder_yaw_joint:   { lower: -1.8, upper: 1.8 },
  right_shoulder_yaw_joint:  { lower: -1.8, upper: 1.8 },
  left_elbow_joint:          { lower: 0.05 },
  right_elbow_joint:         { lower: 0.05 },
}

// The mid360 lidar sits at the top of the robot — best proxy for "eye level"
const EYE_LINK = 'mid360_link'
const EYE_LINK_FALLBACK = 'head_link'

const _eyeWorldPos = new THREE.Vector3()
const _wristPos = new THREE.Vector3()
const _wristQuat = new THREE.Quaternion()
const _footPos = new THREE.Vector3()

export function URDFRobot({ vrMode = 'unlocked' }) {
  const { gl, camera } = useThree()
  const groupRef = useRef()
  const [robot, setRobot] = useState(null)
  const calibrated = useRef(false)
  const modeRef = useRef(vrMode)
  modeRef.current = vrMode

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

  // Setup robot in scene
  useEffect(() => {
    if (!robot || !groupRef.current) return

    robot.quaternion.copy(ROBOT_BASE_QUAT)
    groupRef.current.add(robot)

    // Compute standing height: position group so feet touch y=0
    groupRef.current.position.set(0, 0, 0)
    groupRef.current.updateMatrixWorld(true)

    const footLink = robot.links?.left_ankle_roll_link
    if (footLink) {
      footLink.getWorldPosition(_footPos)
      groupRef.current.position.y = -_footPos.y + 0.015
    } else {
      groupRef.current.position.y = 0.75
    }

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

  // Head visibility depends on mode
  useEffect(() => {
    if (!robot?.links?.head_link) return
    robot.links.head_link.visible = vrMode !== 'locked'
  }, [robot, vrMode])

  useFrame((state, delta, xrFrame) => {
    if (!robot || !groupRef.current) return

    const mode = modeRef.current
    const eyeLink = robot.links?.[EYE_LINK] || robot.links?.[EYE_LINK_FALLBACK]

    if (xrFrame && eyeLink) {
      if (mode === 'locked') {
        // ── Locked mode: robot follows camera every frame ────────────
        const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
        groupRef.current.rotation.y = euler.y

        groupRef.current.updateMatrixWorld(true)
        eyeLink.getWorldPosition(_eyeWorldPos)
        groupRef.current.position.x += camera.position.x - _eyeWorldPos.x
        groupRef.current.position.y += camera.position.y - _eyeWorldPos.y
        groupRef.current.position.z += camera.position.z - _eyeWorldPos.z

      } else if (!calibrated.current) {
        // ── Unlocked mode: one-time calibration ─────────────────────
        // Align robot XZ with camera, keep Y for floor contact
        const savedY = groupRef.current.position.y

        const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
        groupRef.current.rotation.y = euler.y

        groupRef.current.updateMatrixWorld(true)
        eyeLink.getWorldPosition(_eyeWorldPos)

        groupRef.current.position.x += camera.position.x - _eyeWorldPos.x
        groupRef.current.position.z += camera.position.z - _eyeWorldPos.z
        groupRef.current.position.y = savedY

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

      const sm = side === 'left' ? smoothL.current : smoothR.current
      const smoothPos = sm.pos.update(wristData.position)
      const smoothQuat = sm.quat.update(wristData.quaternion)

      _wristPos.copy(smoothPos)
      _wristQuat.copy(smoothQuat)

      // ── Arm IK ─────────────────────────────────────────────────────
      const chainJoints = ARM_CHAIN[side].map(n => robot.joints?.[n]).filter(Boolean)
      const endLink = robot.links?.[HAND_LINK[side]]

      if (chainJoints.length > 0 && endLink) {
        solveCCDIK(chainJoints, endLink, _wristPos, _wristQuat, 25)
      }

      // ── Fingers ────────────────────────────────────────────────────
      const rawData = retargetHand(xrJoints)
      const retarget = side === 'left' ? retargetL.current : retargetR.current
      const smoothed = retarget.update(rawData)
      applyFingerAngles(robot, side, smoothed)
    }
  })

  return <group ref={groupRef} />
}

function curlToAngle(joint, curl) {
  if (!joint?.limit) return 0
  const { lower, upper } = joint.limit
  if (Math.abs(lower) > Math.abs(upper)) return lower * curl
  return upper * curl
}

function applyFingerAngles(robot, side, data) {
  if (!robot.joints || !data) return
  const prefix = side + '_hand_'

  const thumbJ0 = robot.joints[prefix + 'thumb_0_joint']
  if (thumbJ0?.limit) {
    const abd = data.thumb.abduction
    const sign = side === 'left' ? 1 : -1
    const range = Math.min(Math.abs(thumbJ0.limit.lower), Math.abs(thumbJ0.limit.upper))
    thumbJ0.setJointValue(clamp(sign * abd * range, thumbJ0.limit.lower, thumbJ0.limit.upper))
  }

  const thumbJ1 = robot.joints[prefix + 'thumb_1_joint']
  const thumbJ2 = robot.joints[prefix + 'thumb_2_joint']
  if (thumbJ1) thumbJ1.setJointValue(curlToAngle(thumbJ1, data.thumb.curl[0]))
  if (thumbJ2) thumbJ2.setJointValue(curlToAngle(thumbJ2, data.thumb.curl[1]))

  const indexJ0 = robot.joints[prefix + 'index_0_joint']
  const indexJ1 = robot.joints[prefix + 'index_1_joint']
  if (indexJ0) indexJ0.setJointValue(curlToAngle(indexJ0, data.index.curl[0]))
  if (indexJ1) indexJ1.setJointValue(curlToAngle(indexJ1, data.index.curl[1]))

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
