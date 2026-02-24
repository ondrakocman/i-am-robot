import { useRef, useEffect, useState, useCallback } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader from 'urdf-loader'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { solveCCDIK } from '../systems/CCDIK.js'
import { HandImpedance } from '../systems/ImpedanceControl.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { XR_JOINT_NAMES } from '../constants/kinematics.js'

// ─── Constants ────────────────────────────────────────────────────────────────
// Head link Z offset from pelvis (traced through URDF chain)
// pelvis → waist_yaw(0) → waist_roll(0.035) → torso(0.019) → head(-0.054)
// Head link origin is at Z=0 in pelvis frame, but head mesh center of mass
// is at Z≈0.45 in head frame. The "eye level" from pelvis is ~0.45m.
// After Z-up → Y-up rotation this becomes Y offset.
const HEAD_Y_FROM_PELVIS = 0.45

// Material for robot body
const ROBOT_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x1a1a2e,
  roughness: 0.5,
  metalness: 0.3,
})
const ROBOT_MATERIAL_LIGHT = new THREE.MeshStandardMaterial({
  color: 0x2a2a4e,
  roughness: 0.45,
  metalness: 0.35,
})

// ─── Arm joint chain names (from URDF) ───────────────────────────────────────
const LEFT_ARM_JOINTS = [
  'left_shoulder_pitch_joint',
  'left_shoulder_roll_joint',
  'left_shoulder_yaw_joint',
  'left_elbow_joint',
]
const RIGHT_ARM_JOINTS = [
  'right_shoulder_pitch_joint',
  'right_shoulder_roll_joint',
  'right_shoulder_yaw_joint',
  'right_elbow_joint',
]
const LEFT_WRIST_JOINTS = [
  'left_wrist_roll_joint',
  'left_wrist_pitch_joint',
  'left_wrist_yaw_joint',
]
const RIGHT_WRIST_JOINTS = [
  'right_wrist_roll_joint',
  'right_wrist_pitch_joint',
  'right_wrist_yaw_joint',
]

// Dex 3.1 finger joint names in URDF
const LEFT_FINGER_JOINTS = {
  thumb:  ['left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint'],
  index:  ['left_hand_index_0_joint', 'left_hand_index_1_joint'],
  middle: ['left_hand_middle_0_joint', 'left_hand_middle_1_joint'],
}
const RIGHT_FINGER_JOINTS = {
  thumb:  ['right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint'],
  index:  ['right_hand_index_0_joint', 'right_hand_index_1_joint'],
  middle: ['right_hand_middle_0_joint', 'right_hand_middle_1_joint'],
}

// Links used for IK end-effectors
const LEFT_WRIST_LINK  = 'left_wrist_yaw_link'
const RIGHT_WRIST_LINK = 'right_wrist_yaw_link'

// ─── Temp vectors ─────────────────────────────────────────────────────────────
const _wristPos  = new THREE.Vector3()
const _wristQuat = new THREE.Quaternion()
const _yawQuat   = new THREE.Quaternion()
const _euler     = new THREE.Euler()
const _targetPos = new THREE.Vector3()

export function URDFRobot() {
  const { gl, camera } = useThree()
  const groupRef = useRef()
  const [robot, setRobot] = useState(null)

  // Per-hand state
  const impedanceL = useRef(new HandImpedance())
  const impedanceR = useRef(new HandImpedance())
  const retargetL  = useRef(new RetargetingFilter(0.18))
  const retargetR  = useRef(new RetargetingFilter(0.18))
  const hadHandL   = useRef(false)
  const hadHandR   = useRef(false)

  // Load URDF model
  useEffect(() => {
    const loader = new URDFLoader()
    const stlLoader = new STLLoader()
    const manager = new THREE.LoadingManager()

    const basePath = import.meta.env.BASE_URL + 'models/'

    loader.loadMeshCb = (path, _manager, onLoad) => {
      // Resolve mesh path relative to models directory
      const meshPath = basePath + path
      stlLoader.load(meshPath, (geometry) => {
        geometry.computeVertexNormals()
        const mat = path.includes('pelvis_contour') || path.includes('shoulder_roll') ||
                    path.includes('shoulder_pitch') || path.includes('waist') ||
                    path.includes('roll_link') || path.includes('logo')
          ? ROBOT_MATERIAL_LIGHT.clone()
          : ROBOT_MATERIAL.clone()
        const mesh = new THREE.Mesh(geometry, mat)
        mesh.castShadow = false
        mesh.receiveShadow = false
        const group = new THREE.Group()
        group.add(mesh)
        onLoad(group)
      }, undefined, (err) => {
        console.warn('Mesh load error:', path, err)
        onLoad(new THREE.Group())
      })
    }

    const urdfPath = basePath + 'g1.urdf'
    fetch(urdfPath)
      .then(r => r.text())
      .then(urdfText => {
        const model = loader.parse(urdfText)
        setRobot(model)
      })
      .catch(err => console.error('URDF load failed:', err))
  }, [])

  // Add robot to scene group
  useEffect(() => {
    if (!robot || !groupRef.current) return
    // Rotate from URDF Z-up to Three.js Y-up
    robot.rotation.x = -Math.PI / 2
    groupRef.current.add(robot)

    // Hide head mesh (camera IS the head)
    const headLink = robot.links?.head_link
    if (headLink) headLink.visible = false
    // Also hide d435, mid360, IMU sensors (no mesh, just empties — but just in case)

    return () => {
      groupRef.current?.remove(robot)
    }
  }, [robot])

  // ─── Frame loop: position robot + IK + hand tracking ─────────────────────
  useFrame((state, delta, xrFrame) => {
    if (!robot || !groupRef.current) return

    // Position robot: pelvis below camera so head aligns with eye level
    // Camera Y = eye level, pelvis Y = camera.y - HEAD_Y_FROM_PELVIS
    _euler.setFromQuaternion(camera.quaternion, 'YXZ')
    _yawQuat.setFromEuler(new THREE.Euler(0, _euler.y, 0))

    groupRef.current.position.copy(camera.position)
    groupRef.current.position.y -= HEAD_Y_FROM_PELVIS
    groupRef.current.quaternion.copy(_yawQuat)

    // ── Hand tracking ──────────────────────────────────────────────────────
    if (!xrFrame) return

    const session  = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return

    let foundLeft = false, foundRight = false

    for (const source of session.inputSources) {
      if (!source.hand) continue

      // FIX: WebXR 'left' hand is the user's left hand.
      // In the previous version these were swapped.
      const isLeft = source.handedness === 'left'
      foundLeft  = foundLeft  || isLeft
      foundRight = foundRight || !isLeft

      // Extract all joint poses
      const joints = {}
      for (const name of XR_JOINT_NAMES) {
        const space = source.hand.get(name)
        if (!space) continue
        const pose = xrFrame.getJointPose(space, refSpace)
        if (!pose) continue
        const p = pose.transform.position
        const o = pose.transform.orientation
        joints[name] = {
          position: new THREE.Vector3(p.x, p.y, p.z),
          quaternion: new THREE.Quaternion(o.x, o.y, o.z, o.w),
        }
      }

      const wristJoint = joints['wrist']
      if (!wristJoint) continue

      _wristPos.copy(wristJoint.position)
      _wristQuat.copy(wristJoint.quaternion)

      const impedance = isLeft ? impedanceL.current : impedanceR.current
      const retarget  = isLeft ? retargetL.current  : retargetR.current
      const hadHand   = isLeft ? hadHandL : hadHandR

      if (!hadHand.current) {
        impedance.reset()
        impedance.wristPos.reset(_wristPos)
        impedance.wristQuat.reset(_wristQuat)
      }
      hadHand.current = true

      const { position: smoothPos, quaternion: smoothQuat } = impedance.update(
        _wristPos, _wristQuat, Math.min(delta, 0.05)
      )

      // ── Arm IK ────────────────────────────────────────────────────────────
      // Convert wrist world position to robot local frame for CCD IK
      _targetPos.copy(smoothPos)
      groupRef.current.worldToLocal(_targetPos)
      // Also need to undo the Z-up rotation on the target
      // groupRef rotates the robot by -90° on X, so local space has that transform
      // The target is already in the group's local space after worldToLocal

      const armJointNames   = isLeft ? LEFT_ARM_JOINTS   : RIGHT_ARM_JOINTS
      const wristJointNames = isLeft ? LEFT_WRIST_JOINTS  : RIGHT_WRIST_JOINTS
      const wristLinkName   = isLeft ? LEFT_WRIST_LINK    : RIGHT_WRIST_LINK
      const fingerJointMap  = isLeft ? LEFT_FINGER_JOINTS  : RIGHT_FINGER_JOINTS

      // Gather URDF joint objects for IK chain
      const armJoints = armJointNames
        .map(n => robot.joints?.[n])
        .filter(Boolean)

      const wristLink = robot.links?.[wristLinkName]

      if (armJoints.length > 0 && wristLink) {
        // Convert target from groupRef local space to robot local space
        // robot has rotation.x = -PI/2 (Z-up→Y-up), so we invert that
        const robotLocalTarget = _targetPos.clone()
        const invRobotQ = new THREE.Quaternion().setFromEuler(robot.rotation).invert()
        robotLocalTarget.applyQuaternion(invRobotQ)

        solveCCDIK(armJoints, wristLink, robotLocalTarget, 15)
      }

      // ── Wrist orientation (simplified) ──────────────────────────────────
      const wristJointsURDF = wristJointNames
        .map(n => robot.joints?.[n])
        .filter(Boolean)

      // For now, just reset wrist joints to zero — orientation IK is complex
      // and the position IK gives us the main tracking behavior
      for (const wj of wristJointsURDF) {
        if (wj.setJointValue) wj.setJointValue(0)
      }

      // ── Finger retargeting ──────────────────────────────────────────────
      const rawAngles    = retargetHand(joints)
      const smoothAngles = retarget.update(rawAngles)

      // Apply to URDF finger joints
      setFingerAngles(robot, fingerJointMap, smoothAngles)
    }

    // Reset impedance when hands disappear
    if (!foundLeft  && hadHandL.current) { impedanceL.current.reset(); hadHandL.current = false }
    if (!foundRight && hadHandR.current) { impedanceR.current.reset(); hadHandR.current = false }
  })

  return <group ref={groupRef} />
}

function setFingerAngles(robot, fingerJointMap, angles) {
  if (!robot.joints || !angles) return

  // Thumb: 3 joints
  const thumbJoints = fingerJointMap.thumb.map(n => robot.joints[n]).filter(Boolean)
  for (let i = 0; i < thumbJoints.length && i < angles.thumb.length; i++) {
    thumbJoints[i].setJointValue?.(angles.thumb[i])
  }

  // Index: 2 joints
  const indexJoints = fingerJointMap.index.map(n => robot.joints[n]).filter(Boolean)
  for (let i = 0; i < indexJoints.length && i < angles.index.length; i++) {
    indexJoints[i].setJointValue?.(angles.index[i])
  }

  // Middle: 2 joints
  const middleJoints = fingerJointMap.middle.map(n => robot.joints[n]).filter(Boolean)
  for (let i = 0; i < middleJoints.length && i < angles.middle.length; i++) {
    middleJoints[i].setJointValue?.(angles.middle[i])
  }
}

// ─── Tracking HUD ────────────────────────────────────────────────────────────
export function TrackingHUD() {
  const statusRef = useRef()
  const { gl } = useThree()

  useFrame((state, delta, xrFrame) => {
    if (!xrFrame || !statusRef.current) return
    const session = gl.xr.getSession()
    if (!session) return

    let leftFound = false, rightFound = false
    for (const src of session.inputSources) {
      if (src.hand) {
        if (src.handedness === 'left')  leftFound = true
        if (src.handedness === 'right') rightFound = true
      }
    }

    const color = (leftFound && rightFound)
      ? '#00ff88'
      : (leftFound || rightFound) ? '#ffaa00' : '#ff4444'

    statusRef.current.material?.color.set(color)
  })

  return (
    <mesh ref={statusRef} position={[0.28, -0.25, -0.5]}>
      <sphereGeometry args={[0.005, 6, 6]} />
      <meshBasicMaterial color="#ff4444" />
    </mesh>
  )
}
