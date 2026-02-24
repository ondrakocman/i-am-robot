import { useRef, useEffect } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { XR_JOINT_NAMES, G1 } from '../constants/kinematics.js'
import { HandImpedance } from '../systems/ImpedanceControl.js'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { solveTwoBoneIK, getLeftElbowPole, getRightElbowPole } from '../systems/TwoBoneIK.js'

// ─── Temp reusables (avoid per-frame allocation) ──────────────────────────────
const _wristPos  = new THREE.Vector3()
const _wristQuat = new THREE.Quaternion()
const _shoulderL = new THREE.Vector3()
const _shoulderR = new THREE.Vector3()
const _poleL     = new THREE.Vector3()
const _poleR     = new THREE.Vector3()

// ─── HandTracker ─────────────────────────────────────────────────────────────
// Reads WebXR hand tracking data each frame.
// Drives left and right RobotArm refs with:
//   - Two-bone IK for shoulder→elbow→wrist chain
//   - Impedance-filtered wrist pose
//   - Retargeted Dex 3.1 finger angles
//
// Props:
//   leftArmRef  — ref to <RobotArm side="left" />
//   rightArmRef — ref to <RobotArm side="right" />
//   armScale    — optional scale factor from calibration (default 1)

export function HandTracker({ leftArmRef, rightArmRef, armScale = 1 }) {
  const { gl, camera } = useThree()

  const impedanceL = useRef(new HandImpedance())
  const impedanceR = useRef(new HandImpedance())
  const retargetL  = useRef(new RetargetingFilter(0.18))
  const retargetR  = useRef(new RetargetingFilter(0.18))

  // Track whether hand was present last frame (for reset on reconnect)
  const hadHandL = useRef(false)
  const hadHandR = useRef(false)

  useFrame((state, delta, xrFrame) => {
    if (!xrFrame) return

    const session  = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return

    // Shoulder world positions — must exactly match RobotBody's shoulder caps.
    // In RobotBody (head = camera origin):
    //   neckY    = -(headHeight + neckLength/2)          = -(0.14 + 0.04) = -0.18
    //   torsoY   = neckY - neckLength/2 - torsoLength/2  = -0.18 - 0.04 - 0.19 = -0.41
    //   shoulderY = torsoY + torsoLength * 0.22           = -0.41 + 0.084 ≈ -0.326
    //   shoulderX = ±(torsoWidth/2 + 0.01)               = ±0.15
    const SHOULDER_Y = -0.326
    const SHOULDER_X =  0.150
    const SHOULDER_Z = -0.010

    // Only yaw rotation — torso doesn't pitch/roll when user looks around
    const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
    const yawQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y, 0))

    const lLocal = new THREE.Vector3( SHOULDER_X, SHOULDER_Y, SHOULDER_Z).applyQuaternion(yawQuat)
    const rLocal = new THREE.Vector3(-SHOULDER_X, SHOULDER_Y, SHOULDER_Z).applyQuaternion(yawQuat)

    _shoulderL.copy(camera.position).add(lLocal)
    _shoulderR.copy(camera.position).add(rLocal)

    // Process each input source
    let foundLeft  = false
    let foundRight = false

    for (const source of session.inputSources) {
      if (!source.hand) continue

      const isLeft = source.handedness === 'left'
      foundLeft  = foundLeft  || isLeft
      foundRight = foundRight || !isLeft

      // Extract all joint world poses
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

      // Wrist pose
      const wristJoint = joints['wrist']
      if (!wristJoint) continue

      _wristPos.copy(wristJoint.position)
      _wristQuat.copy(wristJoint.quaternion)

      const impedance = isLeft ? impedanceL.current : impedanceR.current
      const retarget  = isLeft ? retargetL.current  : retargetR.current
      const armRef    = isLeft ? leftArmRef          : rightArmRef

      // Reset impedance if hand reconnects
      const hadHand = isLeft ? hadHandL : hadHandR
      if (!hadHand.current) {
        impedance.reset()
        impedance.wristPos.reset(_wristPos)
        impedance.wristQuat.reset(_wristQuat)
      }
      hadHand.current = true

      // Impedance filter: smooth wrist pose
      const { position: smoothPos, quaternion: smoothQuat } = impedance.update(
        _wristPos, _wristQuat, Math.min(delta, 0.05)
      )

      // Scale wrist position toward shoulder to match robot's workspace
      if (armScale !== 1) {
        const shoulder = isLeft ? _shoulderL : _shoulderR
        const scaled = smoothPos.clone().sub(shoulder).multiplyScalar(armScale).add(shoulder)
        smoothPos.copy(scaled)
      }

      // Two-bone IK
      const shoulder = isLeft ? _shoulderL : _shoulderR
      _poleL.copy(getLeftElbowPole(shoulder))
      _poleR.copy(getRightElbowPole(shoulder))
      const pole = isLeft ? _poleL : _poleR

      const { elbowPos } = solveTwoBoneIK(
        shoulder,
        smoothPos,
        pole,
        G1.upperArmLength,
        G1.forearmLength
      )

      // Hand retargeting
      const rawAngles    = retargetHand(joints)
      const smoothAngles = retarget.update(rawAngles)

      // Push to arm component
      if (armRef?.current) {
        armRef.current.update({
          shoulderWorld: shoulder.clone(),
          elbowWorld:    elbowPos,
          wristWorld:    smoothPos.clone(),
          wristQuat:     smoothQuat.clone(),
          fingerAngles:  smoothAngles,
        })
      }
    }

    // Reset impedance if hands disappear
    if (!foundLeft  && hadHandL.current) { impedanceL.current.reset(); hadHandL.current = false }
    if (!foundRight && hadHandR.current) { impedanceR.current.reset(); hadHandR.current = false }
  })

  return null
}

// ─── XR Session Status Display (HUD) ─────────────────────────────────────────
// Small floating text in VR showing tracking quality.
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

    // Tint color by tracking status
    const color = (leftFound && rightFound)
      ? '#00ff88'
      : (leftFound || rightFound) ? '#ffaa00' : '#ff4444'

    if (statusRef.current.material) {
      statusRef.current.material.color.set(color)
    }
  })

  // Small indicator dot in peripheral view
  return (
    <mesh ref={statusRef} position={[0.28, -0.25, -0.5]}>
      <sphereGeometry args={[0.005, 6, 6]} />
      <meshBasicMaterial color="#ff4444" />
    </mesh>
  )
}
