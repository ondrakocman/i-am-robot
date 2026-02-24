import { useRef, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { G1 } from '../constants/kinematics.js'
import { Dex31Hand } from './Dex31Hand.jsx'

// ─── Arm Segment ──────────────────────────────────────────────────────────────
function ArmSegment({ from, to, radius = 0.030, color = '#1a1a2e' }) {
  const dir = new THREE.Vector3().subVectors(to, from)
  const length = dir.length()
  if (length < 0.001) return null

  const mid = new THREE.Vector3().addVectors(from, to).multiplyScalar(0.5)

  // Align capsule Y-axis with direction vector
  const q = new THREE.Quaternion()
  q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize())

  return (
    <mesh position={mid} quaternion={q}>
      <capsuleGeometry args={[radius, length - radius * 2, 6, 12]} />
      <meshStandardMaterial color={color} roughness={0.55} metalness={0.3} />
    </mesh>
  )
}

function JointBall({ position, radius = 0.036, color = '#0d1a33' }) {
  return (
    <mesh position={position}>
      <sphereGeometry args={[radius, 10, 10]} />
      <meshStandardMaterial color={color} metalness={0.55} roughness={0.35} />
    </mesh>
  )
}

// ─── RobotArm ─────────────────────────────────────────────────────────────────
// Rendered as: shoulder joint → upper arm → elbow joint → forearm → wrist → Dex hand
//
// Imperative API:
//   armRef.current.update({ shoulderWorld, elbowWorld, wristWorld, wristQuat, fingerAngles })
//
// All positions are world-space THREE.Vector3.
// The Dex 3.1 hand is positioned at wristWorld with wristQuat orientation.

export const RobotArm = forwardRef(function RobotArm({ side = 'left' }, ref) {
  const groupRef  = useRef()
  const handRef   = useRef()

  // Mutable arm state
  const state = useRef({
    shoulder: new THREE.Vector3(),
    elbow:    new THREE.Vector3(),
    wrist:    new THREE.Vector3(),
    wristQ:   new THREE.Quaternion(),
  })

  // Thin wrapper Objects to hold segment/joint refs for imperative updates
  const segRefs = useRef({
    upperArmMesh: null,
    forearmMesh:  null,
    shoulderBall: null,
    elbowBall:    null,
    wristGroup:   null,
  })

  useImperativeHandle(ref, () => ({
    update({ shoulderWorld, elbowWorld, wristWorld, wristQuat, fingerAngles }) {
      state.current.shoulder.copy(shoulderWorld)
      state.current.elbow.copy(elbowWorld)
      state.current.wrist.copy(wristWorld)
      state.current.wristQ.copy(wristQuat)

      // Update shoulder joint ball
      if (segRefs.current.shoulderBall) {
        segRefs.current.shoulderBall.position.copy(shoulderWorld)
      }
      // Update elbow ball
      if (segRefs.current.elbowBall) {
        segRefs.current.elbowBall.position.copy(elbowWorld)
      }
      // Upper arm segment
      updateSegment(segRefs.current.upperArmMesh, shoulderWorld, elbowWorld)
      // Forearm segment
      updateSegment(segRefs.current.forearmMesh,  elbowWorld,    wristWorld)

      // Wrist group (hand position + orientation)
      if (segRefs.current.wristGroup) {
        segRefs.current.wristGroup.position.copy(wristWorld)
        segRefs.current.wristGroup.quaternion.copy(wristQuat)
      }

      // Finger angles
      if (handRef.current && fingerAngles) {
        handRef.current.setAngles(fingerAngles)
      }
    },

    getHandRef() { return handRef },
  }))

  return (
    <group ref={groupRef}>
      {/* Segment meshes — updated imperatively */}
      <UpperArmMesh segRefs={segRefs} />

      {/* Wrist group: contains the Dex 3.1 hand */}
      <group ref={(el) => { segRefs.current.wristGroup = el }}>
        {/* Wrist joint sphere */}
        <mesh>
          <sphereGeometry args={[0.030, 10, 10]} />
          <meshStandardMaterial color="#0d1a33" metalness={0.55} roughness={0.35} />
        </mesh>
        {/* Dex 3.1 hand, offset slightly forward from wrist joint */}
        <group position={[0, 0, -0.04]} rotation={[0, side === 'right' ? Math.PI : 0, 0]}>
          <Dex31Hand ref={handRef} side={side} />
        </group>
      </group>
    </group>
  )
})

// ─── Segment mesh updater ─────────────────────────────────────────────────────
// Bone lengths are fixed (IK enforces G1.upperArmLength / forearmLength),
// so we only need to reposition + reorient, no scale needed.
const _Y_AXIS = new THREE.Vector3(0, 1, 0)
const _segDir = new THREE.Vector3()
const _segQ   = new THREE.Quaternion()

function updateSegment(mesh, from, to) {
  if (!mesh) return
  _segDir.subVectors(to, from)
  const length = _segDir.length()
  if (length < 0.001) return

  mesh.position.addVectors(from, to).multiplyScalar(0.5)
  _segQ.setFromUnitVectors(_Y_AXIS, _segDir.normalize())
  mesh.quaternion.copy(_segQ)
}

// ─── Arm mesh sub-component ───────────────────────────────────────────────────
// Uses callback refs to store mesh references in the parent segRefs
function UpperArmMesh({ segRefs }) {
  return (
    <>
      {/* Shoulder joint sphere */}
      <mesh ref={(el) => { segRefs.current.shoulderBall = el }}>
        <sphereGeometry args={[0.042, 10, 10]} />
        <meshStandardMaterial color="#0d1a33" metalness={0.55} roughness={0.35} />
      </mesh>

      {/* Upper arm capsule */}
      <mesh ref={(el) => { segRefs.current.upperArmMesh = el }}>
        <capsuleGeometry args={[0.030, G1.upperArmLength - 0.06, 6, 12]} />
        <meshStandardMaterial color="#1a1a2e" roughness={0.55} metalness={0.3} />
      </mesh>

      {/* Elbow joint sphere */}
      <mesh ref={(el) => { segRefs.current.elbowBall = el }}>
        <sphereGeometry args={[0.036, 10, 10]} />
        <meshStandardMaterial color="#0d1a33" metalness={0.55} roughness={0.35} />
      </mesh>

      {/* Forearm capsule */}
      <mesh ref={(el) => { segRefs.current.forearmMesh = el }}>
        <capsuleGeometry args={[0.026, G1.forearmLength - 0.06, 6, 12]} />
        <meshStandardMaterial color="#1a1a2e" roughness={0.55} metalness={0.3} />
      </mesh>
    </>
  )
}
