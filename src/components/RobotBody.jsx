import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { G1 } from '../constants/kinematics.js'

// ─── Capsule helper (Three.js CapsuleGeometry) ───────────────────────────────
function Capsule({ radius = 0.04, height = 0.2, color = '#2a2a3e', ...props }) {
  return (
    <mesh {...props}>
      <capsuleGeometry args={[radius, height, 8, 16]} />
      <meshStandardMaterial color={color} roughness={0.6} metalness={0.3} />
    </mesh>
  )
}

function Box({ args, color = '#1e1e2e', ...props }) {
  return (
    <mesh {...props}>
      <boxGeometry args={args} />
      <meshStandardMaterial color={color} roughness={0.5} metalness={0.4} />
    </mesh>
  )
}

// ─── G1 Robot Body ────────────────────────────────────────────────────────────
// Rendered egocentrically: the head IS the camera, body hangs below.
// groupRef is positioned each frame so robot head = XR camera pose.
// Only yaw rotation is applied (torso doesn't tilt when user looks up/down).

export function RobotBody({ headRef }) {
  const bodyRef = useRef()

  useFrame(({ camera }) => {
    if (!bodyRef.current) return

    // Extract yaw-only rotation from camera
    const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
    const yawQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, euler.y, 0))

    bodyRef.current.position.copy(camera.position)
    bodyRef.current.quaternion.copy(yawQuat)
  })

  // All positions are relative to head (0,0,0) = camera origin
  const neckY    = -G1.headHeight - G1.neckLength * 0.5       // ~-0.11
  const torsoY   = neckY - G1.neckLength * 0.5 - G1.torsoLength * 0.5  // ~-0.38
  const pelvisY  = torsoY - G1.torsoLength * 0.5 - G1.pelvisLength * 0.5 // ~-0.64
  const hipY     = pelvisY - G1.pelvisLength * 0.5             // ~-0.71
  const kneeY    = hipY - G1.thighLength                       // ~-1.03
  const ankleY   = kneeY - G1.shinLength                       // ~-1.33

  const robotColor   = '#1a1a2e'
  const accentColor  = '#00d4ff'
  const jointColor   = '#2d2d4e'

  return (
    <group ref={bodyRef}>
      {/* ── Neck ─────────────────────────────────────────────────────────── */}
      <Capsule
        radius={0.04}
        height={G1.neckLength}
        color={robotColor}
        position={[0, neckY, 0]}
      />

      {/* ── Upper Torso ──────────────────────────────────────────────────── */}
      <Box
        args={[G1.torsoWidth, G1.torsoLength * 0.55, G1.torsoDepth]}
        color={robotColor}
        position={[0, torsoY + G1.torsoLength * 0.12, 0]}
      />

      {/* Chest accent stripe */}
      <mesh position={[0, torsoY + G1.torsoLength * 0.15, G1.torsoDepth * 0.51]}>
        <planeGeometry args={[0.06, 0.18]} />
        <meshStandardMaterial color={accentColor} emissive={accentColor} emissiveIntensity={0.4} />
      </mesh>

      {/* ── Lower Torso / Waist ───────────────────────────────────────────── */}
      <Box
        args={[G1.torsoWidth * 0.85, G1.torsoLength * 0.35, G1.torsoDepth * 0.9]}
        color={jointColor}
        position={[0, torsoY - G1.torsoLength * 0.2, 0]}
      />

      {/* ── Pelvis ───────────────────────────────────────────────────────── */}
      <Box
        args={[G1.pelvisWidth, G1.pelvisLength, G1.torsoDepth * 0.85]}
        color={robotColor}
        position={[0, pelvisY, 0]}
      />

      {/* ── Legs ─────────────────────────────────────────────────────────── */}
      {[-1, 1].map((side) => {
        const x = side * G1.hipWidth
        return (
          <group key={side}>
            {/* Hip joint sphere */}
            <mesh position={[x, hipY, 0]}>
              <sphereGeometry args={[0.055, 12, 12]} />
              <meshStandardMaterial color={jointColor} metalness={0.5} roughness={0.4} />
            </mesh>

            {/* Thigh */}
            <Capsule
              radius={0.048}
              height={G1.thighLength - 0.08}
              color={robotColor}
              position={[x, hipY - G1.thighLength * 0.5, 0]}
              rotation={[0, 0, 0]}
            />

            {/* Knee joint */}
            <mesh position={[x, kneeY, 0]}>
              <sphereGeometry args={[0.050, 12, 12]} />
              <meshStandardMaterial color={jointColor} metalness={0.5} roughness={0.4} />
            </mesh>

            {/* Shin */}
            <Capsule
              radius={0.040}
              height={G1.shinLength - 0.08}
              color={robotColor}
              position={[x, kneeY - G1.shinLength * 0.5, 0]}
            />

            {/* Foot */}
            <Box
              args={[0.09, 0.06, 0.18]}
              color={robotColor}
              position={[x, ankleY - 0.03, 0.04]}
            />
          </group>
        )
      })}

      {/* ── Shoulder caps (visual, arm bones attach separately) ─────────── */}
      {[-1, 1].map((side) => (
        <mesh key={side} position={[side * (G1.torsoWidth * 0.5 + 0.01), torsoY + G1.torsoLength * 0.22, 0]}>
          <sphereGeometry args={[0.06, 12, 12]} />
          <meshStandardMaterial color={jointColor} metalness={0.5} roughness={0.4} />
        </mesh>
      ))}
    </group>
  )
}
