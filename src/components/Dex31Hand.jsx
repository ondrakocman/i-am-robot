import { useRef, forwardRef, useImperativeHandle } from 'react'
import * as THREE from 'three'
import { DEX31 } from '../constants/kinematics.js'

// ─── Materials ────────────────────────────────────────────────────────────────
const HAND_COLOR   = '#1e2a4a'
const JOINT_COLOR  = '#0d1a33'
const ACCENT_COLOR = '#00aaff'
const TIP_COLOR    = '#003366'

function FingerSegment({ length, radius = 0.009, color = HAND_COLOR, ...props }) {
  return (
    <mesh {...props}>
      <capsuleGeometry args={[radius, length, 6, 8]} />
      <meshStandardMaterial color={color} roughness={0.5} metalness={0.3} />
    </mesh>
  )
}

function JointSphere({ radius = 0.011, color = JOINT_COLOR, ...props }) {
  return (
    <mesh {...props}>
      <sphereGeometry args={[radius, 8, 8]} />
      <meshStandardMaterial color={color} metalness={0.6} roughness={0.3} />
    </mesh>
  )
}

// ─── Single Finger with 2 joints ─────────────────────────────────────────────
// basePos: local position relative to palm, baseRot: Euler rotation of base
// j0, j1: joint angles (radians, positive = flex)
// segments: [L_proximal, L_distal]
function TwoJointFinger({ basePos, j0 = 0, j1 = 0, segments, color = HAND_COLOR }) {
  const L0 = segments[0]
  const L1 = segments[1]

  return (
    <group position={basePos}>
      {/* Proximal phalanx — rotates on j0 */}
      <group rotation={[-j0, 0, 0]}>
        <FingerSegment
          length={L0}
          radius={0.009}
          color={color}
          position={[0, L0 * 0.5, 0]}
          rotation={[Math.PI / 2, 0, 0]}
        />
        <JointSphere position={[0, L0, 0]} radius={0.010} />

        {/* Distal phalanx — rotates on j1, relative to proximal end */}
        <group position={[0, L0, 0]} rotation={[-j1, 0, 0]}>
          <FingerSegment
            length={L1}
            radius={0.008}
            color={color}
            position={[0, L1 * 0.5, 0]}
            rotation={[Math.PI / 2, 0, 0]}
          />
          {/* Fingertip dome */}
          <mesh position={[0, L1, 0]}>
            <sphereGeometry args={[0.010, 8, 8]} />
            <meshStandardMaterial color={TIP_COLOR} roughness={0.4} metalness={0.2} />
          </mesh>
        </group>
      </group>
    </group>
  )
}

// ─── Thumb with 3 joints ──────────────────────────────────────────────────────
function Thumb({ j0 = 0, j1 = 0, j2 = 0, side = 'left' }) {
  const S = DEX31.thumbSegments
  const mirrorX = side === 'right' ? -1 : 1

  return (
    <group
      position={[mirrorX * DEX31.thumbBase[0], DEX31.thumbBase[1], DEX31.thumbBase[2]]}
    >
      {/* j0: abduction around Y axis */}
      <group rotation={[0, j0 * mirrorX, 0]}>
        {/* Thumb metacarpal visual */}
        <mesh position={[0, S[0] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <capsuleGeometry args={[0.010, S[0] * 0.7, 6, 8]} />
          <meshStandardMaterial color={HAND_COLOR} roughness={0.5} metalness={0.3} />
        </mesh>
        <JointSphere position={[0, S[0], 0]} radius={0.012} />

        {/* j1: proximal flexion */}
        <group position={[0, S[0], 0]} rotation={[-j1, 0, 0]}>
          <FingerSegment
            length={S[1]}
            radius={0.009}
            color={HAND_COLOR}
            position={[0, S[1] * 0.5, 0]}
            rotation={[Math.PI / 2, 0, 0]}
          />
          <JointSphere position={[0, S[1], 0]} radius={0.011} />

          {/* j2: distal flexion (gear driven) */}
          <group position={[0, S[1], 0]} rotation={[-j2, 0, 0]}>
            <FingerSegment
              length={S[2]}
              radius={0.008}
              color={HAND_COLOR}
              position={[0, S[2] * 0.5, 0]}
              rotation={[Math.PI / 2, 0, 0]}
            />
            <mesh position={[0, S[2], 0]}>
              <sphereGeometry args={[0.010, 8, 8]} />
              <meshStandardMaterial color={TIP_COLOR} roughness={0.4} metalness={0.2} />
            </mesh>
          </group>
        </group>
      </group>
    </group>
  )
}

// ─── Full Dex 3.1 Hand ────────────────────────────────────────────────────────
// Exported with an imperative handle so HandTracker can update joint angles
// without re-rendering (performance critical in 90Hz VR loop).
//
// Usage:
//   const handRef = useRef()
//   <Dex31Hand ref={handRef} side="left" />
//   handRef.current.setAngles({ thumb:[0,0,0], index:[0,0], middle:[0,0] })

export const Dex31Hand = forwardRef(function Dex31Hand({ side = 'left' }, ref) {
  const mirrorX = side === 'right' ? -1 : 1

  // Mutable angle state — updated imperatively to avoid re-render
  const angles = useRef({
    thumb:  [0, 0, 0],
    index:  [0, 0],
    middle: [0, 0],
  })

  // Refs to individual joint groups for imperative rotation
  const thumbJ0Ref   = useRef()
  const thumbJ1Ref   = useRef()
  const thumbJ2Ref   = useRef()
  const indexJ0Ref   = useRef()
  const indexJ1Ref   = useRef()
  const middleJ0Ref  = useRef()
  const middleJ1Ref  = useRef()

  useImperativeHandle(ref, () => ({
    setAngles(newAngles) {
      angles.current = newAngles

      // Thumb
      if (thumbJ0Ref.current && newAngles.thumb) {
        thumbJ0Ref.current.rotation.y   = newAngles.thumb[0] * mirrorX
        thumbJ1Ref.current.rotation.x   = -newAngles.thumb[1]
        thumbJ2Ref.current.rotation.x   = -newAngles.thumb[2]
      }
      // Index
      if (indexJ0Ref.current && newAngles.index) {
        indexJ0Ref.current.rotation.x  = -newAngles.index[0]
        indexJ1Ref.current.rotation.x  = -newAngles.index[1]
      }
      // Middle
      if (middleJ0Ref.current && newAngles.middle) {
        middleJ0Ref.current.rotation.x = -newAngles.middle[0]
        middleJ1Ref.current.rotation.x = -newAngles.middle[1]
      }
    },
  }))

  const S  = DEX31.fingerSegments
  const ST = DEX31.thumbSegments

  return (
    <group scale={[mirrorX, 1, 1]}>
      {/* ── Palm ──────────────────────────────────────────────────────────── */}
      <mesh>
        <boxGeometry args={[DEX31.palmWidth, DEX31.palmHeight, DEX31.palmDepth]} />
        <meshStandardMaterial color={HAND_COLOR} roughness={0.5} metalness={0.3} />
      </mesh>

      {/* Accent line on back of hand */}
      <mesh position={[0, 0.01, DEX31.palmDepth * 0.51]}>
        <planeGeometry args={[0.04, 0.06]} />
        <meshStandardMaterial color={ACCENT_COLOR} emissive={ACCENT_COLOR} emissiveIntensity={0.3} />
      </mesh>

      {/* ── Thumb ─────────────────────────────────────────────────────────── */}
      <group
        position={[DEX31.thumbBase[0], DEX31.thumbBase[1], DEX31.thumbBase[2]]}
      >
        <group ref={thumbJ0Ref}>
          <mesh position={[0, ST[0] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]}>
            <capsuleGeometry args={[0.010, ST[0] * 0.6, 6, 8]} />
            <meshStandardMaterial color={HAND_COLOR} roughness={0.5} metalness={0.3} />
          </mesh>
          <JointSphere position={[0, ST[0], 0]} radius={0.012} />

          <group position={[0, ST[0], 0]} ref={thumbJ1Ref}>
            <FingerSegment length={ST[1]} radius={0.009} color={HAND_COLOR}
              position={[0, ST[1] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
            <JointSphere position={[0, ST[1], 0]} radius={0.011} />

            <group position={[0, ST[1], 0]} ref={thumbJ2Ref}>
              <FingerSegment length={ST[2]} radius={0.008} color={HAND_COLOR}
                position={[0, ST[2] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
              <mesh position={[0, ST[2], 0]}>
                <sphereGeometry args={[0.010, 8, 8]} />
                <meshStandardMaterial color={TIP_COLOR} roughness={0.4} metalness={0.2} />
              </mesh>
            </group>
          </group>
        </group>
      </group>

      {/* ── Index Finger ─────────────────────────────────────────────────── */}
      <group position={[DEX31.indexBase[0], DEX31.indexBase[1], DEX31.indexBase[2]]}>
        <group ref={indexJ0Ref}>
          <FingerSegment length={S[0]} radius={0.009} color={HAND_COLOR}
            position={[0, S[0] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
          <JointSphere position={[0, S[0], 0]} radius={0.010} />

          <group position={[0, S[0], 0]} ref={indexJ1Ref}>
            <FingerSegment length={S[1]} radius={0.008} color={HAND_COLOR}
              position={[0, S[1] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
            <mesh position={[0, S[1], 0]}>
              <sphereGeometry args={[0.010, 8, 8]} />
              <meshStandardMaterial color={TIP_COLOR} roughness={0.4} metalness={0.2} />
            </mesh>
          </group>
        </group>
      </group>

      {/* ── Middle Finger ────────────────────────────────────────────────── */}
      <group position={[DEX31.middleBase[0], DEX31.middleBase[1], DEX31.middleBase[2]]}>
        <group ref={middleJ0Ref}>
          <FingerSegment length={S[0]} radius={0.009} color={HAND_COLOR}
            position={[0, S[0] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
          <JointSphere position={[0, S[0], 0]} radius={0.010} />

          <group position={[0, S[0], 0]} ref={middleJ1Ref}>
            <FingerSegment length={S[1]} radius={0.008} color={HAND_COLOR}
              position={[0, S[1] * 0.5, 0]} rotation={[Math.PI / 2, 0, 0]} />
            <mesh position={[0, S[1], 0]}>
              <sphereGeometry args={[0.010, 8, 8]} />
              <meshStandardMaterial color={TIP_COLOR} roughness={0.4} metalness={0.2} />
            </mesh>
          </group>
        </group>
      </group>
    </group>
  )
})
