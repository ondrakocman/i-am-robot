import { useRef } from 'react'
import { RobotBody } from './RobotBody.jsx'
import { RobotArm } from './RobotArm.jsx'
import { HandTracker, TrackingHUD } from './HandTracker.jsx'

// ─── Environment ──────────────────────────────────────────────────────────────
function Environment() {
  return (
    <>
      {/* Grid floor for spatial reference */}
      <gridHelper args={[20, 40, '#003344', '#001a22']} position={[0, -1.4, 0]} />

      {/* Directional key light */}
      <directionalLight
        position={[3, 6, 4]}
        intensity={1.2}
        color="#d0e8ff"
        castShadow={false}
      />

      {/* Ambient fill */}
      <ambientLight intensity={0.4} color="#1a2a3a" />

      {/* Subtle hemisphere sky/ground */}
      <hemisphereLight
        skyColor="#1a3050"
        groundColor="#050810"
        intensity={0.6}
      />

      {/* Distant rim light to separate robot from background */}
      <directionalLight
        position={[-4, 2, -5]}
        intensity={0.4}
        color="#0055aa"
      />
    </>
  )
}

// ─── Main Scene ───────────────────────────────────────────────────────────────
// Composes the full robot simulation:
//   - RobotBody: torso/legs visible when looking down
//   - Two RobotArms: driven by IK + hand retargeting
//   - HandTracker: reads XR hand data and updates arms
//   - TrackingHUD: tiny indicator dot for tracking quality
//
// No physics for Phase 1 — pure kinematic simulation.

export function Scene() {
  const leftArmRef  = useRef()
  const rightArmRef = useRef()

  return (
    <>
      <Environment />

      {/* Robot body (egocentric — head = camera, body hangs below) */}
      <RobotBody />

      {/* Left arm */}
      <RobotArm ref={leftArmRef}  side="left" />

      {/* Right arm */}
      <RobotArm ref={rightArmRef} side="right" />

      {/* Hand tracking + IK driver (no render output) */}
      <HandTracker
        leftArmRef={leftArmRef}
        rightArmRef={rightArmRef}
      />

      {/* Tracking quality indicator */}
      <TrackingHUD />
    </>
  )
}
