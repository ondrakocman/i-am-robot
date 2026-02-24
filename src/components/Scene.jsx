import { URDFRobot, TrackingHUD } from './URDFRobot.jsx'

// ─── Environment ──────────────────────────────────────────────────────────────
function Environment() {
  return (
    <>
      {/* Grid floor for spatial reference */}
      <gridHelper args={[20, 40, '#335566', '#1a3344']} position={[0, -1.0, 0]} />

      {/* Strong directional key light (sun-like) */}
      <directionalLight
        position={[3, 8, 4]}
        intensity={2.5}
        color="#ffffff"
      />

      {/* Secondary fill from front */}
      <directionalLight
        position={[-2, 4, 6]}
        intensity={1.2}
        color="#e0e8ff"
      />

      {/* Bright ambient so nothing is black */}
      <ambientLight intensity={1.0} color="#b0c0d0" />

      {/* Hemisphere for sky/ground separation */}
      <hemisphereLight
        skyColor="#87CEEB"
        groundColor="#2a2a2a"
        intensity={0.8}
      />

      {/* Rim/back light for depth */}
      <directionalLight
        position={[-4, 3, -5]}
        intensity={0.8}
        color="#4488cc"
      />
    </>
  )
}

// ─── Main Scene ───────────────────────────────────────────────────────────────
export function Scene() {
  return (
    <>
      <Environment />
      <URDFRobot />
      <TrackingHUD />
    </>
  )
}
