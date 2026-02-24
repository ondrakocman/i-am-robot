import { URDFRobot, TrackingHUD } from './URDFRobot.jsx'

function Environment() {
  return (
    <>
      {/* Visible floor plane */}
      <mesh rotation-x={-Math.PI / 2} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[30, 30]} />
        <meshStandardMaterial color="#3a4a5a" roughness={0.9} metalness={0.0} />
      </mesh>

      {/* Grid overlay */}
      <gridHelper args={[30, 60, '#5588aa', '#445566']} position={[0, 0.001, 0]} />

      {/* Main key light — very bright white */}
      <directionalLight position={[5, 10, 7]} intensity={4} color="#ffffff" />

      {/* Fill from opposite side */}
      <directionalLight position={[-4, 6, -3]} intensity={2} color="#ffffff" />

      {/* Front fill to illuminate the robot body facing the user */}
      <directionalLight position={[0, 4, 8]} intensity={2} color="#eeeeff" />

      {/* Strong ambient — prevents anything from being black */}
      <ambientLight intensity={2.0} color="#ffffff" />

      {/* Hemisphere for natural sky/ground color */}
      <hemisphereLight skyColor="#aaccee" groundColor="#555555" intensity={1.5} />
    </>
  )
}

export function Scene() {
  return (
    <>
      <Environment />
      <URDFRobot />
      <TrackingHUD />
    </>
  )
}
