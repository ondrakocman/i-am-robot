import { useRef } from 'react'
import { Physics, RigidBody, CuboidCollider } from '@react-three/rapier'
import { URDFRobot, TrackingHUD } from './URDFRobot.jsx'
import { TestObjects } from './TestObjects.jsx'
import { HandDebugPoints } from './HandDebugPoints.jsx'

function Environment() {
  return (
    <>
      <mesh rotation-x={-Math.PI / 2} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[30, 30]} />
        <meshStandardMaterial color="#3a4a5a" roughness={0.9} metalness={0.0} />
      </mesh>

      <gridHelper args={[30, 60, '#5588aa', '#445566']} position={[0, 0.001, 0]} />

      <directionalLight position={[5, 10, 7]} intensity={4} color="#ffffff" />
      <directionalLight position={[-4, 6, -3]} intensity={2} color="#ffffff" />
      <directionalLight position={[0, 4, 8]} intensity={2} color="#eeeeff" />
      <ambientLight intensity={2.0} color="#ffffff" />
      <hemisphereLight skyColor="#aaccee" groundColor="#555555" intensity={1.5} />
    </>
  )
}

function PhysicsFloor() {
  return (
    <RigidBody type="fixed" colliders={false}>
      <CuboidCollider args={[15, 0.01, 15]} position={[0, -0.01, 0]} />
    </RigidBody>
  )
}

export function Scene({ vrMode }) {
  const worldRef = useRef()

  return (
    <>
      <Physics gravity={[0, -9.81, 0]}>
        <group ref={worldRef}>
          <Environment />
          <PhysicsFloor />
          <URDFRobot vrMode={vrMode} worldRef={worldRef} />
          <TestObjects />
        </group>
      </Physics>
      <HandDebugPoints />
      <TrackingHUD />
    </>
  )
}
