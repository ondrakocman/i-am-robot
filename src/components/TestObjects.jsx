import { RigidBody } from '@react-three/rapier'

export function TestObjects() {
  return (
    <>
      {/* Table */}
      <RigidBody type="fixed" colliders="cuboid">
        <mesh position={[0.5, 0.35, -0.4]}>
          <boxGeometry args={[0.6, 0.02, 0.4]} />
          <meshStandardMaterial color="#8B6914" roughness={0.7} />
        </mesh>
        {/* Legs */}
        {[[-0.25, 0.17, -0.15], [0.25, 0.17, -0.15], [-0.25, 0.17, 0.15], [0.25, 0.17, 0.15]].map((pos, i) => (
          <mesh key={i} position={pos.map((v, j) => j === 0 ? v + 0.5 : j === 2 ? v - 0.4 : v)}>
            <boxGeometry args={[0.03, 0.34, 0.03]} />
            <meshStandardMaterial color="#6B4914" roughness={0.8} />
          </mesh>
        ))}
      </RigidBody>

      {/* Red cube on the table */}
      <RigidBody type="dynamic" colliders="cuboid" position={[0.4, 0.42, -0.4]}>
        <mesh>
          <boxGeometry args={[0.05, 0.05, 0.05]} />
          <meshStandardMaterial color="#cc3333" roughness={0.3} metalness={0.1} />
        </mesh>
      </RigidBody>

      {/* Blue sphere on the table */}
      <RigidBody type="dynamic" colliders="ball" position={[0.6, 0.42, -0.35]}>
        <mesh>
          <sphereGeometry args={[0.03, 16, 16]} />
          <meshStandardMaterial color="#3366cc" roughness={0.2} metalness={0.3} />
        </mesh>
      </RigidBody>

      {/* Green cylinder on the table */}
      <RigidBody type="dynamic" colliders="hull" position={[0.5, 0.44, -0.45]}>
        <mesh>
          <cylinderGeometry args={[0.02, 0.02, 0.06, 12]} />
          <meshStandardMaterial color="#33aa55" roughness={0.4} />
        </mesh>
      </RigidBody>
    </>
  )
}
