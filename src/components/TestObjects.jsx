import { RigidBody } from '@react-three/rapier'

const TABLE_X = 0
const TABLE_Y = 0.78
const TABLE_Z = -0.5
const TABLE_W = 1.0
const TABLE_D = 0.6
const TABLE_THICK = 0.03

const LEG_OFFSETS = [
  [-TABLE_W / 2 + 0.04, -TABLE_D / 2 + 0.04],
  [ TABLE_W / 2 - 0.04, -TABLE_D / 2 + 0.04],
  [-TABLE_W / 2 + 0.04,  TABLE_D / 2 - 0.04],
  [ TABLE_W / 2 - 0.04,  TABLE_D / 2 - 0.04],
]

export function TestObjects() {
  return (
    <>
      <RigidBody type="fixed" colliders="cuboid">
        <mesh position={[TABLE_X, TABLE_Y, TABLE_Z]}>
          <boxGeometry args={[TABLE_W, TABLE_THICK, TABLE_D]} />
          <meshStandardMaterial color="#8B6914" roughness={0.7} />
        </mesh>
        {LEG_OFFSETS.map(([dx, dz], i) => (
          <mesh key={i} position={[TABLE_X + dx, TABLE_Y / 2, TABLE_Z + dz]}>
            <boxGeometry args={[0.04, TABLE_Y - TABLE_THICK, 0.04]} />
            <meshStandardMaterial color="#6B4914" roughness={0.8} />
          </mesh>
        ))}
      </RigidBody>

      <RigidBody type="dynamic" colliders="cuboid" position={[-0.15, TABLE_Y + 0.04, -0.4]}>
        <mesh>
          <boxGeometry args={[0.05, 0.05, 0.05]} />
          <meshStandardMaterial color="#cc3333" roughness={0.3} metalness={0.1} />
        </mesh>
      </RigidBody>

      <RigidBody type="dynamic" colliders="ball" position={[0.15, TABLE_Y + 0.04, -0.45]}>
        <mesh>
          <sphereGeometry args={[0.03, 16, 16]} />
          <meshStandardMaterial color="#3366cc" roughness={0.2} metalness={0.3} />
        </mesh>
      </RigidBody>

      <RigidBody type="dynamic" colliders="hull" position={[0, TABLE_Y + 0.05, -0.55]}>
        <mesh>
          <cylinderGeometry args={[0.02, 0.02, 0.06, 12]} />
          <meshStandardMaterial color="#33aa55" roughness={0.4} />
        </mesh>
      </RigidBody>
    </>
  )
}
