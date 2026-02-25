import * as THREE from 'three'

const ARM_PREFIXES = [
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
  'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'left_hand_',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
  'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
  'right_hand_',
]

const SKIP_LINKS = new Set([
  'imu_in_torso', 'imu_in_pelvis', 'd435_link', 'mid360_link',
  'logo_link', 'waist_support_link',
])

const ARM_GROUP = 0x0001
const BODY_GROUP = 0x0002
const MAX_COLLIDER_VERTS = 512

const _pos = new THREE.Vector3()
const _quat = new THREE.Quaternion()
const _box = new THREE.Box3()

function isArmLink(name) {
  return ARM_PREFIXES.some(p => name.startsWith(p))
}

function extractSubsampledVertices(link, maxVerts) {
  const raw = []
  const linkInv = new THREE.Matrix4()
  link.updateWorldMatrix(true, false)
  linkInv.copy(link.matrixWorld).invert()

  link.traverse((child) => {
    if (!child.isMesh || !child.geometry) return
    const posAttr = child.geometry.attributes.position
    if (!posAttr) return

    child.updateWorldMatrix(true, false)
    const toLocal = new THREE.Matrix4().copy(child.matrixWorld).premultiply(linkInv)
    const v = new THREE.Vector3()

    for (let i = 0; i < posAttr.count; i++) {
      v.fromBufferAttribute(posAttr, i).applyMatrix4(toLocal)
      raw.push(v.x, v.y, v.z)
    }
  })

  if (raw.length < 9) return null

  const totalVerts = raw.length / 3
  if (totalVerts <= maxVerts) return new Float32Array(raw)

  const stride = Math.ceil(totalVerts / maxVerts)
  const sampled = []
  for (let i = 0; i < totalVerts; i += stride) {
    const idx = i * 3
    sampled.push(raw[idx], raw[idx + 1], raw[idx + 2])
  }
  return new Float32Array(sampled)
}

function computeBoundingBoxCollider(link, RAPIER) {
  _box.makeEmpty()
  const linkInv = new THREE.Matrix4()
  link.updateWorldMatrix(true, false)
  linkInv.copy(link.matrixWorld).invert()

  link.traverse((child) => {
    if (!child.isMesh || !child.geometry) return
    const posAttr = child.geometry.attributes.position
    if (!posAttr) return
    child.updateWorldMatrix(true, false)
    const toLocal = new THREE.Matrix4().copy(child.matrixWorld).premultiply(linkInv)
    const v = new THREE.Vector3()
    for (let i = 0; i < posAttr.count; i++) {
      v.fromBufferAttribute(posAttr, i).applyMatrix4(toLocal)
      _box.expandByPoint(v)
    }
  })

  if (_box.isEmpty()) return null
  const size = _box.getSize(new THREE.Vector3())
  const center = _box.getCenter(new THREE.Vector3())
  return RAPIER.ColliderDesc.cuboid(size.x / 2, size.y / 2, size.z / 2)
    .setTranslation(center.x, center.y, center.z)
}

export class PhysicsManager {
  constructor(rapier, world) {
    this.rapier = rapier
    this.world = world
    this.entries = new Map()
    this.colliderHandleToLink = new Map()
    this.linkNames = []
    this.armColliders = []
    this.collidingSides = { left: false, right: false }
  }

  init(robot) {
    const RAPIER = this.rapier
    const world = this.world
    let created = 0

    for (const [name, link] of Object.entries(robot.links)) {
      if (SKIP_LINKS.has(name)) continue

      const arm = isArmLink(name)
      const isFingerLink = name.includes('hand_') && !name.includes('palm')

      const bodyDesc = RAPIER.RigidBodyDesc.kinematicPositionBased()
      link.getWorldPosition(_pos)
      link.getWorldQuaternion(_quat)
      bodyDesc.setTranslation(_pos.x, _pos.y, _pos.z)
      bodyDesc.setRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w })
      const body = world.createRigidBody(bodyDesc)

      let colliderDesc = null
      if (isFingerLink) {
        colliderDesc = computeBoundingBoxCollider(link, RAPIER)
      } else {
        const verts = extractSubsampledVertices(link, MAX_COLLIDER_VERTS)
        if (verts) {
          colliderDesc = RAPIER.ColliderDesc.convexHull(verts)
        }
      }

      if (!colliderDesc) {
        world.removeRigidBody(body)
        continue
      }

      const membership = arm ? ARM_GROUP : BODY_GROUP
      const filter = arm ? (BODY_GROUP | ARM_GROUP) : ARM_GROUP
      colliderDesc.setCollisionGroups((membership << 16) | filter)
      colliderDesc.setActiveCollisionTypes(
        RAPIER.ActiveCollisionTypes.DEFAULT
        | RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED
        | RAPIER.ActiveCollisionTypes.KINEMATIC_KINEMATIC
      )
      colliderDesc.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)

      const collider = world.createCollider(colliderDesc, body)
      this.entries.set(name, { body, collider, arm })
      this.colliderHandleToLink.set(collider.handle, name)
      this.linkNames.push(name)
      if (arm) this.armColliders.push({ name, collider })
      created++
    }

    console.log(`[Physics] ${created} colliders (${this.armColliders.length} arm)`)
  }

  syncToPhysics(robot) {
    for (const name of this.linkNames) {
      const entry = this.entries.get(name)
      const link = robot.links[name]
      if (!entry || !link) continue

      link.getWorldPosition(_pos)
      link.getWorldQuaternion(_quat)
      entry.body.setNextKinematicTranslation({ x: _pos.x, y: _pos.y, z: _pos.z })
      entry.body.setNextKinematicRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w })
    }
  }

  checkCollisions() {
    this.collidingSides.left = false
    this.collidingSides.right = false

    for (const { name, collider } of this.armColliders) {
      this.world.contactPairsWith(collider, (other) => {
        const otherName = this.colliderHandleToLink.get(other.handle)
        if (!otherName) return
        const otherEntry = this.entries.get(otherName)
        if (otherEntry && !otherEntry.arm) {
          if (name.startsWith('left_')) this.collidingSides.left = true
          if (name.startsWith('right_')) this.collidingSides.right = true
        }
      })
    }

    return this.collidingSides
  }

  dispose() {
    for (const [, entry] of this.entries) {
      this.world.removeRigidBody(entry.body)
    }
    this.entries.clear()
    this.colliderHandleToLink.clear()
    this.linkNames = []
    this.armColliders = []
  }
}
