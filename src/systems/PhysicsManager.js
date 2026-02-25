import * as THREE from 'three'

const KINEMATIC_PREFIXES = [
  'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
  'left_elbow', 'left_wrist_roll', 'left_wrist_pitch', 'left_wrist_yaw',
  'left_hand_',
  'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
  'right_elbow', 'right_wrist_roll', 'right_wrist_pitch', 'right_wrist_yaw',
  'right_hand_',
]

const SKIP_LINKS = new Set([
  'imu_in_torso', 'imu_in_pelvis', 'd435_link', 'mid360_link',
])

const ARM_GROUP = 0x0001
const BODY_GROUP = 0x0002

const _pos = new THREE.Vector3()
const _quat = new THREE.Quaternion()

function isKinematicLink(name) {
  return KINEMATIC_PREFIXES.some(p => name.startsWith(p))
}

function extractGeometryVertices(link) {
  const vertices = []
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
      v.fromBufferAttribute(posAttr, i)
      v.applyMatrix4(toLocal)
      vertices.push(v.x, v.y, v.z)
    }
  })
  return vertices.length > 0 ? new Float32Array(vertices) : null
}

function extractGeometryIndices(link) {
  const allIndices = []
  let vertexOffset = 0
  link.traverse((child) => {
    if (!child.isMesh || !child.geometry) return
    const posAttr = child.geometry.attributes.position
    if (!posAttr) return

    if (child.geometry.index) {
      const idx = child.geometry.index
      for (let i = 0; i < idx.count; i++) {
        allIndices.push(idx.getX(i) + vertexOffset)
      }
    } else {
      for (let i = 0; i < posAttr.count; i++) {
        allIndices.push(i + vertexOffset)
      }
    }
    vertexOffset += posAttr.count
  })
  return allIndices.length > 0 ? new Uint32Array(allIndices) : null
}

export class PhysicsManager {
  constructor(rapier, world) {
    this.rapier = rapier
    this.world = world
    this.entries = new Map()
    this.colliderHandleToLink = new Map()
    this.allLinkNames = []
    this.armColliders = []
    this.collidingSides = { left: false, right: false }
  }

  init(robot) {
    const RAPIER = this.rapier
    const world = this.world

    for (const [linkName, link] of Object.entries(robot.links)) {
      if (SKIP_LINKS.has(linkName)) continue

      const verts = extractGeometryVertices(link)
      if (!verts || verts.length < 9) continue

      const isArm = isKinematicLink(linkName)

      const bodyDesc = RAPIER.RigidBodyDesc.kinematicPositionBased()
      link.getWorldPosition(_pos)
      link.getWorldQuaternion(_quat)
      bodyDesc.setTranslation(_pos.x, _pos.y, _pos.z)
      bodyDesc.setRotation({ x: _quat.x, y: _quat.y, z: _quat.z, w: _quat.w })

      const body = world.createRigidBody(bodyDesc)

      let colliderDesc = null
      if (isArm) {
        colliderDesc = RAPIER.ColliderDesc.convexHull(verts)
      } else {
        const indices = extractGeometryIndices(link)
        if (indices && indices.length >= 3) {
          colliderDesc = RAPIER.ColliderDesc.trimesh(verts, indices)
        }
        if (!colliderDesc) {
          colliderDesc = RAPIER.ColliderDesc.convexHull(verts)
        }
      }

      if (!colliderDesc) {
        world.removeRigidBody(body)
        continue
      }

      const membership = isArm ? ARM_GROUP : BODY_GROUP
      const filter = isArm ? (BODY_GROUP | ARM_GROUP) : ARM_GROUP
      colliderDesc.setCollisionGroups((membership << 16) | filter)

      colliderDesc.setActiveCollisionTypes(
        RAPIER.ActiveCollisionTypes.DEFAULT
        | RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED
        | RAPIER.ActiveCollisionTypes.KINEMATIC_KINEMATIC
      )
      colliderDesc.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS)

      const collider = world.createCollider(colliderDesc, body)

      this.entries.set(linkName, { body, collider, isArm })
      this.colliderHandleToLink.set(collider.handle, linkName)
      this.allLinkNames.push(linkName)

      if (isArm) {
        this.armColliders.push({ linkName, collider })
      }
    }

    console.log(`[PhysicsManager] Created ${this.entries.size} bodies (${this.armColliders.length} arm colliders)`)
  }

  syncToPhysics(robot) {
    for (const linkName of this.allLinkNames) {
      const entry = this.entries.get(linkName)
      const link = robot.links[linkName]
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

    for (const { linkName, collider } of this.armColliders) {
      this.world.contactPairsWith(collider, (otherCollider) => {
        const otherName = this.colliderHandleToLink.get(otherCollider.handle)
        if (!otherName) return
        const otherEntry = this.entries.get(otherName)
        if (otherEntry && !otherEntry.isArm) {
          if (linkName.startsWith('left_')) this.collidingSides.left = true
          if (linkName.startsWith('right_')) this.collidingSides.right = true
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
    this.allLinkNames = []
    this.armColliders = []
  }
}
