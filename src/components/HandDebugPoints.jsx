import { useRef } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { XR_JOINT_NAMES } from '../constants/kinematics.js'

const JOINTS_PER_HAND = XR_JOINT_NAMES.length
const TOTAL = JOINTS_PER_HAND * 2
const RADIUS = 0.006
const WRIST_SCALE = 2.5

const _dummy = new THREE.Object3D()
const _hidden = new THREE.Matrix4().makeScale(0, 0, 0)

/**
 * Renders small colored spheres at XR hand joint positions.
 * Cyan = left hand, orange = right hand, larger sphere on wrist.
 * Must be placed OUTSIDE worldRef so positions stay in XR world space.
 */
export function HandDebugPoints() {
  const { gl } = useThree()
  const leftRef = useRef()
  const rightRef = useRef()

  useFrame((_, __, xrFrame) => {
    if (!xrFrame) return
    const session = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return

    for (const mesh of [leftRef.current, rightRef.current]) {
      if (!mesh) continue
      for (let i = 0; i < JOINTS_PER_HAND; i++) mesh.setMatrixAt(i, _hidden)
      mesh.instanceMatrix.needsUpdate = true
    }

    for (const source of session.inputSources) {
      if (!source.hand) continue
      const mesh = source.handedness === 'left' ? leftRef.current
                 : source.handedness === 'right' ? rightRef.current
                 : null
      if (!mesh) continue

      for (let i = 0; i < XR_JOINT_NAMES.length; i++) {
        const space = source.hand.get(XR_JOINT_NAMES[i])
        if (!space) continue
        const pose = xrFrame.getJointPose(space, refSpace)
        if (!pose) continue

        const p = pose.transform.position
        _dummy.position.set(p.x, p.y, p.z)
        const s = XR_JOINT_NAMES[i] === 'wrist' ? WRIST_SCALE : 1
        _dummy.scale.set(s, s, s)
        _dummy.updateMatrix()
        mesh.setMatrixAt(i, _dummy.matrix)
      }
      mesh.instanceMatrix.needsUpdate = true
    }
  })

  return (
    <>
      <instancedMesh ref={leftRef} args={[undefined, undefined, JOINTS_PER_HAND]} frustumCulled={false}>
        <sphereGeometry args={[RADIUS, 8, 8]} />
        <meshBasicMaterial color="#00ddff" transparent opacity={0.7} depthTest={false} />
      </instancedMesh>
      <instancedMesh ref={rightRef} args={[undefined, undefined, JOINTS_PER_HAND]} frustumCulled={false}>
        <sphereGeometry args={[RADIUS, 8, 8]} />
        <meshBasicMaterial color="#ff8844" transparent opacity={0.7} depthTest={false} />
      </instancedMesh>
    </>
  )
}
