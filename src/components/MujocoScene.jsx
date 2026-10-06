import { useEffect, useRef, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import { retargetHand, RetargetingFilter } from '../systems/HandRetargeting.js'
import { QuaternionSmoother } from '../systems/ImpedanceControl.js'
import { OneEuroVector3 } from '../systems/OneEuroFilter.js'
import { XR_JOINT_NAMES, ROBOT_BASE_QUAT, XR_TO_URDF_L, XR_TO_URDF_R } from '../constants/kinematics.js'
import { HAND_INPUT, INPUT_SIZE, RAW_SIZE } from '../sim/TubeBoxSim.js'
import { saveEpisode, onEpisodesChanged } from '../sim/episodeStore.js'

const params = new URLSearchParams(location.search)
const SESSION_ID = crypto.randomUUID?.() ?? String(Date.now())

const MAT_BODY = new THREE.MeshStandardMaterial({ color: 0x4a4a6e, roughness: 0.4, metalness: 0.25 })
const MAT_ACCENT = new THREE.MeshStandardMaterial({ color: 0x6a6a9e, roughness: 0.35, metalness: 0.3 })
const ACCENT_MESH = /contour|shoulder_roll|shoulder_pitch|waist|logo/
const SCENE_BODIES = new Set(['world', 'table', 'box', 'tube'])
const TOUCH_EMISSIVE = new THREE.Color(0x1f6f3a)
const NO_EMISSIVE = new THREE.Color(0x000000)
const CORRECTION = [XR_TO_URDF_L, XR_TO_URDF_R]
const RAW_HAND = 25 * 7

// three.js plane facing the robot (normal -x in the MuJoCo frame), text running toward the robot's right
const FACING_ROBOT = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0),
))

const _v = new THREE.Vector3()
const _q = new THREE.Quaternion()
const _inv = new THREE.Matrix4()
const _rootQinv = new THREE.Quaternion()
const _eye = new THREE.Vector3()

export function MujocoScene({ vrMode = 'unlocked' }) {
  const { gl, camera } = useThree()
  const worldRef = useRef()
  const workerRef = useRef(null)
  const [world, setWorld] = useState(null)
  const latest = useRef(null)
  const applied = useRef(null)
  const saved = useRef({ total: 0, success: 0 })
  const modeRef = useRef(vrMode)
  modeRef.current = vrMode
  const xr = useRef({ session: null, calibrated: false, hands: [newHandState(), newHandState()] })
  const input = useRef({ input: new Float32Array(INPUT_SIZE), raw: new Float32Array(RAW_SIZE) })

  useEffect(() => {
    const worker = new Worker(new URL('../sim/sim.worker.js', import.meta.url), { type: 'module' })
    workerRef.current = worker
    setStatusText('Loading physics…')
    worker.onmessage = ({ data }) => {
      if (data.type === 'state') latest.current = data
      else if (data.type === 'ready') {
        setWorld(buildWorld(data))
        setStatusText(`Physics ready · MuJoCo ${data.timestep * 1000} ms step`)
      } else if (data.type === 'episode') saveEpisode(data.header, data.buffer)
      else if (data.type === 'error') {
        console.error('[sim]', data.message)
        setStatusText('Physics failed to load — see console')
      }
    }
    worker.postMessage({
      type: 'init',
      baseUrl: new URL(import.meta.env.BASE_URL, location.href).href,
      timestep: Number(params.get('dt')) || undefined,
      autopilot: params.has('autopilot'),
      session: SESSION_ID,
    })
    const unsubscribe = onEpisodesChanged(s => { saved.current = s })
    return () => { worker.terminate(); unsubscribe() }
  }, [])

  // Desktop preview camera: front-left of the robot, or the operator's view with ?view=eye.
  // WebXR overrides it with the headset pose.
  useEffect(() => {
    if (gl.xr.isPresenting || !world) return
    if (params.get('view') === 'eye') {
      world.root.updateWorldMatrix(true, false)
      camera.position.copy(world.root.localToWorld(new THREE.Vector3().fromArray(world.eye)))
      camera.lookAt(world.root.localToWorld(new THREE.Vector3(0.45, 0, 0.75)))
    } else {
      camera.position.set(-0.85, 1.5, -1.35)
      camera.lookAt(0, 0.9, -0.3)
    }
  }, [camera, gl, world])

  useEffect(() => {
    const hideHead = vrMode === 'locked' || params.get('view') === 'eye'
    if (world) world.headMeshes.forEach(m => { m.visible = !hideHead })
  }, [world, vrMode])

  useFrame((_state, delta, xrFrame) => {
    if (!world) return
    const s = latest.current
    if (s && s !== applied.current) {
      applied.current = s
      applyBodies(world, s.bodies)
      applyInfo(world, s.info, saved.current)
    }
    if (!xrFrame) return

    const session = gl.xr.getSession()
    const refSpace = gl.xr.getReferenceSpace()
    if (!session || !refSpace) return
    const st = xr.current
    if (st.session !== session) {
      st.session = session
      st.calibrated = false
      st.hands.forEach(h => h.lastSeen = -Infinity)
    }

    _eye.fromArray(world.eye)
    if (!st.calibrated) {
      calibrate(modeRef.current, camera, worldRef.current, world.root, _eye)
      st.calibrated = true
    } else if (modeRef.current === 'locked') {
      followEye(camera, worldRef.current, world.root, _eye)
    }

    const { input: inp, raw } = input.current
    readOperator(xrFrame, session, refSpace, camera, world.root, st.hands, delta, inp, raw)
    workerRef.current?.postMessage({ type: 'input', input: inp, raw })
  })

  return (
    <>
      <directionalLight position={[5, 10, 7]} intensity={4} />
      <directionalLight position={[-4, 6, -3]} intensity={2} />
      <directionalLight position={[0, 4, 8]} intensity={2} color="#eeeeff" />
      <ambientLight intensity={2.0} />
      <hemisphereLight skyColor="#aaccee" groundColor="#555555" intensity={1.5} />
      <group ref={worldRef}>
        <gridHelper args={[30, 60, '#5588aa', '#445566']} position={[0, 0.001, 0]} />
        {world && <primitive object={world.root} />}
      </group>
    </>
  )
}

// ── Scene construction ──────────────────────────────────────────────────────

function buildWorld({ scene, eye, task }) {
  const root = new THREE.Group()
  root.quaternion.copy(ROBOT_BASE_QUAT)

  const bodies = scene.bodies.map(name => {
    const g = new THREE.Group()
    g.name = name
    root.add(g)
    return g
  })
  const handMaterials = [MAT_BODY.clone(), MAT_BODY.clone()]
  const handAccents = [MAT_ACCENT.clone(), MAT_ACCENT.clone()]
  const headMeshes = []
  const meshCache = new Map()

  for (const geom of scene.geoms) {
    const bodyName = scene.bodies[geom.body]
    const meshName = geom.mesh >= 0 ? scene.meshes[geom.mesh].name : ''
    let material
    if (SCENE_BODIES.has(bodyName)) {
      const [r, g, b, a] = geom.rgba
      material = new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
        roughness: geom.type === 0 ? 0.9 : 0.6, transparent: a < 1, opacity: a,
      })
    } else {
      const accent = ACCENT_MESH.test(meshName)
      const hand = bodyName.startsWith('left_hand_') || bodyName === 'left_wrist_yaw_link' ? 0
        : bodyName.startsWith('right_hand_') || bodyName === 'right_wrist_yaw_link' ? 1 : -1
      material = hand >= 0 ? (accent ? handAccents : handMaterials)[hand] : accent ? MAT_ACCENT : MAT_BODY
    }
    const mesh = new THREE.Mesh(geomGeometry(geom, scene.meshes, meshCache), material)
    mesh.position.fromArray(geom.pos)
    mesh.quaternion.set(geom.quat[1], geom.quat[2], geom.quat[3], geom.quat[0])
    if (meshName === 'head_link') headMeshes.push(mesh)
    bodies[geom.body].add(mesh)
  }

  const hud = makeHud()
  hud.mesh.position.set(0.8, 0, 1.04)
  hud.mesh.quaternion.copy(FACING_ROBOT)
  root.add(hud.mesh)

  const reset = makeResetButton()
  reset.group.position.fromArray(task.resetButton)
  root.add(reset.group)

  return { root, bodies, eye, handMaterials: [[handMaterials[0], handAccents[0]], [handMaterials[1], handAccents[1]]], headMeshes, hud, reset }
}

function geomGeometry(g, meshes, cache) {
  const [s0, s1, s2] = g.size
  switch (g.type) {
    case 0: return new THREE.PlaneGeometry(s0 > 0 ? 2 * s0 : 30, s1 > 0 ? 2 * s1 : 30)
    case 2: return new THREE.SphereGeometry(s0, 24, 16)
    case 3: return new THREE.CapsuleGeometry(s0, 2 * s1, 8, 16).rotateX(Math.PI / 2)
    case 4: return new THREE.SphereGeometry(1, 24, 16).scale(s0, s1, s2)
    case 5: return new THREE.CylinderGeometry(s0, s0, 2 * s1, 32).rotateX(Math.PI / 2)
    case 6: return new THREE.BoxGeometry(2 * s0, 2 * s1, 2 * s2)
    case 7: {
      if (!cache.has(g.mesh)) {
        const { vert, face } = meshes[g.mesh]
        const geo = new THREE.BufferGeometry()
        geo.setAttribute('position', new THREE.BufferAttribute(vert, 3))
        geo.setIndex(new THREE.BufferAttribute(face, 1))
        // flat shading like the STL renderer
        const flat = geo.toNonIndexed()
        flat.computeVertexNormals()
        cache.set(g.mesh, flat)
      }
      return cache.get(g.mesh)
    }
    default: return new THREE.BufferGeometry()
  }
}

function applyBodies(world, b) {
  for (let i = 0; i < world.bodies.length; i++) {
    const o = 7 * i
    const g = world.bodies[i]
    g.position.set(b[o], b[o + 1], b[o + 2])
    g.quaternion.set(b[o + 4], b[o + 5], b[o + 6], b[o + 3])
  }
}

function applyInfo(world, info, saved) {
  for (let s = 0; s < 2; s++) {
    for (const mat of world.handMaterials[s]) mat.emissive.copy(info.touching[s] ? TOUCH_EMISSIVE : NO_EMISSIVE)
  }
  world.reset.setProgress(info.resetProgress)
  world.hud.draw(info, saved)
}

// ── In-scene UI ─────────────────────────────────────────────────────────────

const STATUS_TEXT = {
  waiting: ['RAISE YOUR HANDS TO START', '#9fb4c8'],
  running: ['RECORDING', '#ff5d5d'],
  success: ['SUCCESS — SAVED', '#5dff8f'],
  dropped: ['DROPPED — RESETTING', '#ffb35d'],
  timeout: ['TIMEOUT — RESETTING', '#ffb35d'],
  aborted: ['RESET', '#9fb4c8'],
}

function makeHud() {
  const canvas = document.createElement('canvas')
  canvas.width = 1024
  canvas.height = 512
  const ctx = canvas.getContext('2d')
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(0.44, 0.22),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, toneMapped: false }),
  )
  let last = ''
  const draw = (info, saved) => {
    const [label, color] = STATUS_TEXT[info.status] ?? [info.status.toUpperCase(), '#ffffff']
    const lines = [
      label + (info.status === 'running' ? `  ${info.elapsed.toFixed(1)} s` : ''),
      `episode ${info.episode}   saved ${saved.total} (${saved.success} ok)`,
      `physics ${info.rtf.toFixed(2)}× real time · ${info.msPerStep.toFixed(2)} ms/step`,
    ]
    const key = lines.join('|') + color
    if (key === last) return
    last = key
    ctx.clearRect(0, 0, 1024, 512)
    ctx.fillStyle = 'rgba(10, 16, 24, 0.78)'
    ctx.beginPath()
    ctx.roundRect(8, 8, 1008, 496, 36)
    ctx.fill()
    ctx.fillStyle = '#ffffff'
    ctx.font = '600 54px system-ui, sans-serif'
    ctx.fillText('Put the tube in the box', 56, 112)
    ctx.fillStyle = color
    ctx.font = '700 64px system-ui, sans-serif'
    ctx.fillText(lines[0], 56, 228)
    ctx.fillStyle = '#c8d4e0'
    ctx.font = '400 40px system-ui, sans-serif'
    ctx.fillText(lines[1], 56, 338)
    ctx.fillStyle = info.rtf < 0.95 ? '#ffb35d' : '#7f93a6'
    ctx.font = '400 34px system-ui, sans-serif'
    ctx.fillText(lines[2], 56, 432)
    texture.needsUpdate = true
  }
  return { mesh, draw }
}

function makeResetButton() {
  const group = new THREE.Group()
  const material = new THREE.MeshStandardMaterial({ color: 0xc0392b, roughness: 0.4, emissive: 0x220000 })
  group.add(new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16), material))

  const canvas = document.createElement('canvas')
  canvas.width = 256
  canvas.height = 64
  const ctx = canvas.getContext('2d')
  ctx.fillStyle = '#ffffff'
  ctx.font = '600 30px system-ui, sans-serif'
  ctx.textAlign = 'center'
  ctx.fillText('HOLD TO RESET', 128, 42)
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  const label = new THREE.Mesh(
    new THREE.PlaneGeometry(0.12, 0.03),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, toneMapped: false }),
  )
  label.position.set(0, 0, 0.055)
  label.quaternion.copy(FACING_ROBOT)
  group.add(label)

  const idle = new THREE.Color(0xc0392b)
  const full = new THREE.Color(0xffffff)
  return {
    group,
    setProgress: p => {
      material.color.copy(idle).lerp(full, p)
      group.scale.setScalar(1 + 0.3 * p)
    },
  }
}

// ── Calibration (same behaviour as the kinematic app) ───────────────────────

function calibrate(mode, camera, worldGroup, root, eyeLocal) {
  const euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ')
  worldGroup.position.set(0, 0, 0)
  worldGroup.rotation.set(0, euler.y, 0)
  worldGroup.updateMatrixWorld(true)
  const eye = root.localToWorld(eyeLocal.clone())
  worldGroup.position.x += camera.position.x - eye.x
  worldGroup.position.z += camera.position.z - eye.z
  if (mode === 'locked') worldGroup.position.y += camera.position.y - eye.y
  worldGroup.updateMatrixWorld(true)
}

function followEye(camera, worldGroup, root, eyeLocal) {
  worldGroup.updateMatrixWorld(true)
  const eye = root.localToWorld(eyeLocal.clone())
  worldGroup.position.x += camera.position.x - eye.x
  worldGroup.position.z += camera.position.z - eye.z
  worldGroup.updateMatrixWorld(true)
}

// ── Operator input ──────────────────────────────────────────────────────────

function newHandState() {
  const joints = {}
  for (const n of XR_JOINT_NAMES) joints[n] = { position: new THREE.Vector3(), quaternion: new THREE.Quaternion() }
  return {
    joints,
    present: {},
    pos: new OneEuroVector3(),
    quat: new QuaternionSmoother(0.5),
    fingers: new RetargetingFilter(0.4),
    corrected: new THREE.Quaternion(),
    lastSeen: -Infinity,
  }
}

// Writes position + quaternion (w, x, y, z) of a three.js world pose in the MuJoCo frame
function writePose(p, q, out, o) {
  _v.copy(p).applyMatrix4(_inv)
  out[o] = _v.x; out[o + 1] = _v.y; out[o + 2] = _v.z
  _q.copy(_rootQinv).multiply(q)
  out[o + 3] = _q.w; out[o + 4] = _q.x; out[o + 5] = _q.y; out[o + 6] = _q.z
}

function readOperator(xrFrame, session, refSpace, camera, root, hands, dt, input, raw) {
  root.updateWorldMatrix(true, false)
  _inv.copy(root.matrixWorld).invert()
  root.getWorldQuaternion(_rootQinv).invert()
  input.fill(0)
  raw.fill(0)
  writePose(camera.position, camera.quaternion, raw, 0)
  const now = performance.now() / 1000

  for (const source of session.inputSources) {
    if (!source.hand) continue
    const s = source.handedness === 'left' ? 0 : source.handedness === 'right' ? 1 : -1
    if (s < 0) continue
    const h = hands[s]
    const joints = {}
    XR_JOINT_NAMES.forEach((name, i) => {
      const space = source.hand.get(name)
      const pose = space && xrFrame.getJointPose(space, refSpace)
      if (!pose) return
      const j = h.joints[name]
      const { position: p, orientation: q } = pose.transform
      j.position.set(p.x, p.y, p.z)
      j.quaternion.set(q.x, q.y, q.z, q.w)
      joints[name] = j
      writePose(j.position, j.quaternion, raw, 7 + s * RAW_HAND + 7 * i)
    })
    const wrist = joints.wrist
    if (!wrist) continue

    // Re-acquired after a dropout: start the filters fresh instead of sweeping from the old pose
    if (now - h.lastSeen > 0.2) { h.pos.reset(); h.quat.reset(); h.fingers.reset() }
    h.lastSeen = now

    const o = s * HAND_INPUT
    input[o] = 1
    const pos = h.pos.update(wrist.position, dt)
    h.corrected.copy(wrist.quaternion).multiply(CORRECTION[s])
    writePose(pos, h.quat.update(h.corrected), input, o + 1)
    const f = h.fingers.update(retargetHand(joints))
    input.set([f.thumb.abduction, f.thumb.curl[0], f.thumb.curl[1], f.index.curl[0], f.index.curl[1],
      f.middle.curl[0], f.middle.curl[1]], o + 8)
  }
}

function setStatusText(text) {
  const el = document.getElementById('sim-status')
  if (el) el.textContent = text
}
